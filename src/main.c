#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <conio.h>
#include <stdbool.h>
#include <dos.h>
#include <stdarg.h>
#include <dpmi/dbgutil.h>
#include <sbemucfg.h>
#include <pic.h>
#include <opl3emu.h>
#include <vdma.h>
#include <virq.h>
#include <sbemu.h>
#include <untrapio.h>
#include <vmpu.h>
#include <vpcspeaker.h>
#include "qemm.h"
#include "hdpmipt.h"
#include "serial.h"
#include "irqguard.h"
#include "utility.h"
#include <math.h>
#include <go32.h>
#include <sys/farptr.h>

#include <au_cards/au_cards.h>
#include <au_cards/pcibios.h>

#ifdef AU_CARDS_LINK_GEODE
extern one_sndcard_info GEODE_sndcard_info;
extern int GEODE_telemetry_read(struct mpxplay_audioout_info_s *aui,
                                unsigned int selector,uint32_t *value);
extern void GEODE_set_isr_context(int isr);
extern int GEODE_isr_context_failed(void);
extern int GEODE_set_cdin_isr(struct mpxplay_audioout_info_s *aui, unsigned int percent);
#endif
#ifdef AU_CARDS_LINK_GEODE
#endif

static const char *
PROGNAME = "SBEMU";


#ifndef MAIN_SBEMU_VER
#define MAIN_SBEMU_VER "1.0 beta3"
#endif
#ifndef GEODE_AUTHOR
#define GEODE_AUTHOR "OrdinaryLunatic"
#endif
#ifndef GEODE_URL
#define GEODE_URL "https://github.com/OrdinaryLunatic/SBGEODE"
#endif

#define MAIN_TRAP_PMPIC_ONDEMAND 0 //now we need a Virtual PIC to hide some IRQ for protected mode games
#define MAIN_TRAP_RMPIC_ONDEMAND 1 //don't hide IRQ for rm, as some driver(i.e.usbuhci) will use it
#define MAIN_INSTALL_RM_ISR 1 //not needed. but to workaround some rm games' problem. need RAW_HOOk in dpmi_dj2.c - disble for more tests.
#define MAIN_ISR_CHAINED 0 //auto calls next handler AFTER current handler exits - cause more mode switches, disable for more tests.
#define MAIN_VMPU_HDPMI_MEMFIX 1 //workaround hdpmi bug that a TSR allocated memory being treated as primary client's, also better by avoiding DPMI memory fragments
                            //details on '_freeclientmemory' at I31MEM.ASM of HDPMI.
#define MAIN_PCM_RESAMPLE_INTERPOLATION 0

#define MAIN_SETIF 1 //merge form vdpmi branch, not suree if this works well enough

//note: use __dpmi_get_and_disable_virtual_interrupt_state()/__dpmi_get_and_enable_virtual_interrupt_state() causes crash.
//needs to find why HDPMI traps #GP well but int31h not not working.
#if MAIN_SETIF
#define SETIF()  __asm__ __volatile__("sti") //STI() //call __dpmi_get_and_disable_virtual_interrupt_state() directly
#define RESETIF() __asm__ __volatile__("cli") //CLI() //call __dpmi_get_and_enable_virtual_interrupt_state() directly
#else
#define SETIF() 
#define RESETIF() 
#endif

#define MAIN_TSR_INT 0x2D   //AMIS multiplex. TODO: 0x2F?
#define MAIN_TSR_INTSTART_ID 0x01 //start id

#if SBEMU_VMPU 

#define VMPU_ENABLED (MAIN_Options[OPT_VMPU_VOICES].value)

#if MAIN_VMPU_HDPMI_MEMFIX

#define MAIN_VMPU_MEMSIZE (7*1024*1024) //OK for a 4M SF and a 6M (Yamaha XG) SF with TSF_FLOAT_LUT

static void* VMPU_Mem; //pre-alloc mem. this works because free() does not return dpmi memory immediately to dpmi host.

#define MAIN_VMPU_ALLOCMEM() while(!VMPU_Mem) { \
    VMPU_Mem = malloc(MAIN_VMPU_MEMSIZE); \
    if(!VMPU_Mem) { MAIN_CPrintf(RED, "Failed to allocate memory.\n"); exit(-1); } \
}

#define MAIN_VMPU_FREEMEM() do { \
    if(VMPU_Mem) {free(VMPU_Mem); VMPU_Mem = NULL; } \
} while(0)
#endif //MAIN_VMPU_HDPMI_MEMFIX

#else //SBEMU_VMPU

#define VMPU_ENABLED FALSE

#endif

static mpxplay_audioout_info_s aui = {0};
static mpxplay_audioout_info_s fm_aui = {0};
static mpxplay_audioout_info_s mpu401_aui = {0};

#define MAIN_PCM_SAMPLESIZE 16384

static int16_t MAIN_OPLPCM[MAIN_PCM_SAMPLESIZE];
static int16_t MAIN_PCM[MAIN_PCM_SAMPLESIZE];
static int16_t MAIN_PCMResample[MAIN_PCM_SAMPLESIZE];
static int MAIN_LastSBRate = 0;

/* Geode: keep OPL running after the last key-off so release tails finish
   instead of being cut to digital silence (chip time otherwise freezes). */
#define MAIN_OPL_TAIL_SECONDS 3
static int MAIN_OPLTail = 0; //samples left to generate after the last key-on

/* Geode: ISR cost telemetry in TSC cycles (read through the multiplex call).
   Counters wrap at 32 bits and are only updated from MAIN_Interrupt(). */
static uint32_t MAIN_IsrMax = 0;
static uint64_t MAIN_IsrSum = 0;     //cycles, 64 bit (32 bit wrapped after ~40 min)
static uint32_t MAIN_IsrCount = 0;
static inline uint32_t MAIN_TSC(void)
{
    uint32_t lo, hi;
    __asm__ __volatile__("rdtsc" : "=a"(lo), "=d"(hi));
    return lo;
}

/* Geode: saturate instead of wrapping the int16 mix on overload. */
static inline int16_t MAIN_Sat16(int v)
{
    return (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
}

/* Geode: anti-image filter for digital PCM. SB DMA data (often 7-22 kHz) is
   linearly interpolated to the card rate, which leaves strong mirror images
   between fs/2 and the card Nyquist. A real SB16 filters its DAC output.
   Two cascaded Butterworth biquads (fc = 0.45 * source rate), integer
   state/coefficients only; coefficient table is built once outside the ISR. */
#define MAIN_LPF_STEP 250
#define MAIN_LPF_MAXRATE 24000
#define MAIN_LPF_ENTRIES (MAIN_LPF_MAXRATE/MAIN_LPF_STEP+1)
#define MAIN_LPF_Q 14
typedef struct { int32_t b0, b1, b2, a1, a2; } MAIN_Biquad;
typedef struct { int32_t x1, x2, y1, y2; } MAIN_BiquadState;
static MAIN_Biquad MAIN_LPFCoef[MAIN_LPF_ENTRIES][2];
static MAIN_BiquadState MAIN_LPFState[SBEMU_CHANNELS][2];
static int MAIN_LPFRate = 0; //source rate the state belongs to

static void MAIN_LPF_Build(int cardrate)
{
    static const double q[2] = {0.5411961, 1.3065630}; //4th-order Butterworth
    for(int e = 0; e < MAIN_LPF_ENTRIES; ++e)
    {
        double fc = 0.45 * (e ? e : 1) * MAIN_LPF_STEP;
        for(int k = 0; k < 2; ++k)
        {
            double w = 2.0 * M_PI * fc / cardrate;
            double alpha = sin(w) / (2.0 * q[k]);
            double cw = cos(w);
            double a0 = 1.0 + alpha;
            MAIN_LPFCoef[e][k].b0 = (int32_t)floor((1.0 - cw) / 2.0 / a0 * (1<<MAIN_LPF_Q) + 0.5);
            MAIN_LPFCoef[e][k].b1 = (int32_t)floor((1.0 - cw) / a0 * (1<<MAIN_LPF_Q) + 0.5);
            MAIN_LPFCoef[e][k].b2 = MAIN_LPFCoef[e][k].b0;
            MAIN_LPFCoef[e][k].a1 = (int32_t)floor(-2.0 * cw / a0 * (1<<MAIN_LPF_Q) + 0.5);
            MAIN_LPFCoef[e][k].a2 = (int32_t)floor((1.0 - alpha) / a0 * (1<<MAIN_LPF_Q) + 0.5);
        }
    }
}

static void MAIN_LPF_Process(int16_t* pcm, int frames, int srcrate)
{
    int e = (srcrate + MAIN_LPF_STEP/2) / MAIN_LPF_STEP;
    if(e >= MAIN_LPF_ENTRIES) return;
    if(srcrate != MAIN_LPFRate)
    {   //Geode: start in the steady state of the first frame (DC gain 1), not at 0: a block that does not start
        //at 0 made the filter ring - a click at the start of every effect
        for(int ch = 0; ch < SBEMU_CHANNELS; ++ch)
        {
            int v = frames > 0 ? pcm[ch] : 0;
            MAIN_BiquadState s = {v, v, v, v};
            MAIN_LPFState[ch][0] = MAIN_LPFState[ch][1] = s;
        }
        MAIN_LPFRate = srcrate;
    }
    for(int ch = 0; ch < SBEMU_CHANNELS; ++ch)
    {
        MAIN_BiquadState st0 = MAIN_LPFState[ch][0], st1 = MAIN_LPFState[ch][1];
        const MAIN_Biquad* c0 = &MAIN_LPFCoef[e][0];
        const MAIN_Biquad* c1 = &MAIN_LPFCoef[e][1];
        for(int i = 0; i < frames; ++i)
        {
            int16_t* p = pcm + i*SBEMU_CHANNELS + ch;
            long long acc = (long long)c0->b0*(*p) + (long long)c0->b1*st0.x1 + (long long)c0->b2*st0.x2
                          - (long long)c0->a1*st0.y1 - (long long)c0->a2*st0.y2;
            int y = (int)((acc + (1<<(MAIN_LPF_Q-1))) >> MAIN_LPF_Q);
            y = y > 32767 ? 32767 : (y < -32768 ? -32768 : y);
            st0.x2 = st0.x1; st0.x1 = *p; st0.y2 = st0.y1; st0.y1 = y;
            int x = y;
            acc = (long long)c1->b0*x + (long long)c1->b1*st1.x1 + (long long)c1->b2*st1.x2
                - (long long)c1->a1*st1.y1 - (long long)c1->a2*st1.y2;
            y = (int)((acc + (1<<(MAIN_LPF_Q-1))) >> MAIN_LPF_Q);
            y = y > 32767 ? 32767 : (y < -32768 ? -32768 : y);
            st1.x2 = st1.x1; st1.x1 = x; st1.y2 = st1.y1; st1.y1 = y;
            *p = (int16_t)y;
        }
        MAIN_LPFState[ch][0] = st0; MAIN_LPFState[ch][1] = st1;
    }
}
static int16_t MAIN_LastResample[SBEMU_CHANNELS];
/* Geode: a real Sound Blaster DAC keeps its last value when a DMA transfer ends, and its output is AC coupled.
   A game whose samples sit off the 80h centre gets no step between its sounds there.
   SBEMU output 0 then: a thump at every start and end. Keep the last digital frame (before the gains) while the
   DSP is stopped, and pass the whole digital stream - blocks and the held level - through a DC blocker of about
   2 Hz (the coupling capacitor), so the offset goes away slowly and never as a step. */
static int16_t MAIN_DacHold[2];
static int MAIN_DacHoldOn;
static int32_t MAIN_DCx1[2];
static int64_t MAIN_DCy1[2]; //Q16
static void MAIN_DCBlock(int16_t* pcm, int frames)
{   //y = x - x1 + R*y1, R = 1 - 17/65536: corner (17/65536)*48000/(2*pi) = 2.0 Hz
    for(int ch = 0; ch < 2; ++ch)
    {
        int32_t x1 = MAIN_DCx1[ch];
        int64_t y1 = MAIN_DCy1[ch];
        for(int i = 0; i < frames; ++i)
        {
            int32_t x = pcm[i*2+ch];
            y1 = ((int64_t)(x - x1) << 16) + y1 - ((y1 * 17) >> 16);
            x1 = x;
            int32_t y = (int32_t)(y1 >> 16);
            pcm[i*2+ch] = (int16_t)(y > 32767 ? 32767 : (y < -32768 ? -32768 : y));
        }
        MAIN_DCx1[ch] = x1; MAIN_DCy1[ch] = y1;
    }
}

/* Geode: continuous-phase linear resampler for the digital (DMA) path.
   mixer_speed_lq restarted at phase 0 for every ~6 ms chunk and repeated the
   last sample, which put a 1/period (about 167 Hz) modulation on every PCM
   effect. This keeps the fractional position and the previous frame across
   calls, so the stream is interpolated as if it were one block. */
static int16_t MAIN_RSLast[SBEMU_CHANNELS];  //e[0]: last frame of the previous chunk
static int16_t MAIN_RSLast2[SBEMU_CHANNELS]; //e[-1]: the frame before it
/* /FILT=2: 8-tap Kaiser-windowed sinc (beta 6, cutoff = source Nyquist), 1024
   phases, Q14. Images of 44.1 kHz material fall from -32 dBc (linear) to
   -64 dBc at 6 kHz; costs 16 MAC per frame. The kernel for position i uses
   e[i-3..i+4], so output lags the source by 4 frames. History keeps 8 frames
   (e[-7..0]): a call may end with the next position still at frames-4 (step
   < 1), which is i = -4 in the next call and needs e[-7]. Table is built once
   at start (MAIN_Resample_Build). */
#define MAIN_RS_TAPS 8
#define MAIN_RS_PHASES 1024
#define MAIN_RS_HIST 8
static int16_t MAIN_RSTab[MAIN_RS_PHASES+1][MAIN_RS_TAPS];
static int16_t MAIN_RSHist[MAIN_RS_HIST][SBEMU_CHANNELS]; //[j] = e[-j]
static int16_t MAIN_RSWork[(MAIN_RS_HIST + 2048) * SBEMU_CHANNELS];
static int MAIN_RSHQ = 0; //1 = /FILT=2 (set per interrupt)
static double MAIN_BesselI0(double x)
{
    double sum = 1.0, term = 1.0;
    for(int k = 1; k < 40; ++k)
    {
        term *= (x / (2.0 * k)) * (x / (2.0 * k));
        sum += term;
    }
    return sum;
}
static void MAIN_Resample_Build(void)
{
    const double beta = 6.0;
    for(int p = 0; p <= MAIN_RS_PHASES; ++p)
    {
        double fr = (double)p / MAIN_RS_PHASES, h[MAIN_RS_TAPS], sum = 0.0;
        for(int k = 0; k < MAIN_RS_TAPS; ++k)
        {
            double x = (k - 3) - fr; //offset of e[i-3+k] from the output position
            double sinc = fabs(x) < 1e-9 ? 1.0 : sin(M_PI * x) / (M_PI * x);
            double r = x / (MAIN_RS_TAPS/2), w = 0.0;
            if(fabs(r) < 1.0)
                w = MAIN_BesselI0(beta * sqrt(1.0 - r*r)) / MAIN_BesselI0(beta);
            h[k] = sinc * w;
            sum += h[k];
        }
        for(int k = 0; k < MAIN_RS_TAPS; ++k)
            MAIN_RSTab[p][k] = (int16_t)floor(h[k] / sum * 16384.0 + 0.5);
    }
}
static int64_t MAIN_RSPos; //16.16 position in e[] coordinates, may be in [-1, 0) units
static int MAIN_RSValid = 0;
static void MAIN_ResampleReset(void)
{
    MAIN_RSValid = 0;
    MAIN_RSPos = 0;
    MAIN_RSLast[0] = MAIN_RSLast[1] = MAIN_RSLast2[0] = MAIN_RSLast2[1] = 0;
    memset(MAIN_RSHist, 0, sizeof(MAIN_RSHist));
}
//source frames needed to produce exactly 'want' output frames from here
static int MAIN_ResampleNeed(int want, uint32_t srcrate, uint32_t dstrate)
{
    if(want <= 0) return 0;
    const int64_t step = (int64_t)((((uint64_t)srcrate << 16)) / dstrate);
    const int64_t p = MAIN_RSValid ? MAIN_RSPos : 0;
    int64_t n = ((p + (int64_t)(want-1)*step) >> 16) + 1 + (MAIN_RSHQ ? 3 : 0); //8-tap kernel: e[i+4] must be present
    return n < 0 ? 0 : (int)n;
}
//src: frames*channels samples (frames may be 0), dest capacity in int16 units,
//maxout frames wanted; returns output frames. /FILT=2 selects the 8-tap kernel.
static int MAIN_ResampleStream(int16_t* dest, int destcap, int maxout, const int16_t* src, int frames, int channels, uint32_t srcrate, uint32_t dstrate)
{
    const int64_t step = (int64_t)((((uint64_t)srcrate << 16)) / dstrate);
    const int hq = MAIN_RSHQ && frames <= 2048;
    const int64_t lastpos = (int64_t)frames - (hq ? 3 : 0); //first position that cannot be produced yet
    int out = 0;
    if(!MAIN_RSValid)
    {
        if(frames <= 0) return 0;
        for(int c = 0; c < channels; ++c)
        {
            MAIN_RSLast[c] = MAIN_RSLast2[c] = src[c];
            for(int j = 0; j < MAIN_RS_HIST; ++j) MAIN_RSHist[j][c] = src[c];
        }
        MAIN_RSPos = 0;
        MAIN_RSValid = 1;
    }
    int64_t p = MAIN_RSPos;
    if(hq)
    {   //work[j] = e[j-7]: history then the new frames, so e[i] = work[(i+7)*channels]
        for(int j = 0; j < MAIN_RS_HIST; ++j)
            for(int c = 0; c < channels; ++c)
                MAIN_RSWork[j*channels + c] = MAIN_RSHist[MAIN_RS_HIST-1-j][c];
        memcpy(MAIN_RSWork + MAIN_RS_HIST*channels, src, frames*channels*sizeof(int16_t));
        //e[i-3+k] for output i (>= -4): index (i-3+k+7) = i+4+k >= 0
        while((p >> 16) < lastpos && (out+1)*channels <= destcap && out < maxout)
        {
            const int64_t i = p >> 16;
            const int16_t* tab = MAIN_RSTab[((int)(p & 0xFFFF) + 32) >> 6];
            const int16_t* w = MAIN_RSWork + (i+4)*channels;
            for(int c = 0; c < channels; ++c)
            {
                int acc = 8192;
                for(int k = 0; k < MAIN_RS_TAPS; ++k)
                    acc += tab[k] * w[k*channels + c];
                dest[out*channels + c] = MAIN_Sat16(acc >> 14);
            }
            ++out;
            p += step;
        }
    }
    else //e[-1]=last2, e[0]=last, e[j]=src[j-1]; output i needs e[i], e[i+1]
    while((p >> 16) < lastpos && (out+1)*channels <= destcap && out < maxout)
    {
        int64_t i = p >> 16;
        int m2 = (int)((p >> 4) & 0xFFF), m1 = 4096 - m2;
        for(int c = 0; c < channels; ++c)
        {
            int a = i < 0 ? MAIN_RSLast2[c] : (i == 0 ? MAIN_RSLast[c] : src[(i-1)*channels + c]);
            int b = i < 0 ? MAIN_RSLast[c]  : src[i*channels + c];
            dest[out*channels + c] = (int16_t)((a*m1 + b*m2) >> 12);
        }
        ++out;
        p += step;
    }
    MAIN_RSPos = p - ((int64_t)frames << 16);
    if(frames > 0)
        for(int c = 0; c < channels; ++c)
        {
            //new history e[-j] = the j-th frame back from the end of (old history + src)
            for(int j = MAIN_RS_HIST-1; j >= 0; --j) //descending: reads the old, lower entries
            {
                int k = frames - 1 - j; //index into src
                MAIN_RSHist[j][c] = k >= 0 ? src[k*channels + c] : MAIN_RSHist[j-frames][c];
            }
            MAIN_RSLast2[c] = frames >= 2 ? src[(frames-2)*channels + c] : MAIN_RSLast[c];
            MAIN_RSLast[c] = src[(frames-1)*channels + c];
        }
    return out;
}

static DPMI_ISR_HANDLE MAIN_IntHandlePM;
static DPMI_ISR_HANDLE MAIN_IntHandleRM;
static DPMI_REG MAIN_RMIntREG;
static INTCONTEXT MAIN_IntContext;
static uint32_t MAIN_DMA_Addr = 0;
static uint32_t MAIN_DMA_Size = 0;
static uint32_t MAIN_DMA_MappedAddr = 0;
static uint8_t MAIN_QEMM_Present = 0;
static uint8_t MAIN_HDPMI_Present = 0;
static uint8_t MAIN_InINT;
static volatile uint32_t MAIN_VirtualIRQCount;
#define MAIN_ININT_PM 0x01
#define MAIN_ININT_RM 0x02

SBEMU_EXTFUNS MAIN_SbemuExtFun;

static void MAIN_Interrupt();
static void MAIN_InterruptPM();
static void MAIN_InterruptRM();

static DPMI_ISR_HANDLE MAIN_TSRIntHandle;
static DPMI_REG MAIN_TSRREG;
static uint32_t MAIN_TSR_INT_FNO = MAIN_TSR_INTSTART_ID;
static uint32_t MAIN_ISR_DOSID;
static const char MAIN_ISR_DOSID_String[] = "Crazii  SBEMU   Sound Blaster emulation on AC97"; //8:8:asciiz
static void MAIN_TSR_InstallationCheck();
static void MAIN_TSR_Interrupt();

#define HW_FM_HANDLER(n) do {\
  if (fm_aui.fm) {                                                      \
    if (out) {                                                          \
      if (fm_aui.card_handler->card_fm_write) {                         \
        fm_aui.card_handler->card_fm_write(&fm_aui, n, (uint8_t)(val & 0xff)); \
        return val;                                                     \
      }                                                                 \
    } else {                                                            \
      if (fm_aui.card_handler->card_fm_read)                            \
        return fm_aui.card_handler->card_fm_read(&fm_aui, n);           \
    }                                                                   \
  }                                                                     \
  return 0;                                                             \
  } while (0)

static uint32_t MAIN_OPL3_388(uint32_t port, uint32_t val, uint32_t out)
{
  return out ? OPL3EMU_PrimaryWriteIndex(val) : OPL3EMU_PrimaryRead(val);
}
static uint32_t MAIN_OPL3_389(uint32_t port, uint32_t val, uint32_t out)
{
  return out ? OPL3EMU_PrimaryWriteData(val) : OPL3EMU_PrimaryRead(val);
}
static uint32_t MAIN_OPL3_38A(uint32_t port, uint32_t val, uint32_t out)
{
  return out ? OPL3EMU_SecondaryWriteIndex(val) : OPL3EMU_SecondaryRead(val);
}
static uint32_t MAIN_OPL3_38B(uint32_t port, uint32_t val, uint32_t out)
{
  return out ? OPL3EMU_SecondaryWriteData(val) : OPL3EMU_SecondaryRead(val);
}

static uint32_t MAIN_HW_OPL3_388(uint32_t port, uint32_t val, uint32_t out)
{
  HW_FM_HANDLER(0);
}
static uint32_t MAIN_HW_OPL3_389(uint32_t port, uint32_t val, uint32_t out)
{
  HW_FM_HANDLER(1);
}
static uint32_t MAIN_HW_OPL3_38A(uint32_t port, uint32_t val, uint32_t out)
{
  HW_FM_HANDLER(2);
}
static uint32_t MAIN_HW_OPL3_38B(uint32_t port, uint32_t val, uint32_t out)
{
  HW_FM_HANDLER(3);
}

static uint32_t MAIN_DMA(uint32_t port, uint32_t val, uint32_t out)
{
    return out ? (VDMA_Write(port, val), val) : (val &=~0xFF, val |= VDMA_Read(port));
}

static uint32_t MAIN_IRQ(uint32_t port, uint32_t val, uint32_t out)
{
    return out ? (VIRQ_Write(port, val), val) : (val &=~0xFF, val |= VIRQ_Read(port));
}

static uint32_t MAIN_SB_MixerAddr(uint32_t port, uint32_t val, uint32_t out)
{
    return out ? (SBEMU_Mixer_WriteAddr(port, val), val) : val;
}
static uint32_t MAIN_SB_MixerData(uint32_t port, uint32_t val, uint32_t out)
{
    return out ? (SBEMU_Mixer_Write(port, val), val) : (val &=~0xFF, val |= SBEMU_Mixer_Read(port));
}
static uint32_t MAIN_SB_DSP_Reset(uint32_t port, uint32_t val, uint32_t out)
{
    return out ? (SBEMU_DSP_Reset(port, val), val) : val;
}
static uint32_t MAIN_SB_DSP_Read(uint32_t port, uint32_t val, uint32_t out)
{
    return out ? val : (val &=~0xFF, val |= SBEMU_DSP_Read(port));
}
static uint32_t MAIN_SB_DSP_Write(uint32_t port, uint32_t val, uint32_t out)
{
    return out ? (SBEMU_DSP_Write(port, val), val) : SBEMU_DSP_WriteStatus(port);
}
static uint32_t MAIN_SB_DSP_ReadStatus(uint32_t port, uint32_t val, uint32_t out)
{
    return out ? val : (val &=~0xFF, val |= SBEMU_DSP_ReadStatus(port));
}
static uint32_t MAIN_SB_DSP_ReadINT16BitACK(uint32_t port, uint32_t val, uint32_t out)
{
    return out ? val : (val &=~0xFF, val |= SBEMU_DSP_INT16ACK(port));
}

int mpu_state = 0;
#define MPU_DEBUG 1
#if MPU_DEBUG
static int mpu_debug = 0;
static int mpu_dbg_ctr = 0;
#endif

static uint32_t MAIN_MPU_330(uint32_t port, uint32_t val, uint32_t out)
{
#if MPU_DEBUG
  if (mpu_debug) {
    if (out) {
      if (mpu_debug >= 2) {
        mpu_dbg_ctr++;
        char c = ' ';
        if (mpu_dbg_ctr == 26) {
          c = '\n';
          mpu_dbg_ctr = 0;
        }
        DBG_Logi("%02x%c", val, c);
      }
    } else {
      DBG_Logi("r%x\n", mpu_state);
    }
  }
#endif
  if (out) {
    if (mpu401_aui.mpu401 && mpu401_aui.card_handler->card_mpu401_write)
      mpu401_aui.card_handler->card_mpu401_write(&mpu401_aui, 0, (uint8_t)(val & 0xff));
    ser_putbyte((int)(val & 0xff));
    return 0;
  } else {
    uint8_t val, hwval, hwval_valid = 0;
    if (mpu401_aui.mpu401 && mpu401_aui.card_handler->card_mpu401_read) {
      hwval = mpu401_aui.card_handler->card_mpu401_read(&mpu401_aui, 0);
      if (!mpu401_aui.mpu401_softread)
        hwval_valid = 1;
    }
    if (mpu_state == 1) {
      mpu_state = 0;
      val = 0xfe;
    } else if (mpu_state == 2) {
      mpu_state = 4;
      val = 0xfe;
    } else {
      val = 0;
    }
    if (hwval_valid)
      return hwval;
    else
      return val;
  }
}
static uint32_t MAIN_MPU_331(uint32_t port, uint32_t val, uint32_t out)
{
#if MPU_DEBUG
  if (mpu_debug) {
    if (out) {
      DBG_Logi("s%x\n", val);
    } else {
      if (mpu_dbg_ctr < 10 && mpu_state <= 2) {
        DBG_Logi("sr%x\n", mpu_state);
        mpu_dbg_ctr++;
      }
    }
  }
#endif
  if (out) {
    if (mpu401_aui.mpu401 && mpu401_aui.card_handler->card_mpu401_write)
      mpu401_aui.card_handler->card_mpu401_write(&mpu401_aui, 1, (uint8_t)(val & 0xff));
    if (val == 0xff) { // Reset
#if MPU_DEBUG
      mpu_dbg_ctr = 0;
#endif
      mpu_state = 1;
    } else if (val == 0x3f) { // UART mode
#if MPU_DEBUG
      mpu_dbg_ctr = 0;
#endif
      mpu_state = 2;
    }
    return 0;
  }
  if (mpu401_aui.mpu401 && mpu401_aui.card_handler->card_mpu401_read) {
    uint8_t hwval = mpu401_aui.card_handler->card_mpu401_read(&mpu401_aui, 1);
    if (!mpu401_aui.mpu401_softread)
      return hwval;
  }
  if ((mpu_state & 3) == 0) {
    return 0x80;
  } else {
    return 0;
  }
}

static QEMM_IODT MAIN_MPUIODT[2] =
{
    0x330, &MAIN_MPU_330,
    0x331, &MAIN_MPU_331
};

static QEMM_IODT MAIN_OPL3IODT[4] =
{
    0x388, &MAIN_OPL3_388,
    0x389, &MAIN_OPL3_389,
    0x38A, &MAIN_OPL3_38A,
    0x38B, &MAIN_OPL3_38B
};

static QEMM_IODT MAIN_HW_OPL3IODT[4] =
{
    0x388, &MAIN_HW_OPL3_388,
    0x389, &MAIN_HW_OPL3_389,
    0x38A, &MAIN_HW_OPL3_38A,
    0x38B, &MAIN_HW_OPL3_38B
};

static QEMM_IODT MAIN_VDMA_IODT[40] =
{
    0x00, &MAIN_DMA,
    0x01, &MAIN_DMA,
    0x02, &MAIN_DMA,
    0x03, &MAIN_DMA,
    0x04, &MAIN_DMA,
    0x05, &MAIN_DMA,
    0x06, &MAIN_DMA,
    0x07, &MAIN_DMA,
    0x08, &MAIN_DMA,
    0x09, &MAIN_DMA,
    0x0A, &MAIN_DMA,
    0x0B, &MAIN_DMA,
    0x0C, &MAIN_DMA,
    0x0D, &MAIN_DMA,
    0x0E, &MAIN_DMA,
    0x0F, &MAIN_DMA,
    0x81, &MAIN_DMA,
    0x82, &MAIN_DMA,
    0x83, &MAIN_DMA,
    0x87, &MAIN_DMA,

    0xC0, &MAIN_DMA,
    0xC2, &MAIN_DMA,
    0xC4, &MAIN_DMA,
    0xC6, &MAIN_DMA,
    0xC8, &MAIN_DMA,
    0xCA, &MAIN_DMA,
    0xCC, &MAIN_DMA,
    0xCE, &MAIN_DMA,
    0xD0, &MAIN_DMA,
    0xD2, &MAIN_DMA,
    0xD4, &MAIN_DMA,
    0xD6, &MAIN_DMA,
    0xD8, &MAIN_DMA,
    0xDA, &MAIN_DMA,
    0xDC, &MAIN_DMA,
    0xDE, &MAIN_DMA,
    0x89, &MAIN_DMA,
    0x8A, &MAIN_DMA,
    0x8B, &MAIN_DMA,
    0x8F, &MAIN_DMA,
};

static QEMM_IODT MAIN_VIRQ_IODT[4] =
{
    0x20, &MAIN_IRQ,
    0x21, &MAIN_IRQ,
    0xA0, &MAIN_IRQ,
    0xA1, &MAIN_IRQ,
};

static QEMM_IODT MAIN_SB_IODT[13] =
{ //MAIN_Options[OPT_ADDR].value will be added at runtime
    0x00, &MAIN_OPL3_388,
    0x01, &MAIN_OPL3_389,
    0x02, &MAIN_OPL3_38A,
    0x03, &MAIN_OPL3_38B,
    0x04, &MAIN_SB_MixerAddr,
    0x05, &MAIN_SB_MixerData,
    0x06, &MAIN_SB_DSP_Reset,
    0x08, &MAIN_OPL3_388,
    0x09, &MAIN_OPL3_389,
    0x0A, &MAIN_SB_DSP_Read,
    0x0C, &MAIN_SB_DSP_Write,
    0x0E, &MAIN_SB_DSP_ReadStatus,
    0x0F, &MAIN_SB_DSP_ReadINT16BitACK,
};

QEMM_IOPT OPL3IOPT;
QEMM_IOPT OPL3IOPT_PM;
QEMM_IOPT MPUIOPT;
QEMM_IOPT MPUIOPT_PM;
QEMM_IOPT MAIN_VDMA_IOPT;
QEMM_IOPT MAIN_VIRQ_IOPT;
QEMM_IOPT MAIN_SB_IOPT;
QEMM_IOPT MAIN_VDMA_IOPT_PM1;
QEMM_IOPT MAIN_VDMA_IOPT_PM2;
QEMM_IOPT MAIN_VDMA_IOPT_PM3;
QEMM_IOPT MAIN_VHDMA_IOPT_PM1;
QEMM_IOPT MAIN_VHDMA_IOPT_PM2;
QEMM_IOPT MAIN_VHDMA_IOPT_PM3;
QEMM_IOPT MAIN_VIRQ_IOPT_PM1;
QEMM_IOPT MAIN_VIRQ_IOPT_PM2;
QEMM_IOPT MAIN_SB_IOPT_PM;

// PC Speaker emulation IO trap handles.
// Note: QEMM accepts the whole 3-port table in a single call, but HDPMI requires
// contiguous ranges, so 0x42-0x43 and 0x61 need separate HDPMI trap handles.
#if SBEMU_PCSPEAKER
static QEMM_IODT MAIN_PCSPEAKER_IODT[3];
static QEMM_IOPT MAIN_PCSPEAKER_IOPT;
static QEMM_IOPT MAIN_PCSPEAKER_IOPT_PM;
static QEMM_IOPT MAIN_PCSPEAKER_IOPT_61;
#endif

#define MAIN_SETCMD_INTRL 0x80000000L //hack: internal. used by TSR communication
#define MAIN_SETCMD_CHGD 0x40000000L //value changed (extra states)
#define OPT_CHANGED(i) (MAIN_Options[i].setcmd&MAIN_SETCMD_CHGD)

#define MAIN_SETCMD_SET 0x01 //set in command line
#define MAIN_SETCMD_HIDDEN 0x02 //hidden flag on report
#define MAIN_SETCMD_BASE10 0x04 //use decimal value (default hex)
#define MAIN_SETCMD_STR 0x08 //string
#define MAIN_SETCMD_STRCS 0x10 //case sensitive
#define MAIN_SETCMD_STRCPY 0x20 //a copy of string, needs free

struct MAIN_OPT
{
    const char* option;
    const char* desc;
    int value;
    int setcmd; //set by command line
}MAIN_Options[] =
{
    "/?", "Show this help screen", FALSE, 0,
    "/DBG", "Debug output (0=console, 1=COM1, 2=COM2, 3=COM3, 4=COM4, otherwise base address)", 0, 0,

    "/A", "IO address (220 or 240) [*]", 0x220, 0,
    "/I", "IRQ number (5 or 7 or 9) [*]", 5, 0,
    "/D", "8-bit DMA channel (0, 1 or 3) [*]", 1, 0,
    "/T", "SB Type (1, 2 or 3=SB; 4 or 5=SBPro; 6=SB16) [*]", 5, 0,
    "/H", "16-bit (\"high\") DMA channel (5, 6 or 7) [*]", 5, 0,

    "/OPL", "Enable OPL3 emulation", TRUE, 0,
    "/PM", "Enable protected mode support (requires HDPMI32I)", TRUE, 0,
    "/RM", "Enable real mode support (requires QEMM or JEMM+QPIEMU)", TRUE, 0,

    "/O", "Select output. 0: headphone, 1: speaker (Intel HDA) or S/PDIF (Xonar DG)", 1, 0,
    "/VOL", "Set master volume (0-" STR(SBEMU_VOLUME_MAX) ", default 100; 85 suits headphones)", 100, MAIN_SETCMD_BASE10,
    "/SFXVOL", "PCM effect level (0-100, default 100; FM unchanged)", 100, MAIN_SETCMD_BASE10,
    "/FILT", "Digital PCM quality (0=linear, 1=+low-pass <=24k, 2=+8-tap sinc interpolation)", 2, MAIN_SETCMD_BASE10,
    "/SWAP", "Swap left/right (0=off, 1=digital PCM only, as SB Pro; 2=whole output)", 0, MAIN_SETCMD_BASE10,
    "/FMQ", "FM synthesis rate (0=auto: half rate below 450 MHz, 1=half rate, 2=full rate)", 0, MAIN_SETCMD_BASE10,
    "/VIRQ", "Sound IRQ for a program in protected mode (1=through the real-mode vector: also real-mode TSR drivers such as Miles under DOS/4G; 0=protected-mode vector only, as upstream)", 1, MAIN_SETCMD_BASE10,
    "/MSHARE", "MIDI synthesis share of the audio period in 16ths (4-14, default 10; lower: shorter interrupts, more notes cut)", 10, MAIN_SETCMD_BASE10,
    "/NOVCPI", "Turn VCPI off in JEMM after loading (1): DOS extenders that pick VCPI before DPMI (DOS/32A, PMODE/W) then run under HDPMI and get sound; 0=VCPI on (default: left as is)", 0, MAIN_SETCMD_BASE10,

    "/K", "Internal sample rate", 22050, MAIN_SETCMD_BASE10,
    "/FIXTC", "Fix time constant to match 11/22/44 kHz sample rate", TRUE, 0,
    "/SCL", "List installed sound cards", 0, MAIN_SETCMD_HIDDEN,
    "/SCFM", "Select FM(OPL) sound card index in list (/SCL)", 0, MAIN_SETCMD_HIDDEN|MAIN_SETCMD_BASE10,
    "/SCMPU", "Select MPU-401 sound card index in list (/SCL)", 0, MAIN_SETCMD_HIDDEN|MAIN_SETCMD_BASE10,
    "/SC", "Select sound card index in list (/SCL)", 0, MAIN_SETCMD_HIDDEN|MAIN_SETCMD_BASE10,
    "/R", "Reset sound card driver", 0, MAIN_SETCMD_HIDDEN,
    "/P", "UART mode MPU-401 IO address [*]", 0x330, 0,
    "/MCOM", "UART mode MPU-401 COM port (1=COM1, 2=COM2, 3=COM3, 4=COM4, 9:HW MPU only, otherwise base address)", 9, 0,
    "/COML", "List installed COM ports", 0, MAIN_SETCMD_HIDDEN,
#if MPU_DEBUG
    "/MDBG", "Enable MPU-401 debugging (0 to disable, 1 or 2 to enable)", 0, 0,
#endif
#if SBEMU_VMPU
    "/VMPU","Virtual MPU max voices ([32-256]. 1=64, 0=disabled)", 0, MAIN_SETCMD_BASE10 | (MAIN_VMPU_HDPMI_MEMFIX ? 0 : MAIN_SETCMD_HIDDEN), //voice num also act as a switch. 0:turn off VMPU
    "/VMSF", "VMPU Soundfont2 file", (uintptr_t)VMPU_DEF_SF2, MAIN_SETCMD_STR | (MAIN_VMPU_HDPMI_MEMFIX ? 0 : MAIN_SETCMD_HIDDEN), //NOTE: string ptr to int conversion always safe for 32bit code
    "/VMSFBF", "", 0, MAIN_SETCMD_INTRL|MAIN_SETCMD_HIDDEN,
#endif
#ifdef __DJ2__
    "/XMSLOW", "Install legacy XMS DMA limit until reboot (0=off, 1=on)", 0, MAIN_SETCMD_BASE10,
#endif
    NULL, NULL, 0,
};
enum EOption
{
    OPT_Help,
    OPT_DEBUG_OUTPUT,

    OPT_ADDR,
    OPT_IRQ,
    OPT_DMA,
    OPT_TYPE,
    OPT_HDMA,
    OPT_OPL,
    OPT_PM,
    OPT_RM,
    OPT_OUTPUT,
    OPT_VOL,
    OPT_SFXVOL,
    OPT_FILT,
    OPT_SWAP,
    OPT_OPLQ,
    OPT_VIRQ,
    OPT_MSHARE,
    OPT_NOVCPI,
    OPT_RATE,
    OPT_FIX_TC,
    OPT_SCLIST,
    OPT_SCFM,
    OPT_SCMPU,
    OPT_SC,
    OPT_RESET,
    OPT_MPUADDR,
    OPT_MPUCOMPORT,
    OPT_COMPORTLIST,
#if MPU_DEBUG
    OPT_MDBG,
#endif
#if SBEMU_VMPU
    OPT_VMPU_VOICES,
    OPT_VMPU_SF,
    OPT_VMPU_SFBUF,
#endif
#ifdef __DJ2__
    OPT_XMSLOW,
#endif
    OPT_COUNT,
};

//T1~T6 maps
static const char* MAIN_SBTypeString[] =
{
    "0",
    "1.0/1.5",
    "Pro (old)",
    "2.0",
    "Pro",
    "Pro",
    "16",
};
static int MAIN_SB_DSPVersion[] =
{
    0,
    0x0100,
    0x0300,
    0x0202,
    0x0302,
    0x0302,
    0x0405, //Geode: as DOSBox; real SB16 cards report 4.04-4.13, never 4.00
};

/* Geode: FM synthesis at the card rate (48 kHz) is the largest part of the audio
   interrupt. Below about 450 MHz it is rendered at half rate and interpolated
   (/FMQ=0 decides from the measured CPU clock, 1 and 2 force it). */
static int MAIN_CPUMHz;
#ifndef MAIN_BIOS_TICKS
#define MAIN_BIOS_TICKS() _farpeekl(_dos_ds, 0x46c)
#endif
static int MAIN_CalibrateCPU(void)
{
    /* A delay function promises a minimum delay, not an exact time interval.
     * Measure four BIOS/PIT ticks instead. Reject a stalled/jumped clock;
     * retry across midnight rather than treating the BDA reset as a wrap. */
    for(unsigned attempt = 0; attempt < 3; ++attempt)
    {
        uint32_t first = MAIN_BIOS_TICKS(), start, end;
        unsigned guard = 100000000;
        while(MAIN_BIOS_TICKS() == first && --guard) {}
        if(!guard) return 0;
        start = MAIN_BIOS_TICKS();
        const uint32_t t0 = MAIN_TSC();
        guard = 100000000;
        do { end = MAIN_BIOS_TICKS(); }
        while(end >= start && end - start < 4 && --guard);
        const uint32_t cycles = MAIN_TSC() - t0;
        if(!guard) return 0;
        if(end < start || end - start != 4) continue;
        const uint64_t divisor = 4ULL * 65536 * 1000000;
        return (int)(((uint64_t)cycles * 1193182 + divisor / 2) / divisor);
    }
    return 0;
}
static int MAIN_OPLHalfRate(void)
{
    if(!MAIN_CPUMHz)
        MAIN_CPUMHz = MAIN_CalibrateCPU();
    if(MAIN_CPUMHz <= 0)
    {
        fprintf(stderr, "Cannot calibrate CPU: BIOS clock unavailable.\n");
        exit(1);
    }
    if(MAIN_Options[OPT_OPLQ].value == 1) return 1;
    if(MAIN_Options[OPT_OPLQ].value == 2) return 0;
    return MAIN_CPUMHz < 450;
}

static void MAIN_InitOPL(int samplerate)
{
    /* Calibration must finish before MAIN_CPUMHz is evaluated as an argument. */
    const int half = MAIN_OPLHalfRate();
    OPL3EMU_Init(samplerate, half, MAIN_CPUMHz);
}

static void MAIN_InvokeIRQ(uint8_t irq) //generate virtual IRQ
{
    MAIN_VirtualIRQCount++;
    #if MAIN_TRAP_RMPIC_ONDEMAND
    if(MAIN_Options[OPT_RM].value) QEMM_Install_IOPortTrap(MAIN_VIRQ_IODT, countof(MAIN_VIRQ_IODT), &MAIN_VIRQ_IOPT);
    #endif
    #if MAIN_TRAP_PMPIC_ONDEMAND
    if(MAIN_Options[OPT_PM].value)
    {
        HDPMIPT_Install_IOPortTrap(0x20, 0x21, MAIN_VIRQ_IODT, 2, &MAIN_VIRQ_IOPT_PM1);
        HDPMIPT_Install_IOPortTrap(0xA0, 0xA1, MAIN_VIRQ_IODT+2, 2, &MAIN_VIRQ_IOPT_PM2);
    }
    #endif

    HDPMIPT_DisableIRQRouting(irq); //disable routing
    IRQGUARD_Enable();
    VIRQ_Invoke(irq, &MAIN_IntContext.regs, MAIN_IntContext.EFLAGS&CPU_VMFLAG);
    IRQGUARD_Disable();
    HDPMIPT_EnableIRQRouting(irq); //restore routing

    #if MAIN_TRAP_RMPIC_ONDEMAND
    if(MAIN_Options[OPT_RM].value) QEMM_Uninstall_IOPortTrap(&MAIN_VIRQ_IOPT);
    #endif
    #if MAIN_TRAP_PMPIC_ONDEMAND
    if(MAIN_Options[OPT_PM].value)
    {
        HDPMIPT_Uninstall_IOPortTrap(&MAIN_VIRQ_IOPT_PM1);
        HDPMIPT_Uninstall_IOPortTrap(&MAIN_VIRQ_IOPT_PM2);
    }
    #endif
}

static void MAIN_SetBlasterEnv(struct MAIN_OPT* opt) //alter BLASTER env.
{
    char buf[256];
    if(opt[OPT_TYPE].value != 6) //T5 not working for some games, should use T4 for SBPro (t=4/5)
        sprintf(buf, "A%x I%x D%x P%x T%x", opt[OPT_ADDR].value, opt[OPT_IRQ].value, opt[OPT_DMA].value, opt[OPT_MPUADDR].value, opt[OPT_TYPE].value == 5 ? 4 : opt[OPT_TYPE].value);
    else
        sprintf(buf, "A%x I%x D%x T%x H%x P%x", opt[OPT_ADDR].value, opt[OPT_IRQ].value, opt[OPT_DMA].value, opt[OPT_TYPE].value, opt[OPT_HDMA].value, opt[OPT_MPUADDR].value);
    #ifdef DJGPP //makes vscode happy
    setenv("BLASTER", buf, TRUE);
    #endif
}

static void MAIN_CPrintf(int color, const char* fmt, ...)
{
    char buf[2048]; //limit for cprintf (DJGPP): 2048
    va_list aptr;
    va_start(aptr, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, aptr);
    va_end(aptr);
    int crlf = strcmp(buf+n-2,"\r\n") == 0 || strcmp(buf+n-2,"\n\r") == 0;
    int lf = !crlf && buf[n-1] == '\n';
    buf[n-2] = crlf ? '\0' : buf[n-2];
    buf[n-1] = lf ? '\0' : buf[n-1];
    textcolor(color);
    cputs(buf); //more safe with a fmt string, incase buf contains any fmt.
    textcolor(LIGHTGRAY);
    if(crlf || lf) cputs("\r\n");
}

//Geode: ISA DMA reaches only the first 16 MB (24-bit address: page register + 16-bit address). Drivers such as
//Miles AIL play straight from a locked XMS block; when the memory this TSR and HDPMI32i take during the install
//lies below 16 MB, those blocks land above it and the DMA page register gets only bits 16-23.
//The resulting address no longer points to the original samples. Hold the free XMS below 16 MB in one block while installing, so
//our own allocations go above it, and give it back before going resident.
static uint32_t MAIN_XMSEntry;
static uint16_t MAIN_XMSHoldHandle;
static uint32_t MAIN_XMSHoldKB;
static BOOL MAIN_XMSCall(DPMI_REG* r)
{
    r->w.cs = (uint16_t)(MAIN_XMSEntry>>16); r->w.ip = (uint16_t)MAIN_XMSEntry; r->w.ss = r->w.sp = 0;
    return DPMI_CallRealModeRETF(r) == 0 && r->w.ax == 1;
}
static void MAIN_XMSHold(void)
{
    DPMI_REG r = {0};
    r.w.ax = 0x4300;
    DPMI_CallRealModeINT(0x2F, &r);
    if(r.h.al != 0x80)
        return;
    memset(&r, 0, sizeof(r)); r.w.ax = 0x4310;
    DPMI_CallRealModeINT(0x2F, &r);
    MAIN_XMSEntry = ((uint32_t)r.w.es<<16) | r.w.bx;
    memset(&r, 0, sizeof(r)); r.h.ah = 0x09; r.w.dx = 1; //probe: where does the lowest free XMS start?
    if(!MAIN_XMSCall(&r))
        return;
    uint16_t probe = r.w.dx;
    memset(&r, 0, sizeof(r)); r.h.ah = 0x0C; r.w.dx = probe;
    uint32_t addr = MAIN_XMSCall(&r) ? ((uint32_t)r.w.dx<<16) | r.w.bx : 0xFFFFFFFFUL;
    memset(&r, 0, sizeof(r)); r.h.ah = 0x0D; r.w.dx = probe; MAIN_XMSCall(&r);
    memset(&r, 0, sizeof(r)); r.h.ah = 0x0A; r.w.dx = probe; MAIN_XMSCall(&r);
    if(addr >= 16UL*1024*1024)
        return;
    uint32_t kb = (16UL*1024*1024 - addr) / 1024;
    memset(&r, 0, sizeof(r)); r.h.ah = 0x88; //XMS 3.0: EDX = total free KB
    if(!MAIN_XMSCall(&r) && r.h.bl != 0)
        return;
    if(r.d.edx < kb + 32*1024) //small machine: the TSR needs that memory itself
        return;
    memset(&r, 0, sizeof(r)); r.h.ah = 0x89; r.d.edx = kb; //XMS 3.0: size in EDX
    if(!MAIN_XMSCall(&r))
    {
        memset(&r, 0, sizeof(r)); r.h.ah = 0x09; r.w.dx = kb > 0xFFFF ? 0xFFFF : (uint16_t)kb;
        if(!MAIN_XMSCall(&r))
            return;
    }
    MAIN_XMSHoldHandle = r.w.dx;
    MAIN_XMSHoldKB = kb;
}
static void MAIN_XMSRelease(void)
{
    if(!MAIN_XMSHoldHandle)
        return;
    DPMI_REG r = {0};
    r.h.ah = 0x0A; r.w.dx = MAIN_XMSHoldHandle;
    MAIN_XMSCall(&r);
    MAIN_XMSHoldHandle = 0;
}

#ifdef __DJ2__
extern const unsigned char MAIN_XMSLowCode[], MAIN_XMSLowCodeEnd[];
static uint16_t MAIN_XMSLowSegment;
enum { XMSLOW_PREVIOUS = 4, XMSLOW_MANAGER = 8, XMSLOW_PASSIVE = 12, XMSLOW_ISSUED = 13 };

static void MAIN_XMSLowRollback(void)
{
    uint16_t segment = MAIN_XMSLowSegment;
    if(!segment)
        return;
    uint32_t linear = DPMI_SEGOFF2L(segment, 0);
    uint32_t ivt = 0x2f * 4;
    CLIS();
    DPMI_StoreB(linear + XMSLOW_PASSIVE, 1);
    BOOL head = DPMI_LoadD(ivt) == (uint32_t)segment << 16;
    if(head)
        DPMI_StoreD(ivt, DPMI_LoadD(linear + XMSLOW_PREVIOUS));
    BOOL keep = !head || DPMI_LoadB(linear + XMSLOW_ISSUED);
    if(keep)
    {
        // A later interrupt hook or cached far pointer can still enter this
        // standalone DOS block. Give its MCB self-ownership across PSP exit.
        DPMI_StoreW(linear - 16 + 1, segment);
    }
    STIL();
    if(!keep)
    {
        DPMI_REG r = {0};
        r.w.ax = 0x4900;
        r.w.es = segment;
        DPMI_CallRealModeINT(0x21, &r);
    }
    MAIN_XMSLowSegment = 0;
}

static BOOL MAIN_XMSLowInstall(void)
{
    DPMI_REG r = {0};
    r.w.ax = 0x4300;
    if(DPMI_CallRealModeINT(0x2f, &r) || r.h.al != 0x80)
        return FALSE;
    r.w.ax = 0x4310;
    if(DPMI_CallRealModeINT(0x2f, &r) || (!r.w.es && !r.w.bx))
        return FALSE;
    uint32_t manager = ((uint32_t)r.w.es << 16) | r.w.bx;
    if(atexit(MAIN_XMSLowRollback))
        return FALSE;
    // A real DOS allocation lets rollback retain a passive stub independently
    // of DPMI client teardown if another TSR has acquired a reference.
    unsigned bytes = (uintptr_t)MAIN_XMSLowCodeEnd - (uintptr_t)MAIN_XMSLowCode;
    r.w.ax = 0x4800;
    r.w.bx = (bytes + 15) >> 4;
    if(DPMI_CallRealModeINT(0x21, &r) || (r.w.flags & CPU_CFLAG))
        return FALSE;
    uint16_t segment = r.w.ax;
    uint32_t linear = DPMI_SEGOFF2L(segment, 0);
    DPMI_LMemcpy(linear, DPMI_PTR2L((void*)MAIN_XMSLowCode), bytes);
    DPMI_StoreD(linear + XMSLOW_MANAGER, manager);
    CLIS();
    DPMI_StoreD(linear + XMSLOW_PREVIOUS, DPMI_LoadD(0x2f * 4));
    DPMI_StoreD(0x2f * 4, (uint32_t)segment << 16);
    MAIN_XMSLowSegment = segment;
    STIL();
    return TRUE;
}
#endif

static void
MAIN_Print_Enabled_Newline(bool enabled)
{
    MAIN_CPrintf(enabled ? LIGHTGREEN : LIGHTRED, "%s", enabled ? "enabled" : "disabled");
    cputs(".\r\n");
}

//Geode: VCPI on/off in a running Jemm386/JemmEx, as "JEMMEX NOVCPI" at the prompt does (Jemm src/INIT16.ASM EmmUpdate,
//src/DEV.ASM IoctlWrite): IOCTL write 15 to EMMXXXX0 (EMMQXXX0 with NOEMS), 0FFh = leave unchanged. Only the VCPI API
//(DE00h-DE0Bh) is refused; the DE0Ch mode switch of an already loaded HDPMI keeps working.
//DOS/32A always tries VCPI first and PMODE/W does by default: under VCPI their port I/O cannot be trapped.
//Returns the previous state (0=VCPI on, 1=off) or -1 if no Jemm. Plain DOS calls: not for the TSR interrupt.
static int
MAIN_JemmSetNoVCPI(int novcpi)
{
    static const char* devices[2] = {"EMMXXXX0", "EMMQXXX0"};
    uint32_t dosmem = DPMI_DOSMalloc(4); //64 bytes: name at 0, IOCTL buffer at 16
    if(!dosmem)
        return -1;
    uint16_t seg = (uint16_t)dosmem;
    uint32_t lin = (uint32_t)seg << 4;
    int prev = -1;
    for(int n = 0; n < 2 && prev < 0; ++n)
    {
        DPMI_REG r = {0};
        DPMI_LMemcpy(lin, DPMI_PTR2L((void*)devices[n]), 9);
        r.w.ax = 0x3D00; //open
        r.w.ds = seg;
        r.w.dx = 0;
        if(DPMI_CallRealModeINT(0x21, &r) != 0 || (r.w.flags&CPU_CFLAG))
            continue;
        uint16_t handle = r.w.ax;
        uint8_t buf[24];
        memset(buf, 0, sizeof(buf)); //function 0: version, 0028h = Jemm
        DPMI_LMemcpy(lin+16, DPMI_PTR2L(buf), sizeof(buf));
        memset(&r, 0, sizeof(r));
        r.w.ax = 0x4402; r.w.bx = handle; r.w.cx = 6; r.w.ds = seg; r.w.dx = 16;
        BOOL jemm = DPMI_CallRealModeINT(0x21, &r) == 0 && !(r.w.flags&CPU_CFLAG);
        DPMI_LMemcpy(DPMI_PTR2L(buf), lin+16, 6);
        jemm = jemm && buf[0] == 0x28 && buf[1] == 0;
        if(jemm)
        {
            memset(buf, 0, sizeof(buf));
            buf[0] = 6; //EMMDEV_SYSVARS: e06_NoVCPI at +3
            DPMI_LMemcpy(lin+16, DPMI_PTR2L(buf), sizeof(buf));
            memset(&r, 0, sizeof(r));
            r.w.ax = 0x4402; r.w.bx = handle; r.w.cx = sizeof(buf); r.w.ds = seg; r.w.dx = 16;
            int oldstate = 0;
            if(DPMI_CallRealModeINT(0x21, &r) == 0 && !(r.w.flags&CPU_CFLAG))
            {
                DPMI_LMemcpy(DPMI_PTR2L(buf), lin+16, sizeof(buf));
                oldstate = buf[3] ? 1 : 0;
            }
            const uint8_t update[5] = {15, 0xFF, 0xFF, (uint8_t)(novcpi ? 1 : 0), 0xFF}; //EMMDEV_UPDATE: VME, A20, VCPI, PGE
            DPMI_LMemcpy(lin+16, DPMI_PTR2L((void*)update), sizeof(update));
            memset(&r, 0, sizeof(r));
            r.w.ax = 0x4403; r.w.bx = handle; r.w.cx = sizeof(update); r.w.ds = seg; r.w.dx = 16;
            if(DPMI_CallRealModeINT(0x21, &r) == 0 && !(r.w.flags&CPU_CFLAG))
                prev = oldstate;
        }
        memset(&r, 0, sizeof(r));
        r.w.ax = 0x3E00; r.w.bx = handle; //close
        DPMI_CallRealModeINT(0x21, &r);
    }
    DPMI_DOSFree(dosmem);
    return prev;
}

static int
update_serial_debug_output()
{
    bool enabled = (MAIN_Options[OPT_DEBUG_OUTPUT].value != 0);
    if (!enabled) {
        _LOG("Serial port debugging disabled.\n");
    }
    #if MPU_DEBUG
    int err = ser_setup(MAIN_Options[OPT_MDBG].value ? SBEMU_SERIAL_TYPE_FASTDBG : SBEMU_SERIAL_TYPE_DBG, MAIN_Options[OPT_DEBUG_OUTPUT].value);
    #else
    int err = ser_setup(SBEMU_SERIAL_TYPE_DBG, MAIN_Options[OPT_DEBUG_OUTPUT].value);
    #endif
    if (enabled) {
        _LOG("Serial port debugging enabled.\n");
    }
    return err;
}

static int
update_serial_mpu_output()
{
    bool enabled = (MAIN_Options[OPT_MPUCOMPORT].value != 0);
    if (!enabled) {
        _LOG("MPU-401 serial output disabled.\n");
    }
    int err = ser_setup(SBEMU_SERIAL_TYPE_MIDI, MAIN_Options[OPT_MPUCOMPORT].value);
    if (enabled) {
        _LOG("MPU-401 serial output enabled.\n");
    }
    return err;
}

static BOOL OPLRMInstalled, OPLPMInstalled;
static BOOL MAIN_TSRISRInstalled, MAIN_PMISRInstalled, MAIN_RMISRInstalled;
static BOOL MAIN_IRQRoutingLocked;
static BOOL MAIN_AudioStarted;
static uint16_t MAIN_PICOriginalMask, MAIN_PICChangedBits;
static BOOL MAIN_PCIInterruptWasDisabled;
#define MAIN_PCI_INTX_DISABLE (1u << 10)
static int MAIN_PreviousNoVCPI = -1;

static struct
{
    uint8_t irq;
    HDPMIPT_IRQRoutedHandle saved;
} MAIN_Routes[] = {{5}, {7}, {9}, {0}};

static BOOL MAIN_SetIRQRoute(int index, uint16_t cs, uint32_t offset, uint16_t rmcs, uint16_t rmoffset)
{
    HDPMIPT_IRQRoutedHandle* saved = &MAIN_Routes[index].saved;
    const uint8_t irq = MAIN_Routes[index].irq;
    if(!HDPMIPT_GetIRQRoutedHandler(irq, &saved->cs, &saved->offset, &saved->rmcs, &saved->rmoffset))
        return FALSE;
    if(!HDPMIPT_InstallIRQRoutedHandler(irq, cs, offset, rmcs, rmoffset))
        return FALSE;
    saved->valid = TRUE;
    return TRUE;
}

static BOOL MAIN_InstallIRQRoutes(void)
{
    const int physical = countof(MAIN_Routes) - 1;
    for(int i = 0; i < physical; ++i)
    {
        uint8_t irq = MAIN_Routes[i].irq;
        if(irq == aui.card_irq)
            continue; // The physical route below owns this line.
        DPMI_ISR_HANDLE handle;
        // DPMI_GetISR takes an interrupt vector, including for IRQ 5, 7 and 9.
        DPMI_GetISR(PIC_IRQ2VEC(irq), &handle);
        if(!MAIN_SetIRQRoute(i, handle.old_cs, handle.old_offset, handle.old_rm_cs, handle.old_rm_offset))
            return FALSE;
    }

    MAIN_Routes[physical].irq = aui.card_irq;
#if MAIN_INSTALL_RM_ISR
    const uint16_t rmcs = MAIN_IntHandleRM.wrapper_cs;
    const uint16_t rmoffset = MAIN_IntHandleRM.wrapper_offset;
#else
    const uint16_t rmcs = MAIN_IntHandlePM.old_rm_cs;
    const uint16_t rmoffset = MAIN_IntHandlePM.old_rm_offset;
#endif
    if(!MAIN_SetIRQRoute(physical, MAIN_IntHandlePM.wrapper_cs, MAIN_IntHandlePM.wrapper_offset, rmcs, rmoffset))
        return FALSE;
    MAIN_IRQRoutingLocked = HDPMIPT_LockIRQRouting(TRUE);
    return MAIN_IRQRoutingLocked;
}

static void MAIN_Cleanup(void)
{
    // The source and PIC line must be quiet before removing their routes and vectors.
    if(MAIN_AudioStarted)
    {
        PIC_MaskIRQ(aui.card_irq);
        AU_stop(&aui);
        MAIN_AudioStarted = FALSE;
    }
    if(MAIN_PCIInterruptWasDisabled)
    {
        uint16_t command = pcibios_ReadConfig_Word(aui.card_pci_dev, PCIR_PCICMD);
        pcibios_WriteConfig_Word(aui.card_pci_dev, PCIR_PCICMD, command | MAIN_PCI_INTX_DISABLE);
        MAIN_PCIInterruptWasDisabled = FALSE;
    }
    if(MAIN_IRQRoutingLocked)
    {
        HDPMIPT_LockIRQRouting(FALSE);
        MAIN_IRQRoutingLocked = FALSE;
    }
    for(int i = countof(MAIN_Routes) - 1; i >= 0; --i)
    {
        HDPMIPT_IRQRoutedHandle* saved = &MAIN_Routes[i].saved;
        if(saved->valid)
        {
            HDPMIPT_InstallIRQRoutedHandlerH(MAIN_Routes[i].irq, saved);
            saved->valid = FALSE;
        }
    }

    IRQGUARD_Uninstall();
    if(MAIN_RMISRInstalled)
    {
        DPMI_UninstallISR(&MAIN_IntHandleRM);
        MAIN_RMISRInstalled = FALSE;
    }
    if(MAIN_PMISRInstalled)
    {
        DPMI_UninstallISR(&MAIN_IntHandlePM);
        MAIN_PMISRInstalled = FALSE;
    }
    if(MAIN_TSRISRInstalled)
    {
        DPMI_UninstallISR(&MAIN_TSRIntHandle);
        MAIN_TSRISRInstalled = FALSE;
    }
    if(MAIN_ISR_DOSID)
    {
        DPMI_HighFree(MAIN_ISR_DOSID);
        MAIN_ISR_DOSID = 0;
    }

    QEMM_IOPT* const pmTraps[] = {
#if SBEMU_PCSPEAKER
        &MAIN_PCSPEAKER_IOPT_61, &MAIN_PCSPEAKER_IOPT_PM,
#endif
        &MAIN_SB_IOPT_PM, &MAIN_VIRQ_IOPT_PM2, &MAIN_VIRQ_IOPT_PM1,
        &MAIN_VHDMA_IOPT_PM3, &MAIN_VHDMA_IOPT_PM2, &MAIN_VHDMA_IOPT_PM1,
        &MAIN_VDMA_IOPT_PM3, &MAIN_VDMA_IOPT_PM2, &MAIN_VDMA_IOPT_PM1,
        &MPUIOPT_PM, &OPL3IOPT_PM,
    };
    QEMM_IOPT* const rmTraps[] = {
#if SBEMU_PCSPEAKER
        &MAIN_PCSPEAKER_IOPT,
#endif
        &MAIN_SB_IOPT, &MAIN_VIRQ_IOPT, &MAIN_VDMA_IOPT, &MPUIOPT, &OPL3IOPT,
    };
    for(int i = 0; i < countof(pmTraps); ++i)
        if(pmTraps[i]->memory)
            HDPMIPT_Uninstall_IOPortTrap(pmTraps[i]);
    for(int i = 0; i < countof(rmTraps); ++i)
        if(rmTraps[i]->memory)
            QEMM_Uninstall_IOPortTrap(rmTraps[i]);
    QEMM_ReleaseIOPortTrap();

    if(MAIN_PreviousNoVCPI == 0)
    {
        MAIN_JemmSetNoVCPI(0);
        MAIN_PreviousNoVCPI = -1;
    }
    if(MAIN_DMA_MappedAddr > 1024*1024)
    {
        DPMI_UnmappMemory(MAIN_DMA_MappedAddr);
        MAIN_DMA_MappedAddr = 0;
    }
    AU_close(&aui, &fm_aui, &mpu401_aui);
    if(MAIN_PICChangedBits)
    {
        uint16_t mask = PIC_GetIRQMask();
        PIC_SetIRQMask((mask & ~MAIN_PICChangedBits) | (MAIN_PICOriginalMask & MAIN_PICChangedBits));
        MAIN_PICChangedBits = 0;
    }
}

static int MAIN_Install(BOOL enableRM, BOOL enablePM, QEMM_IODT* SB_Iodt, int SB_IodtCount)
{
    BOOL QEMMInstalledVDMA = !enableRM || QEMM_Install_IOPortTrap(MAIN_VDMA_IODT, countof(MAIN_VDMA_IODT), &MAIN_VDMA_IOPT);
    #if MAIN_TRAP_RMPIC_ONDEMAND//will crash with VIRQ installed, do it temporarily. TODO: figure out why
    BOOL QEMMInstalledVIRQ = TRUE;
    #else
    BOOL QEMMInstalledVIRQ = !enableRM || QEMM_Install_IOPortTrap(MAIN_VIRQ_IODT, countof(MAIN_VIRQ_IODT), &MAIN_VIRQ_IOPT);
    #endif
    BOOL QEMMInstalledSB = !enableRM || QEMM_Install_IOPortTrap(SB_Iodt, SB_IodtCount, &MAIN_SB_IOPT);

    BOOL HDPMIInstalledVDMA1 = !enablePM || HDPMIPT_Install_IOPortTrap(0x0, 0xF, MAIN_VDMA_IODT, 16, &MAIN_VDMA_IOPT_PM1);
    BOOL HDPMIInstalledVDMA2 = !enablePM || HDPMIPT_Install_IOPortTrap(0x81, 0x83, MAIN_VDMA_IODT+16, 3, &MAIN_VDMA_IOPT_PM2);
    BOOL HDPMIInstalledVDMA3 = !enablePM || HDPMIPT_Install_IOPortTrap(0x87, 0x87, MAIN_VDMA_IODT+19, 1, &MAIN_VDMA_IOPT_PM3);
    BOOL HDPMIInstalledVHDMA1 = !enablePM || HDPMIPT_Install_IOPortTrap(0xC0, 0xDE, MAIN_VDMA_IODT+20, 16, &MAIN_VHDMA_IOPT_PM1);
    BOOL HDPMIInstalledVHDMA2 = !enablePM || HDPMIPT_Install_IOPortTrap(0x89, 0x8B, MAIN_VDMA_IODT+36, 3, &MAIN_VHDMA_IOPT_PM2);
    BOOL HDPMIInstalledVHDMA3 = !enablePM || HDPMIPT_Install_IOPortTrap(0x8F, 0x8F, MAIN_VDMA_IODT+39, 1, &MAIN_VHDMA_IOPT_PM3);
    #if MAIN_TRAP_PMPIC_ONDEMAND
    BOOL HDPMIInstalledVIRQ1 = TRUE;
    BOOL HDPMIInstalledVIRQ2 = TRUE;
    #else
    BOOL HDPMIInstalledVIRQ1 = !enablePM || HDPMIPT_Install_IOPortTrap(0x20, 0x21, MAIN_VIRQ_IODT, 2, &MAIN_VIRQ_IOPT_PM1);
    BOOL HDPMIInstalledVIRQ2 = !enablePM || HDPMIPT_Install_IOPortTrap(0xA0, 0xA1, MAIN_VIRQ_IODT+2, 2, &MAIN_VIRQ_IOPT_PM2);
    #endif
    BOOL HDPMIInstalledSB = !enablePM || HDPMIPT_Install_IOPortTrap(MAIN_Options[OPT_ADDR].value, MAIN_Options[OPT_ADDR].value+0x0F, SB_Iodt, SB_IodtCount, &MAIN_SB_IOPT_PM);

#if SBEMU_PCSPEAKER
    //* --- PC Speaker init (andersrodrig) ---
    // Populate PC Speaker IO trap table.
    // 0x42/0x43 = PIT Channel 2 data/command, 0x61 = System Control Port B (speaker gate).
    {
        MAIN_PCSPEAKER_IODT[0].port = 0x42; // PIT Channel 2 Data
        MAIN_PCSPEAKER_IODT[1].port = 0x43; // PIT Command
        MAIN_PCSPEAKER_IODT[2].port = 0x61; // System Control Port B
        for(int i = 0; i < 3; i++) {
            MAIN_PCSPEAKER_IODT[i].handler = (void*)VPCSPEAKER_IOHandler;
        }
    }
#endif // SBEMU_PCSPEAKER

    if(!QEMMInstalledVDMA || !QEMMInstalledVIRQ || !QEMMInstalledSB)
    {
        MAIN_CPrintf(RED, "Error: Failed installing IO port trap for QEMM.\n");
        return 1;
    }
    if(!HDPMIInstalledVDMA1 || !HDPMIInstalledVDMA2 || !HDPMIInstalledVDMA3
    || !HDPMIInstalledVHDMA1 || !HDPMIInstalledVHDMA2 || !HDPMIInstalledVHDMA3
    || !HDPMIInstalledVIRQ1 || !HDPMIInstalledVIRQ2 || !HDPMIInstalledSB)
    {
        MAIN_CPrintf(RED, "Error: Failed installing IO port trap for HDPMI.\n");
        return 1;
    }

    for(int i = MAIN_TSR_INTSTART_ID; i <= 0xFF; ++i)
    {
        DPMI_REG r = {0};
        r.h.ah = i;
        DPMI_CallRealModeINT(MAIN_TSR_INT, &r);
        if(r.h.al != 0)
            continue;
        MAIN_TSR_INT_FNO = i;
        MAIN_TSRISRInstalled = DPMI_InstallRealModeISR(MAIN_TSR_INT, MAIN_TSR_Interrupt, &MAIN_TSRREG, &MAIN_TSRIntHandle, FALSE) == 0;
        break;
    }
    if(!MAIN_TSRISRInstalled)
    {
        MAIN_CPrintf(RED, "Error: Failed installing TSR interrupt.\n");
        return 1;
    }
    MAIN_ISR_DOSID = DPMI_HighMalloc((sizeof(MAIN_ISR_DOSID_String)+15)>>4, TRUE);
    if(!MAIN_ISR_DOSID)
    {
        MAIN_CPrintf(RED, "Error: Could not allocate DOS memory for TSR ID.\n");
        return 1;
    }
    DPMI_LMemcpy(DPMI_SEGOFF2L(MAIN_ISR_DOSID,0), DPMI_PTR2L((char*)MAIN_ISR_DOSID_String), sizeof(MAIN_ISR_DOSID_String));

    _LOG("sound card IRQ: %d\n", aui.card_irq);
    MAIN_PICOriginalMask = PIC_GetIRQMask();
    MAIN_PICChangedBits = 1u << aui.card_irq;
    PIC_MaskIRQ(aui.card_irq);
    AU_ini_interrupts(&aui);
    VIRQ_SetRealModeVector(MAIN_Options[OPT_VIRQ].value);
    int samplerate = MAIN_Options[OPT_RATE].value;
    mpxplay_audio_decoder_info_s adi = {NULL, 0, 1, samplerate, SBEMU_CHANNELS, SBEMU_CHANNELS, NULL, SBEMU_BITS, SBEMU_BITS/8, 0};
    AU_setrate(&aui, &adi);
#if SBEMU_VMPU
    VMPU_SetOutputRate(aui.freq_card);
    VMPU_SetIsrShare(MAIN_Options[OPT_MSHARE].value);
#endif
    AU_setmixer_init(&aui);
    AU_setmixer_outs(&aui, MIXER_SETMODE_ABSOLUTE, 100);
    //set volume
    AU_setmixer_one(&aui, AU_MIXCHAN_MASTER, MIXER_SETMODE_ABSOLUTE, MAIN_Options[OPT_VOL].value);
    MAIN_LPF_Build(aui.freq_card);
    MAIN_Resample_Build();
    if(MAIN_Options[OPT_OPL].value)
        MAIN_InitOPL(aui.freq_card); //aui.freq_card available after AU_setrate

    // Physical interrupts can arrive in either CPU mode, independently of the port traps.
    MAIN_PMISRInstalled = DPMI_InstallISR(PIC_IRQ2VEC(aui.card_irq), MAIN_InterruptPM, &MAIN_IntHandlePM, MAIN_ISR_CHAINED) == 0;
    if(!MAIN_PMISRInstalled)
    {
        MAIN_CPrintf(RED, "Error: Failed installing protected-mode sound card ISR.\n");
        return 1;
    }
#if MAIN_INSTALL_RM_ISR
    MAIN_RMISRInstalled = DPMI_InstallRealModeISR(PIC_IRQ2VEC(aui.card_irq), MAIN_InterruptRM, &MAIN_RMIntREG, &MAIN_IntHandleRM, MAIN_ISR_CHAINED) == 0;
    if(!MAIN_RMISRInstalled)
    {
        MAIN_CPrintf(RED, "Error: Failed installing real-mode sound card ISR.\n");
        return 1;
    }
#endif
    if(!IRQGUARD_Install(MAIN_Options[OPT_IRQ].value))
    {
        MAIN_CPrintf(RED, "Error: Failed installing virtual IRQ guard.\n");
        return 1;
    }

    // An RM-only configuration can also run under the modified HDPMI host.
    if((enablePM || HDPMIPT_Detect()) && !MAIN_InstallIRQRoutes())
    {
        MAIN_CPrintf(RED, "Error: Failed installing HDPMI IRQ routes.\n");
        return 1;
    }

#if SBEMU_PCSPEAKER
    VPCSPEAKER_Init(aui.freq_card);
    BOOL speakerRM = enableRM && QEMM_Install_IOPortTrap(MAIN_PCSPEAKER_IODT, 3, &MAIN_PCSPEAKER_IOPT);
    BOOL speakerPM = enablePM
        && HDPMIPT_Install_IOPortTrap(0x42, 0x43, MAIN_PCSPEAKER_IODT, 2, &MAIN_PCSPEAKER_IOPT_PM)
        && HDPMIPT_Install_IOPortTrap(0x61, 0x61, MAIN_PCSPEAKER_IODT + 2, 1, &MAIN_PCSPEAKER_IOPT_61);
    if(!speakerPM && MAIN_PCSPEAKER_IOPT_PM.memory)
        HDPMIPT_Uninstall_IOPortTrap(&MAIN_PCSPEAKER_IOPT_PM);
    printf("PC Speaker emulation (Ports 42,43,61): ");
    MAIN_Print_Enabled_Newline(speakerRM || speakerPM);
#endif

    MAIN_PCIInterruptWasDisabled =
        (pcibios_ReadConfig_Word(aui.card_pci_dev, PCIR_PCICMD) & MAIN_PCI_INTX_DISABLE) != 0;
    pcibios_enable_interrupt(aui.card_pci_dev);
    if(aui.card_irq >= 8)
        MAIN_PICChangedBits |= 1u << 2; // PIC_UnmaskIRQ also opens the slave cascade.
    PIC_UnmaskIRQ(aui.card_irq);
    MAIN_AudioStarted = TRUE;
    AU_prestart(&aui);
    AU_start(&aui);

    if(MAIN_Options[OPT_NOVCPI].value)
    {
        printf("VCPI off for DOS extenders (JEMM): ");
        MAIN_PreviousNoVCPI = MAIN_JemmSetNoVCPI(1);
        MAIN_Print_Enabled_Newline(MAIN_PreviousNoVCPI >= 0);
    }

    if(MAIN_XMSHoldKB)
        printf("XMS below 16 MB kept free for DMA buffers: %lu KB.\n", (unsigned long)MAIN_XMSHoldKB);
    MAIN_XMSRelease();

#ifdef __DJ2__
    if(MAIN_Options[OPT_XMSLOW].value && !MAIN_XMSLowInstall())
    {
        MAIN_CPrintf(RED, "Error installing /XMSLOW.\n");
        return 1;
    }
#endif
    DPMI_TSR(); // On success DOS stays resident without running atexit handlers.
    MAIN_CPrintf(RED, "Error: Failed installing TSR.\n");
    return 1;
}

int main(int argc, char* argv[])
{

    MAIN_CPrintf(CYAN, "\r\nSBGEODE ");
    MAIN_CPrintf(LIGHTCYAN, "%s", MAIN_SBEMU_VER);
    MAIN_CPrintf(LIGHTGRAY, " by ");
    MAIN_CPrintf(WHITE, "%s", GEODE_AUTHOR);
    MAIN_CPrintf(LIGHTGRAY, " | ");
    MAIN_CPrintf(WHITE, "%s\r\n", GEODE_URL);
    MAIN_CPrintf(WHITE, "Sound Blaster emulation on AMD Geode LX / CS5536 for DOS.\r\n");
    MAIN_CPrintf(LIGHTGRAY, "Based on %s by crazii (https://github.com/crazii/SBEMU).\r\n", PROGNAME);
    MAIN_CPrintf(LIGHTGRAY, "Based on MPXPlay (drivers) and DOSBox (OPL-3 emulation).\r\n");
    printf("\n");

    if((argc == 2 && stricmp(argv[1],"/?") == 0))
    {
        printf("Usage:\n");

        int i = 0;
        while(MAIN_Options[i].option)
        {
            if(MAIN_Options[i].setcmd&MAIN_SETCMD_INTRL) {++i; continue;}

            printf(" %-7s: %s", MAIN_Options[i].option, MAIN_Options[i].desc);
            if (i != 0) {
                if(MAIN_Options[i].setcmd&MAIN_SETCMD_STR)
                    printf(", default: %s.\n", (const char*)MAIN_Options[i].value);
                else if(MAIN_Options[i].setcmd&MAIN_SETCMD_BASE10)
                    printf(", default: %d.\n", MAIN_Options[i].value);
                else
                    printf(", default: %x.\n", MAIN_Options[i].value);
            } else {
                printf(".\n");
            }
            ++i;
        }

        printf("\n");
        printf("  [*] Values will default to the BLASTER variable if not specified.\n");

        return 0;
    }

    //parse BLASTER env first.
    {
        char* blaster = getenv("BLASTER");
        if(blaster != NULL)
        {
            char c;
            while((c=toupper(*(blaster++))))
            {
                if(c == 'I')
                    MAIN_Options[OPT_IRQ].value = *(blaster++) - '0';
                else if(c == 'D')
                    MAIN_Options[OPT_DMA].value = *(blaster++) - '0';
                else if(c == 'A')
                    MAIN_Options[OPT_ADDR].value = strtol(blaster, &blaster, 16);
                else if(c == 'P')
                    MAIN_Options[OPT_MPUADDR].value = strtol(blaster, &blaster, 16);
                else if(c =='T')
                    MAIN_Options[OPT_TYPE].value = *(blaster++) - '0';
                else if(c =='H')
                    MAIN_Options[OPT_HDMA].value = *(blaster++) - '0';
            }
        }
    }

    for(int i = 1; i < argc; ++i)
    {
        int matched = 0;
        for(int j = 0; j < OPT_COUNT; ++j)
        {
            int len = strlen(MAIN_Options[j].option);
            if(memicmp(argv[i], MAIN_Options[j].option, len) == 0)
            {
                if(argv[i][len] == '=' || argv[i][len] == ':') ++len; //optional separator
                int arglen = strlen(argv[i]);
                if(MAIN_Options[j].setcmd&MAIN_SETCMD_STR)
                    MAIN_Options[j].value = arglen == len ? MAIN_Options[j].value : (uintptr_t)&argv[i][len]; //default set to NULL.
                else
                {
                    int base = (MAIN_Options[j].setcmd&MAIN_SETCMD_BASE10) ? 10 : 16;
                    char* end = NULL;
                    MAIN_Options[j].value = arglen == len ? 1 : strtol(&argv[i][len], &end, base);
                    //Geode: options match by prefix, so "/VOLUME=50" used to set /VOL to 0 ("UME=50" read as a number)
                    if(arglen != len && (end == &argv[i][len] || *end != '\0'))
                    {
                        MAIN_CPrintf(RED, "Error: invalid value in %s (%s expects a %s number).\n", argv[i], MAIN_Options[j].option, base == 10 ? "decimal" : "hexadecimal");
                        return 1;
                    }
                }
                MAIN_Options[j].setcmd |= MAIN_SETCMD_SET;
                matched = 1;
                break;
            }
        }
        if(!matched) //Geode: a mistyped option used to be ignored without a word
            MAIN_CPrintf(YELLOW, "Warning: unknown option %s ignored (/? lists the options).\n", argv[i]);
    }

#if MPU_DEBUG
    mpu_debug = MAIN_Options[OPT_MDBG].value;
#endif
    if (MAIN_Options[OPT_DEBUG_OUTPUT].value) {
        if (update_serial_debug_output()) {
            return 1;
        }
    }

    if (MAIN_Options[OPT_COMPORTLIST].value) {
      ser_print_com_ports();
      return 0;
    }

    if (MAIN_Options[OPT_MPUCOMPORT].value) {
        if (update_serial_mpu_output()) {
            return 1;
        }
    }

    if(MAIN_Options[OPT_ADDR].value != 0x220 && MAIN_Options[OPT_ADDR].value != 0x240)
    {
        MAIN_CPrintf(RED, "Error: Invalid IO port address: %x.\n", MAIN_Options[OPT_ADDR].value);
        return 1;
    }
    if(MAIN_Options[OPT_IRQ].value != 0x5 && MAIN_Options[OPT_IRQ].value != 0x7 && MAIN_Options[OPT_IRQ].value != 0x9)
    {
        MAIN_CPrintf(RED, "Error: invalid IRQ: %d.\n", MAIN_Options[OPT_IRQ].value);
        return 1;
    }
    if(MAIN_Options[OPT_DMA].value != 0x0 && MAIN_Options[OPT_DMA].value != 0x1 && MAIN_Options[OPT_DMA].value != 0x3)
    {
        MAIN_CPrintf(RED, "Error: invalid DMA channel.\n");
        return 1;
    }
    if(MAIN_Options[OPT_TYPE].value <= 0 || MAIN_Options[OPT_TYPE].value > 6)
    {
        MAIN_CPrintf(RED, "Error: invalid SB Type.\n");
        return 1;
    }
    if(MAIN_Options[OPT_OUTPUT].value != 0 && MAIN_Options[OPT_OUTPUT].value != 1)
    {
        MAIN_CPrintf(RED, "Error: Invalid Output.\n");
        return 1;
    }
    if(MAIN_Options[OPT_VOL].value < 0 || MAIN_Options[OPT_VOL].value > SBEMU_VOLUME_MAX)
    {
        MAIN_CPrintf(RED, "Error: Invalid Volume.\n");
        return 1;
    }
    if(MAIN_Options[OPT_SFXVOL].value < 0 || MAIN_Options[OPT_SFXVOL].value > 100)
    {
        MAIN_CPrintf(RED, "Error: Invalid PCM effect level.\n");
        return 1;
    }
    if(MAIN_Options[OPT_FILT].value < 0 || MAIN_Options[OPT_FILT].value > 2)
    {
        MAIN_CPrintf(RED, "Error: Invalid /FILT value.\n");
        return 1;
    }
    if(MAIN_Options[OPT_OPLQ].value < 0 || MAIN_Options[OPT_OPLQ].value > 2)
    {
        MAIN_CPrintf(RED, "Error: Invalid /FMQ value.\n");
        return 1;
    }
    if(MAIN_Options[OPT_VIRQ].value < 0 || MAIN_Options[OPT_VIRQ].value > 1)
    {
        MAIN_CPrintf(RED, "Error: Invalid /VIRQ value.\n");
        return 1;
    }
#ifdef __DJ2__
    if(MAIN_Options[OPT_XMSLOW].value < 0 || MAIN_Options[OPT_XMSLOW].value > 1)
    {
        MAIN_CPrintf(RED, "Error: /XMSLOW accepts 0 or 1.\n");
        return 1;
    }
#endif
    if(MAIN_Options[OPT_NOVCPI].value < 0 || MAIN_Options[OPT_NOVCPI].value > 1)
    {
        MAIN_CPrintf(RED, "Error: Invalid /NOVCPI value.\n");
        return 1;
    }
    if(MAIN_Options[OPT_MSHARE].value < 4 || MAIN_Options[OPT_MSHARE].value > 14)
    {
        MAIN_CPrintf(RED, "Error: Invalid /MSHARE value.\n");
        return 1;
    }
    if(MAIN_Options[OPT_SWAP].value < 0 || MAIN_Options[OPT_SWAP].value > 2)
    {
        MAIN_CPrintf(RED, "Error: Invalid /SWAP value.\n");
        return 1;
    }
    if(MAIN_Options[OPT_RATE].value < 4000 || MAIN_Options[OPT_RATE].value > 192000)
    {
        MAIN_CPrintf(RED, "Error: Invalid Sample rate.\n");
        return 1;
    }
    if(MAIN_Options[OPT_HDMA].value < 5 && MAIN_Options[OPT_HDMA].value != MAIN_Options[OPT_DMA].value) //16 bit transfer through 8 bit dma
    {
        printf("Warning: HDMA using 8 bit channel: H=%d, "
        "using 5/6/7 is recommended.\n"
        "set both DMA & HDMA to %d to resolve conflicts\n", MAIN_Options[OPT_HDMA].value, MAIN_Options[OPT_DMA].value);
        MAIN_Options[OPT_HDMA].value = MAIN_Options[OPT_DMA].value; //only one low DMA channel allowed, use same channel for hdma & low dma.
    }

    DPMI_Init();

    MAIN_QEMM_Present = TRUE;
    if(MAIN_Options[OPT_RM].value)
    {
        MAIN_Options[OPT_RM].value = TRUE; //set to known value for compare
        int qemm_version = QEMM_GetVersion();
        int qemm_major = qemm_version >> 8;
        int qemm_minor = qemm_version & 0xFF;

        _LOG("QEMM version: %x.%02x\n", qemm_major, qemm_minor);

        if (qemm_version < 0x0703) {
            if (qemm_major == 0) {
                printf("QEMM or QPIEMU not installed, disabling real mode support.\n");
            } else {
                printf("QEMM or QPIEMU version below 7.03: %d.%02d, disabling real mode support.\n", qemm_major, qemm_minor);
            }

            MAIN_Options[OPT_RM].value = FALSE;
            MAIN_QEMM_Present = FALSE;
        }
    }
    MAIN_HDPMI_Present = FALSE;
    if(MAIN_Options[OPT_PM].value)
    {
        MAIN_Options[OPT_PM].value = TRUE; //set to known value for compare
        BOOL hasHDPMI = HDPMIPT_Detect(); //another DPMI host used other than HDPMI
        if(!hasHDPMI)
            printf("HDPMI not installed, disabling protected mode support.\n");
        MAIN_Options[OPT_PM].value = hasHDPMI;
        MAIN_HDPMI_Present = hasHDPMI;
    }
    MAIN_Options[OPT_OPL].value = !!MAIN_Options[OPT_OPL].value;

#if SBEMU_VMPU

    //HW MPU is the default, if VMPU being set, override HW MPU. othewise error.
    //should be checked before MAIN_TSR_InstallationCheck
    if((MAIN_Options[OPT_SCMPU].value || (MAIN_Options[OPT_MPUCOMPORT].value != 9 && MAIN_Options[OPT_MPUCOMPORT].value != 0)) && VMPU_ENABLED)
    {
        MAIN_CPrintf(RED, "Conflict settings: %s with %s or %s.\n", 
            MAIN_Options[OPT_VMPU_VOICES].option, MAIN_Options[OPT_SCMPU].option, MAIN_Options[OPT_MPUCOMPORT].option);
        return -1;
    }
    if(VMPU_ENABLED) //default: /VMPU without params
        MAIN_Options[OPT_VMPU_VOICES].value = 64;
    if(!MAIN_Options[OPT_VMPU_SF].value) //NULL="/VMSF"
        MAIN_Options[OPT_VMPU_SF].value = (uintptr_t)VMPU_DEF_SF2;
#endif

    //TSR installation check: update parameter & exit if already installed
    MAIN_TSR_InstallationCheck();

    /* Cache while still in foreground, even with /OPL=0. Rate changes later
     * occur in the resident interrupt handler and must never wait for IRQ0. */
    MAIN_CPUMHz = MAIN_CalibrateCPU();
    if(MAIN_CPUMHz <= 0)
    {
        fprintf(stderr, "Cannot calibrate CPU: BIOS clock unavailable.\n");
        return 1;
    }

    MAIN_XMSHold();
    atexit(MAIN_XMSRelease); //every exit path of the install gives the held block back

#if SBEMU_VMPU && MAIN_VMPU_HDPMI_MEMFIX
    MAIN_VMPU_ALLOCMEM();
#endif

    MAIN_SetBlasterEnv(MAIN_Options);

    BOOL enablePM = MAIN_Options[OPT_PM].value;
    BOOL enableRM = MAIN_Options[OPT_RM].value;
    if(!enablePM && !enableRM)
    {
        MAIN_CPrintf(RED, "Both real mode & protected mode support are disabled, exiting.\r\n");
        return 1;
    }

    if(MAIN_Options[OPT_SCLIST].value)
        aui.card_controlbits |= AUINFOS_CARDCNTRLBIT_TESTCARD; //note: this bit will make aui.card_handler == NULL and quit.
    if(MAIN_Options[OPT_SCFM].value)
        aui.card_select_index_fm = MAIN_Options[OPT_SCFM].value;
    if(MAIN_Options[OPT_SCMPU].value)
        aui.card_select_index_mpu401 = MAIN_Options[OPT_SCMPU].value;
    if(MAIN_Options[OPT_SC].value)
        aui.card_select_index = MAIN_Options[OPT_SC].value;
    aui.card_select_config = MAIN_Options[OPT_OUTPUT].value;
    AU_init(&aui, &fm_aui, &mpu401_aui);
    if(!aui.card_handler)
        return 1;
    atexit(&MAIN_Cleanup);

    if(aui.card_irq > 15) //UEFI with CSM may have APIC enabled (16-31) - but we need read APIC, not implemented for now.
    {
        printf("Invalid Sound card IRQ: ");
        MAIN_CPrintf(RED, "%d", aui.card_irq);
        printf(", Trying to assign a valid IRQ...\n");
        aui.card_irq = pcibios_AssignIRQ(aui.card_pci_dev);
        if(aui.card_irq == 0xFF)
        {
            MAIN_CPrintf(RED, "Failed to assign a valid IRQ for sound card, abort.\n");
            return 1;
        }
        printf("Sound card IRQ assigned: ");
        MAIN_CPrintf(LIGHTGREEN, "%d", aui.card_irq);
        printf(".\n");
    }
    if(aui.card_irq == MAIN_Options[OPT_IRQ].value)
    {
        printf("Sound card IRQ %d conflict with options /i%d, abort.\n", aui.card_irq, aui.card_irq);
        printf("Please try use /i5 or /i7 switch, or disable some onboard devices in the BIOS settings to release IRQs.\n");
        return 1;
    }
    printf("Real mode support: ");
    MAIN_Print_Enabled_Newline(enableRM);

    printf("Protected mode support: ");
    MAIN_Print_Enabled_Newline(enablePM);

    if(enablePM) //prefer PM IO since there's no mode switch and thus more faster. previously QEMM IO was used to avoid bugs/crashes.
    {
        UntrappedIO_OUT_Handler = &HDPMIPT_UntrappedIO_Write;
        UntrappedIO_IN_Handler = &HDPMIPT_UntrappedIO_Read;
    }
    else
    {
        UntrappedIO_OUT_Handler = &QEMM_UntrappedIO_Write;
        UntrappedIO_IN_Handler = &QEMM_UntrappedIO_Read;
    }

    if(MAIN_Options[OPT_OPL].value)
    {
        if (!(fm_aui.fm && fm_aui.fm_port == 0x388)) {
            QEMM_IODT *iodt = fm_aui.fm ? MAIN_HW_OPL3IODT : MAIN_OPL3IODT;
            if(enableRM && !(OPLRMInstalled=QEMM_Install_IOPortTrap(iodt, 4, &OPL3IOPT)))
            {
                MAIN_CPrintf(RED, "Error: Failed installing IO port trap for QEMM.\n");
                return 1;
            }
            if(enablePM && !(OPLPMInstalled=HDPMIPT_Install_IOPortTrap(0x388, 0x38B, iodt, 4, &OPL3IOPT_PM)))
            {
                MAIN_CPrintf(RED, "Error: Failed installing IO port trap for HDPMI.\n");
                return 1;
            }
        } else {
            printf("Not installing IO port trap. Using hardware OPL3 at port 388.\n");
        }

        const char *emutype = fm_aui.fm ? "hardware" : "emulation";
        char hwdesc[64];
        hwdesc[0] = '\0';
        if (fm_aui.fm)
          sprintf(hwdesc, "(%d:%s)", fm_aui.card_test_index, fm_aui.card_handler->shortname);
        printf("OPL3 %s%s at port 388: ", emutype, hwdesc);
        MAIN_Print_Enabled_Newline(true);
    }

    if(MAIN_Options[OPT_MPUADDR].value && 
        (MAIN_Options[OPT_MPUCOMPORT].value || VMPU_ENABLED)
    )
    {
#if SBEMU_VMPU
        if(VMPU_ENABLED)
        {
            #if MAIN_VMPU_HDPMI_MEMFIX
            MAIN_VMPU_FREEMEM(); //not needed because there is an init, will re-alloc if init failed.
            #endif

            const char* sf = (const char*)MAIN_Options[OPT_VMPU_SF].value;
            char fullpath[_MAX_PATH];
            if( VMPU_Init(MAIN_Options[OPT_MPUADDR].value, &MAIN_Options[OPT_VMPU_VOICES].value, aui.freq_card, get_abs_path(fullpath, sizeof(fullpath), sf) ) )
            {
                MAIN_CPrintf(LIGHTGRAY, "MPU voices: %d, SoundFont: ", MAIN_Options[OPT_VMPU_VOICES].value);
                MAIN_CPrintf(LIGHTBLUE, "%s\n", fullpath);
            }
            else
            {
                MAIN_CPrintf(YELLOW, "VMPU warning: sound font file not found: %s, functions disabled.\n", fullpath);
                #if MAIN_VMPU_HDPMI_MEMFIX
                MAIN_VMPU_ALLOCMEM();
                #endif
            }
            //always set
            MAIN_MPUIODT[0].handler = &VMPU_MPU; //330
            MAIN_MPUIODT[1].handler = &VMPU_MPU; //331
        }
#endif

        for(int i = 0; i < countof(MAIN_MPUIODT); ++i) MAIN_MPUIODT[i].port = MAIN_Options[OPT_MPUADDR].value+i;
        if(enableRM && !QEMM_Install_IOPortTrap(MAIN_MPUIODT, 2, &MPUIOPT))
        {
            MAIN_CPrintf(RED, "Error: Failed installing MPU-401 IO port trap for QEMM.\n");
            return 1;
        }
        if(enablePM && !HDPMIPT_Install_IOPortTrap(MAIN_Options[OPT_MPUADDR].value, MAIN_Options[OPT_MPUADDR].value+1, MAIN_MPUIODT, 2, &MPUIOPT_PM))
        {
            MAIN_CPrintf(RED, "Error: Failed installing MPU-401 IO port trap for HDPMI.\n");
            return 1;
        }

        const char *emutype = mpu401_aui.mpu401 ? "hardware" : "emulation";
        char hwdesc[64];
        hwdesc[0] = '\0';
        if (mpu401_aui.mpu401)
          sprintf(hwdesc, "(%d:%s)", mpu401_aui.card_test_index, mpu401_aui.card_handler->shortname);
        printf("MPU-401 UART %s%s at address %x: ",
               emutype, hwdesc, MAIN_Options[OPT_MPUADDR].value);
        MAIN_Print_Enabled_Newline(true);
    }

    VIRQ_Init();

    MAIN_SbemuExtFun.StartPlayback = NULL; //not used
    MAIN_SbemuExtFun.RaiseIRQ = NULL;
    MAIN_SbemuExtFun.DMA_Size = &VDMA_GetCounter;
    MAIN_SbemuExtFun.DMA_Write = &VDMA_WriteData;
    SBEMU_Init(
        MAIN_Options[OPT_IRQ].value,
        MAIN_Options[OPT_DMA].value,
        MAIN_Options[OPT_HDMA].value,
        MAIN_SB_DSPVersion[MAIN_Options[OPT_TYPE].value],
        MAIN_Options[OPT_FIX_TC].value,
        &MAIN_SbemuExtFun);
    VDMA_Virtualize(MAIN_Options[OPT_DMA].value, TRUE);
    VDMA_Virtualize(MAIN_Options[OPT_HDMA].value, TRUE);
    for(int i = 0; i < countof(MAIN_SB_IODT); ++i)
        MAIN_SB_IODT[i].port += MAIN_Options[OPT_ADDR].value;
    QEMM_IODT* SB_Iodt = MAIN_Options[OPT_OPL].value ? MAIN_SB_IODT : MAIN_SB_IODT+4;
    int SB_IodtCount = MAIN_Options[OPT_OPL].value ? countof(MAIN_SB_IODT) : countof(MAIN_SB_IODT)-4;

    {
      char hwdesc[64];
      hwdesc[0] = '\0';
      if (aui.pcm)
        sprintf(hwdesc, "(%d:%s)", aui.card_test_index, aui.card_handler->shortname);
      printf("SB %s%s emulation at address %x, IRQ %x, DMA %x: ",
             MAIN_SBTypeString[MAIN_Options[OPT_TYPE].value],
             hwdesc,
             MAIN_Options[OPT_ADDR].value,
             MAIN_Options[OPT_IRQ].value,
             MAIN_Options[OPT_DMA].value);
    }
    MAIN_Print_Enabled_Newline(true);

    return MAIN_Install(enableRM, enablePM, SB_Iodt, SB_IodtCount);
}

//with the modified fork of HDPMI, HDPMI will route PM interrupts to IVT.
//IRQ routing path:
//PM: IDT -> PM handlers after SBEMU -> SBEMU MAIN_InterruptPM(*) -> PM handlers befoe SBEMU -> IVT -> SBEMU MAIN_InterruptRM(*) -> DPMI entrance -> IVT handlers before DPMI installed
//RM: IVT -> RM handlers before SBEMU -> SBEMU MAIN_InterruptRM(*) -> RM handlers after SBEMU -> DPMI entrance -> PM handlers after SBEMU -> SBEMU MAIN_InterruptPM(*) -> PM handlers befoe SBEMU -> IVT handlers before DPMI installed
//(*) means SBEMU might early terminate the calling chain if sound irq is handled (when MAIN_ISR_CHAINED==0).
//early terminating is OK because PCI irq are level triggered, IRQ signal will keep high (raised) unless the hardware IRQ is ACKed.

static void MAIN_InterruptPM()
{
    const uint8_t irq = PIC_GetIRQ();
    if(irq != aui.card_irq) //shared IRQ handled by other handlers(EOI sent) or new irq arrived after EOI but not for us
        return;

    //if(MAIN_InINT&MAIN_ININT_PM) return; //skip reentrance. go32 will do this so actually we don't need it
    //DBG_Log("INTPM %d\n", MAIN_InINT);
    MAIN_InINT |= MAIN_ININT_PM;

    //note: we have full control of the calling chain, if the irq belongs to the sound card,
    //we send EOI and skip calling the chain - it will be a little faster. if other devices raises irq at the same time,
    //the interrupt handler will entered again (not nested) so won't be a problem.
    //also we send EOI on our own and terminate, this doesn't rely on the default implementation in IVT - some platform (i.e. VirtualBox)
    //don't send EOI on default handler in IVT.
    //
    //it has one problem that if other drivers (shared IRQ) enables interrupts (because it needs wait or is time consuming)
    //then because we're still in MAIN_InterruptPM, so MAIN_InterruptPM is never entered again (guarded by go32 or MAIN_ININT_PM),
    //so the newly coming irq will never be processed and the IRQ will flood the system (freeze)
    //an alternative chained methods will EXIT MAIN_InterruptPM FIRST and calls next handler, which will avoid this case, see @MAIN_ISR_CHAINED
    //but we need a hack if the default handler in IVT doesn't send EOI or masks the irq - this is done in the RM final wrapper, see @DPMI_RMISR_ChainedWrapper

    //MAIN_IntContext.EFLAGS |= (MAIN_InINT&MAIN_ININT_RM) ? (MAIN_IntContext.EFLAGS&CPU_VMFLAG) : 0;
    HDPMIPT_GetInterrupContext(&MAIN_IntContext);
    if(/*!(MAIN_InINT&MAIN_ININT_RM) && */aui.card_handler->irq_routine && aui.card_handler->irq_routine(&aui)) //check if the irq belong the sound card
    {
        SETIF();
        MAIN_Interrupt();
        RESETIF();
        PIC_SendEOIWithIRQ(aui.card_irq); //some BIOS driver doesn't works well if not sending EOI, there's extra check for EOI in DPMI_RMISR_ChainedWrapper
    }
    #if !MAIN_ISR_CHAINED
    else
    {
        if(/*(MAIN_InINT&MAIN_ININT_RM) || */(MAIN_IntContext.EFLAGS&CPU_VMFLAG))
            DPMI_CallOldISR(&MAIN_IntHandlePM);
        else
            DPMI_CallOldISRWithContext(&MAIN_IntHandlePM, &MAIN_IntContext.regs);
        PIC_UnmaskIRQ(aui.card_irq);
    }
    #endif
    //DBG_Log("INTPME %d\n", MAIN_InINT);
    MAIN_InINT &= ~MAIN_ININT_PM;
}

static void MAIN_InterruptRM()
{
    const uint8_t irq = PIC_GetIRQ();
    if(irq != aui.card_irq) //shared IRQ handled by other handlers(EOI sent) or new irq arrived after EOI but not for us
        return;

    //if(MAIN_InINT&MAIN_ININT_RM) return; //skip reentrance. go32 will do this so actually we don't need it
    //DBG_Log("INTRM %d\n", MAIN_InINT);
    MAIN_InINT |= MAIN_ININT_RM;

    if(/*!(MAIN_InINT&MAIN_ININT_PM) && */aui.card_handler->irq_routine && aui.card_handler->irq_routine(&aui)) //check if the irq belong the sound card
    {
        MAIN_IntContext.regs = MAIN_RMIntREG;
        MAIN_IntContext.EFLAGS = MAIN_RMIntREG.w.flags | CPU_VMFLAG;
        SETIF();
        MAIN_Interrupt();
        RESETIF();
        PIC_SendEOIWithIRQ(aui.card_irq); //some BIOS driver doesn't works well if not sending EOI, there's extra check for EOI in DPMI_RMISR_ChainedWrapper
    }
    #if !MAIN_ISR_CHAINED
    else
    {
        DPMI_REG r = MAIN_RMIntREG; //don't modify MAIN_RMIntREG on hardware interrupt
        DPMI_CallRealModeOldISR(&MAIN_IntHandleRM, &r);
        PIC_UnmaskIRQ(aui.card_irq);
    }
    #endif
    //DBG_Log("INTRME %d\n", MAIN_InINT);
    MAIN_InINT &= ~MAIN_ININT_RM;
}

static void MAIN_Interrupt()
{
    if(!(aui.card_infobits&AUINFOS_CARDINFOBIT_PLAYING))
        return;
    uint32_t isr_t0 = MAIN_TSC();
#if SBEMU_VMPU
    {   //Geode: measure the buffer period in TSC cycles (CPU speed independent) as
        //the minimum of 32-period averages; ISR time budget of the VMPU is derived from it
        static uint32_t last_t0, blk_sum, period_est;
        static int blk_n;
        uint32_t d = isr_t0 - last_t0;
        last_t0 = isr_t0;
        if(d > 60000000UL) {blk_n = 0; blk_sum = 0;} //idle gap or first call
        else if(d > 0)
        {
            blk_sum += d;
            if(++blk_n >= 32)
            {
                uint32_t avg = blk_sum / 32;
                if(!period_est || avg < period_est)
                    period_est = avg;
                blk_n = 0; blk_sum = 0;
            }
        }
        if(VMPU_ENABLED)
            VMPU_BeginPeriod(isr_t0, period_est);
    }
#endif

    if(SBEMU_IRQTriggered())
    {
        MAIN_InvokeIRQ(SBEMU_GetIRQ());
        SBEMU_SetIRQTriggered(FALSE);
    }
    int32_t vol_l, vol_r;
    int32_t voicevol_l, voicevol_r;
    int32_t midivol_l, midivol_r;
    int32_t cdvol; // 0-100 (percentage)
    static int32_t last_cdvol = -1;
    if(MAIN_Options[OPT_TYPE].value == 1 || MAIN_Options[OPT_TYPE].value == 3) //SB2.0 and before
    {
        vol_l = vol_r = (SBEMU_GetMixerReg(SBEMU_MIXERREG_MASTERVOL) >> 1)*256/7;
        voicevol_l = voicevol_r = (SBEMU_GetMixerReg(SBEMU_MIXERREG_VOICEVOL) >> 1)*256/3;
        midivol_l = midivol_r = (SBEMU_GetMixerReg(SBEMU_MIXERREG_MIDIVOL) >> 1)*256/7;
        cdvol = (SBEMU_GetMixerReg(SBEMU_MIXERREG_CDVOL) >> 1)*100/7;
    }
    else if(MAIN_Options[OPT_TYPE].value == 6) //SB16
    {
        // CT1745 5-bit controls attenuate by 2 dB per step. Odd entries
        // preserve the Q8 gains already verified for legacy 4-bit controls.
        static const uint16_t gain[32] = {
            0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6,
            8, 10, 13, 16, 20, 26, 32, 41, 51, 64, 81, 102, 128, 162, 203, 256
        };
        vol_l = gain[SBEMU_GetMixerReg(SBEMU_MIXRREG_MASTERL) >> 3];
        vol_r = gain[SBEMU_GetMixerReg(SBEMU_MIXRREG_MASTERR) >> 3];
        voicevol_l = gain[SBEMU_GetMixerReg(SBEMU_MIXRREG_VOICEL) >> 3];
        voicevol_r = gain[SBEMU_GetMixerReg(SBEMU_MIXRREG_VOICER) >> 3];
        midivol_l = gain[SBEMU_GetMixerReg(SBEMU_MIXRREG_MIDIL) >> 3];
        midivol_r = gain[SBEMU_GetMixerReg(SBEMU_MIXRREG_MIDIR) >> 3];
        cdvol = (SBEMU_GetMixerReg(SBEMU_MIXERREG_CDSTEREO)>>4)*100/15; //4:4
    }
    else //SBPro
    {
        uint8_t master = SBEMU_GetMixerReg(SBEMU_MIXERREG_MASTERSTEREO);
        uint8_t voice = SBEMU_GetMixerReg(SBEMU_MIXERREG_VOICESTEREO);
        uint8_t midi = SBEMU_GetMixerReg(SBEMU_MIXERREG_MIDISTEREO);
        vol_l = (master >> 5)*256/7;
        vol_r = ((master >> 1)&7)*256/7;
        voicevol_l = (voice >> 5)*256/7;
        voicevol_r = ((voice >> 1)&7)*256/7;
        midivol_l = (midi >> 5)*256/7;
        midivol_r = ((midi >> 1)&7)*256/7;
        cdvol = (SBEMU_GetMixerReg(SBEMU_MIXERREG_CDSTEREO)>>5)*100/7;
    }

    // User comfort control, applied after the emulated SB voice mixer.
    // Do not attenuate OPL or the physical AC'97 master/headphone output.
    voicevol_l = voicevol_l * MAIN_Options[OPT_SFXVOL].value / 100;
    voicevol_r = voicevol_r * MAIN_Options[OPT_SFXVOL].value / 100;

    if (cdvol != last_cdvol) {
        // Apply CD-Audio volume to device mixer (CDDA mixing is handled purely on soundcard,
        // so plug in your 4-pin audio cable from the CD-ROM drive to the soundcard CD-IN)
#ifdef AU_CARDS_LINK_GEODE
        if(aui.card_handler == &GEODE_sndcard_info)
        {   //integer only (no x87 in the interrupt), bounded codec wait; retry next period if it did not finish
            if(GEODE_set_cdin_isr(&aui, cdvol))
                last_cdvol = cdvol;
        }
        else
#endif
        {
            AU_setmixer_one(&aui, AU_MIXCHAN_CDIN, MIXER_SETMODE_ABSOLUTE, cdvol);
            last_cdvol = cdvol;
        }
    }

    aui.card_outbytes = aui.card_dmasize;
    int samples = AU_cardbuf_space(&aui) / sizeof(int16_t) / SBEMU_CHANNELS; //16 bit, 2 channels
    //_LOG("samples:%d\n",samples);
    if(samples == 0)
        return;

    RESETIF();
    BOOL vmpu_active = VMPU_IsActive();
    SETIF();
    
    BOOL opl_keyon = MAIN_Options[OPT_OPL].value && OPL3EMU_IsActive();
    if(opl_keyon)
        MAIN_OPLTail = aui.freq_card * MAIN_OPL_TAIL_SECONDS;
    BOOL opl_active = MAIN_Options[OPT_OPL].value && (opl_keyon || MAIN_OPLTail > 0);
    BOOL digital = SBEMU_HasStarted();
    int dma = (SBEMU_GetBits() <= 8 /*|| MAIN_Options[OPT_TYPE].value < 6*/) ? SBEMU_GetDMA() : SBEMU_GetHDMA();
    int32_t DMA_Count = VDMA_GetCounter(dma); //count in bytes
    MAIN_RSHQ = MAIN_Options[OPT_FILT].value == 2;
    if(!digital) { MAIN_LastSBRate = 0; MAIN_ResampleReset(); MAIN_LPFRate = 0; }
    if(digital)//&& DMA_Count != 0x10000) //-1(0xFFFF)+1=0
    {
        uint32_t DMA_Addr = VDMA_GetAddress(dma);
        int32_t DMA_Index = VDMA_GetIndex(dma);
        uint32_t SB_Bytes = SBEMU_GetSampleBytes();
        uint32_t SB_Pos = SBEMU_GetPos();
        uint32_t SB_Rate = SBEMU_GetSampleRate();
        if(MAIN_LastSBRate != SB_Rate)
        {
            MAIN_ResampleReset();
            MAIN_LastSBRate = SB_Rate;
        }
        int samplesize = max(1, SBEMU_GetBits()/8); //sample size in bytes 1 for 8bit. 2 for 16bit
        int channels = SBEMU_GetChannels();
        _LOG("sample rate: %d %d\n", SB_Rate, aui.freq_card);
        _LOG("channels: %d, size:%d\n", channels, samplesize);
        //_LOG("DMA index: %x\n", DMA_Index);
        //_LOG("digital start\n");
        int pos = 0;
        do
        {
            if(MAIN_DMA_MappedAddr != 0
             && !(DMA_Addr >= MAIN_DMA_Addr && DMA_Addr+DMA_Index+DMA_Count <= MAIN_DMA_Addr+MAIN_DMA_Size))
            {
                if(MAIN_DMA_MappedAddr > 1024*1024)
                    DPMI_UnmappMemory(MAIN_DMA_MappedAddr);
                MAIN_DMA_MappedAddr = 0;
            }
            if(MAIN_DMA_MappedAddr == 0)
            {
                MAIN_DMA_Addr = DMA_Addr&~0xFFF;
                MAIN_DMA_Size = align(max(DMA_Addr-MAIN_DMA_Addr+DMA_Index+DMA_Count, 64*1024*2), 4096);
                MAIN_DMA_MappedAddr = (DMA_Addr+DMA_Index+DMA_Count <= 1024*1024) ? (DMA_Addr&~0xFFF) : DPMI_MapMemory(MAIN_DMA_Addr, MAIN_DMA_Size);
            }
            //_LOG("DMA_ADDR:%x, %x, %x\n",DMA_Addr, MAIN_DMA_Addr, MAIN_DMA_MappedAddr);

            int count = samples-pos;
            BOOL resample = TRUE; //don't resample if sample rates are close
            const int want = count; //output frames still needed this interrupt
            if(SB_Rate < aui.freq_card-50 || SB_Rate > aui.freq_card+50)
                count = MAIN_ResampleNeed(count, SB_Rate, aui.freq_card); //exact source need
            else
                resample = FALSE;
            count = min(count, max(1,(DMA_Count)/samplesize/channels)); //max for stereo initial 1 byte
            count = min(count, max(1,(SB_Bytes-SB_Pos)/samplesize/channels)); //max for stereo initial 1 byte. 1/2channel = 0, make it 1
            if(SBEMU_GetBits()<8) //ADPCM 8bit
                count = max(1, count / (9 / SBEMU_GetBits()));
            _LOG("samples:%d %d %d, %d %d, %d %d\n", samples, pos+count, count, DMA_Count, DMA_Index, SB_Bytes, SB_Pos);
            int bytes = count * samplesize * channels;

            {
                int16_t* pcm = resample ? MAIN_PCMResample+channels*MAIN_PCM_RESAMPLE_INTERPOLATION  : MAIN_PCM + pos*2;
                if(MAIN_DMA_MappedAddr == 0) //map failed?
                    memset(pcm, 0, bytes);
                else
                    DPMI_LMemcpy(DPMI_PTR2L(pcm), MAIN_DMA_MappedAddr+(DMA_Addr-MAIN_DMA_Addr)+DMA_Index, bytes);
                if(SBEMU_GetBits()<8) //ADPCM  8bit
                    count = SBEMU_DecodeADPCM((uint8_t*)(pcm), bytes);
                else if(SBEMU_GetBits() == 8 && SBEMU_IsSigned()) //Geode: SB16 8-bit signed -> the unsigned the converter expects
                    for(int i = 0; i < bytes; ++i) ((uint8_t*)pcm)[i] ^= 0x80;
                else if(SBEMU_GetBits() == 16 && !SBEMU_IsSigned()) //Geode: SB16 16-bit unsigned -> signed
                    for(int i = 0; i < bytes / 2; ++i) ((uint16_t*)pcm)[i] ^= 0x8000;
                // An ADPCM reference byte consumes DMA input without producing PCM.
                if(count > 0 && samplesize != 2)
                    cv_bits_n_to_m(pcm, count*channels, samplesize, 2);
                if(count > 0 && resample)
                {
                    #if MAIN_PCM_RESAMPLE_INTERPOLATION
                    for(int i = 0; i < channels; ++i)
                    {
                        MAIN_PCMResample[i] = MAIN_LastResample[i]; //put last sample at beginning for interpolation
                        MAIN_LastResample[i] = *(pcm + (count-1)*channels + i); //record last sample
                    }
                    count += channels;
                    #endif
                    count = MAIN_ResampleStream(MAIN_PCM+pos*2, MAIN_PCM_SAMPLESIZE-pos*2, want, MAIN_PCMResample, count, channels, SB_Rate, aui.freq_card);
                }
            }
            if(channels == 1) //should be the last step
                cv_channels_1_to_n(MAIN_PCM+pos*2, count, 2, 2);
            pos += count;
            //_LOG("samples:%d %d %d\n", count, pos, samples);
            RESETIF();
            DMA_Index = VDMA_SetIndexCounter(dma, DMA_Index+bytes, DMA_Count-bytes);
            SETIF();
            DMA_Count = VDMA_GetCounter(dma);
            SB_Pos = SBEMU_SetPos(SB_Pos+bytes);
            //_LOG("SB bytes: %d %d\n", SB_Pos, SB_Bytes);
            if(SB_Pos >= SB_Bytes)
            {
                //_LOG("INT:%d,%d,%d,%d\n",MAIN_SBBytes,SBEMU_GetSampleBytes(),MAIN_DMAIndex,DMA_Count);
                //_LOG("SBEMU: Auto: %d\n",SBEMU_GetAuto());
                if(!SBEMU_GetAuto())
                    SBEMU_Stop();
                SB_Pos = SBEMU_SetPos(0);

                MAIN_InvokeIRQ(SBEMU_GetIRQ());
                if(SB_Bytes <= 32) //detection routine?
                {
                    int c = SBEMU_GetDetectionCounter();
                    if(++c >= 256) //Miles Sound will "freeze" or crash when we continually send virtual interrupt to it, it seems it processes slow and virtual interrupt keeps happening until crash.
                        SBEMU_Stop(); //fix problem when Miles Sound using SB driver on SBPro emulation
                    SBEMU_SetDetectionCounter(c);
                    break; //fix crash in virtualbox.
                }

                SB_Bytes = SBEMU_GetSampleBytes();
                SB_Pos = SBEMU_GetPos();
                SB_Rate = SBEMU_GetSampleRate();
                //incase IRQ handler re-programs DMA
                DMA_Index = VDMA_GetIndex(dma);
                DMA_Count = VDMA_GetCounter(dma);
                DMA_Addr = VDMA_GetAddress(dma);
                //_LOG("DMACount: %d, DMAIndex:%d, DMA_Addr:%x\n",DMA_Count, DMA_Index, DMA_Addr);
            }
        } while(VDMA_GetAuto(dma) && (pos < samples) && SBEMU_HasStarted());
        //_LOG("digital end %d %d\n", samples, pos);
        //for(int i = pos; i < samples; ++i)
        //    MAIN_PCM[i*2+1] = MAIN_PCM[i*2] = 0;
        samples = min(samples, pos);
        if(samples > 0)
        {
            MAIN_DacHold[0] = MAIN_PCM[(samples-1)*2];
            MAIN_DacHold[1] = MAIN_PCM[(samples-1)*2+1];
            MAIN_DacHoldOn = 1;
            MAIN_DCBlock(MAIN_PCM, samples);
        }
        if(MAIN_Options[OPT_FILT].value && SB_Rate < aui.freq_card-50 && SB_Rate <= MAIN_LPF_MAXRATE)
            MAIN_LPF_Process(MAIN_PCM, samples, SB_Rate);
        else
            MAIN_LPFRate = 0;
    }
    else if(SBEMU_GetDirectCount()>=3)
    {
        samples = SBEMU_GetDirectCount();
        _LOG("direct out:%d %d\n",samples,aui.card_samples_per_int);
        memcpy(MAIN_PCMResample, SBEMU_GetDirectPCM8(), samples);
        SBEMU_ResetDirect();
#if 0   //fix noise for some games - SBEMU-X NOTE: unlikely to be needed
        //NOTE: it's a noise during game startup that can be ignored.
        int zeros = TRUE;
        for(int i = 0; i < samples && zeros; ++i)
        {
            if(((uint8_t*)MAIN_PCM)[i] != 0)
                zeros = FALSE;
        }
        if(zeros)
        {
            for(int i = 0; i < samples; ++i)
                ((uint8_t*)MAIN_PCM)[i] = 128;
        }
#endif
        //for(int i = 0; i < samples; ++i) _LOG("%d ",((uint8_t*)MAIN_PCM)[i]); _LOG("\n");
        cv_bits_n_to_m(MAIN_PCMResample, samples, 1, 2);
        //for(int i = 0; i < samples; ++i) _LOG("%d ",MAIN_PCM[i]); _LOG("\n");
        // the actual sample rate is derived from current count of samples in direct output buffer
        samples = mixer_speed_lq(MAIN_PCM, MAIN_PCM_SAMPLESIZE, MAIN_PCMResample, samples, 1, (samples * aui.freq_card) / aui.card_samples_per_int, aui.freq_card);
        //for(int i = 0; i < samples; ++i) _LOG("%d ",MAIN_PCM[i]); _LOG("\n");
        cv_channels_1_to_n(MAIN_PCM, samples, 2, 2);
        digital = TRUE;
    }
    else if(MAIN_DacHoldOn) //Geode: the held DAC level, through the same gains as the DMA stream
    {
        for(int i = 0; i < samples; ++i)
        {
            MAIN_PCM[i*2] = MAIN_DacHold[0];
            MAIN_PCM[i*2+1] = MAIN_DacHold[1];
        }
        MAIN_DCBlock(MAIN_PCM, samples);
        //the blocker has settled: stop mixing the held level (its state stays, so the next block continues smoothly)
        if(llabs(MAIN_DCy1[0]) < 65536 && llabs(MAIN_DCy1[1]) < 65536)
            MAIN_DacHoldOn = 0;
        digital = TRUE;
    }
    else if(!opl_active && !vmpu_active)
    {
        _LOG("muted\n");
        memset(MAIN_PCM, 0, samples*sizeof(int16_t)*2); //output muted samples.
    }

    /* A real Sound Blaster keeps DMA and IRQ progress running while the DAC
       speaker is disabled, but does not route those samples to its output.
       Some old players start their DMA engine during initialization and only
       issue D1 when valid music is ready. */
    if(digital && MAIN_Options[OPT_TYPE].value != 6 && !SBEMU_GetDACSpeaker()) //SB16 ignores D1/D3 (as DOSBox)
        memset(MAIN_PCM, 0, samples*sizeof(int16_t)*2);

    /* /SWAP=1: mirror the digital PCM only, before the SB voice L/R gains (what
       the upstream build switch SBEMU_SWAP_STEREO did; for games written for
       the reversed stereo of the SB Pro). FM and MIDI keep their sides. */
    if(digital && MAIN_Options[OPT_SWAP].value == 1)
        for(int i = 0; i < samples*2; i+=2)
        {
            int16_t x = MAIN_PCM[i]; MAIN_PCM[i] = MAIN_PCM[i+1]; MAIN_PCM[i+1] = x;
        }

    if(opl_active && !fm_aui.fm
        && !vmpu_active //NOTE: skip OPL if MPU is active
    )
    {
        int16_t* pcm = digital ? MAIN_OPLPCM : MAIN_PCM;
        OPL3EMU_GenSamples(pcm, samples); //will generate samples*2 if stereo
        if(!OPL3EMU_GetMode()) //always use 2 channels, convert opl2 samples to 2 channel
            cv_channels_1_to_n(pcm, samples, 2, SBEMU_BITS/8);
        if(!opl_keyon)
        {   //release tail: stop early once the chip is fully silent
            int silent = 1;
            for(int i = 0; i < samples*2 && silent; ++i)
                if(pcm[i]) silent = 0;
            MAIN_OPLTail = silent ? 0 : MAIN_OPLTail - samples;
        }

        if(digital)
        {
            for(int i = 0; i < samples*2; i+=2)
            {
                #if !SBEMU_LINEAR_MIX
                // https://stackoverflow.com/questions/12089662/mixing-16-bit-linear-pcm-streams-and-avoiding-clipping-overflow
                int pcm_l = MAIN_PCM[i], pcm_r = MAIN_PCM[i+1];
                #if SBEMU_SWAP_STEREO
                {int x = pcm_l; pcm_l = pcm_r; pcm_r = x;}
                #endif
                int la = pcm_l * voicevol_l/256 + 32768;
                int ra = pcm_r * voicevol_r/256 + 32768;
                int lb = (int)((MAIN_OPLPCM[i]+MAIN_OPLPCM[i]*SBEMU_OPL_VOLUME_AMPLICATION/2) * midivol_l/256) + 32768;
                int rb = (int)((MAIN_OPLPCM[i+1]+MAIN_OPLPCM[i+1]*SBEMU_OPL_VOLUME_AMPLICATION/2) * midivol_r/256) + 32768;
                int l = (la < 32768 || lb < 32768) ? (la*lb/32768) : ((la+lb)*2 - la*lb/32768 - 65536);
                if(l == 65536) l = 65535;
                int r = (ra < 32768 || rb < 32768) ? (ra*rb/32768) : ((ra+rb)*2 - ra*rb/32768 - 65536);
                if(r == 65536) r = 65535;
                MAIN_PCM[i] = (l - 32768) * vol_l/256;
                MAIN_PCM[i+1] = (r - 32768) * vol_r/256;
                
                #else //simple average: sounds the same as DOSBox

                int pcm_l = MAIN_PCM[i], pcm_r = MAIN_PCM[i+1];
                #if SBEMU_SWAP_STEREO
                {int x = pcm_l; pcm_l = pcm_r; pcm_r = x;}
                #endif
                int la = pcm_l * voicevol_l/256; //SFX PCM
                int ra = pcm_r * voicevol_r/256;

                // Keep the FM branch free of dry digital PCM. A copy here
                // bypasses the voice L/R gains and collapses panned effects.
                int lb = (int)((MAIN_OPLPCM[i]+MAIN_OPLPCM[i]*SBEMU_OPL_VOLUME_AMPLICATION/2) * midivol_l/256); //OPL PCM
                int rb = (int)((MAIN_OPLPCM[i+1]+MAIN_OPLPCM[i+1]*SBEMU_OPL_VOLUME_AMPLICATION/2) * midivol_r/256);
                int l = (la*SBEMU_LINEAR_MIX_COMBINED_SFX_RATIO/SBEMU_LINEAR_MIX_FRACTION + lb*SBEMU_OPL_RATIO) * vol_l/256;
                int r = (ra*SBEMU_LINEAR_MIX_COMBINED_SFX_RATIO/SBEMU_LINEAR_MIX_FRACTION + rb*SBEMU_OPL_RATIO) * vol_r/256;
                MAIN_PCM[i] = MAIN_Sat16(l);
                MAIN_PCM[i+1] = MAIN_Sat16(r);
                #endif
            }
        }
        else for(int i = 0; i < samples*2; i+=2) //MAIN_PCM is opl here
        {
            MAIN_PCM[i] = MAIN_Sat16((MAIN_PCM[i] + MAIN_PCM[i]*SBEMU_OPL_VOLUME_AMPLICATION/2) * SBEMU_OPL_RATIO * midivol_l/256 * vol_l/256);
            MAIN_PCM[i+1] = MAIN_Sat16((MAIN_PCM[i+1] + MAIN_PCM[i+1]*SBEMU_OPL_VOLUME_AMPLICATION/2) * SBEMU_OPL_RATIO * midivol_r/256 * vol_r/256);
        }
    }
    else if(digital)
        for(int i = 0; i < samples*2; i+=2)
        {
            int16_t l = MAIN_PCM[i];
            int16_t r = MAIN_PCM[i+1];
            #if SBEMU_SWAP_STEREO
            {int x = l; l = r; r = x;}
            #endif
            MAIN_PCM[i] = MAIN_Sat16(l * voicevol_l/256 * vol_l/256 * SBEMU_SFX_RATIO);
            MAIN_PCM[i+1] = MAIN_Sat16(r * voicevol_r/256 * vol_r/256 * SBEMU_SFX_RATIO);
        }

#if SBEMU_VMPU
    if(vmpu_active)
    {
        RESETIF();
        VMPU_GenSamples(MAIN_PCM, samples, aui.freq_card, digital);
        SETIF();
    }
#endif

#if SBEMU_PCSPEAKER
    // Mix PC Speaker output on top of whatever is already in MAIN_PCM.
    // VPCSPEAKER_GenSamples() uses additive saturating mix (domix=1), so it is
    // safe even when the SB/OPL stream is currently silent.
    if(VPCSPEAKER_IsActive())
    {
        VPCSPEAKER_GenSamples(MAIN_PCM, samples, aui.freq_card, /*domix=*/1);
    }
#endif

    samples *= 2; //to stereo

    aui.samplenum = samples;
    /* /SWAP=2: mirror the finished mix (PCM, FM and MIDI), for a reversed jack. */
    if(MAIN_Options[OPT_SWAP].value == 2)
        for(int i = 0; i < samples; i+=2) //samples counts int16 values here
        {
            int16_t x = MAIN_PCM[i]; MAIN_PCM[i] = MAIN_PCM[i+1]; MAIN_PCM[i+1] = x;
        }
    aui.pcm_sample = MAIN_PCM;
    AU_writedata(&aui);

    {
        uint32_t dt = MAIN_TSC() - isr_t0;
        if(dt > MAIN_IsrMax) MAIN_IsrMax = dt;
        MAIN_IsrSum += dt;
        ++MAIN_IsrCount;
    }
    //_LOG("MAIN INT END\n");
}

void MAIN_TSR_InstallationCheck()
{
    for(int i = MAIN_TSR_INTSTART_ID; i <= 0xFF; ++i)
    { //detect TSR existence
        DPMI_REG r = {0};
        r.h.ah = i;
        DPMI_CallRealModeINT(MAIN_TSR_INT, &r);
        if(r.h.al == 0)
            continue;
        //printf("DOSID:%x\n", DPMI_SEGOFF2L(r.w.dx, r.w.di));
        if(DPMI_LMemcmp(DPMI_SEGOFF2L(r.w.dx, r.w.di), DPMI_PTR2L((char*)MAIN_ISR_DOSID_String), 16) == 0)
        {
            printf("SBGEODE is active.\n"); /* Geode: the resident label MAIN_ISR_DOSID_String stays "Crazii  SBEMU" */

            //note: copy array won't copy string pointers in the array, don't reference them, use string in MAIN_Options.
            //except for MAIN_SETCMD_STR, they are converted to linear addresses.
            struct MAIN_OPT* opt = (struct MAIN_OPT*)alloca(sizeof(MAIN_Options));
            r.h.ah = i;
            r.h.al = 0x01; //get current settings
            r.d.ebx = DPMI_PTR2L(opt); //dest
            r.d.ecx = sizeof(MAIN_Options); //size: in case that a different version of SBEMU is installed
            DPMI_CallRealModeINT(MAIN_TSR_INT, &r);
            if((r.w.flags&CPU_CFLAG))
            {
                MAIN_CPrintf(RED, "Failed to get SBEMU TSR information.\n");
                exit(-1);
            }

#ifdef __DJ2__
            if((MAIN_Options[OPT_XMSLOW].setcmd & MAIN_SETCMD_SET) &&
               MAIN_Options[OPT_XMSLOW].value != opt[OPT_XMSLOW].value)
            {
                MAIN_CPrintf(RED, "/XMSLOW is fixed at installation. Reboot DOS to change it.\n");
                exit(1);
            }
#endif
            for(int j = 0; j < OPT_COUNT; ++j)
            {
                if(!(MAIN_Options[j].setcmd&MAIN_SETCMD_SET) || (MAIN_Options[j].setcmd&MAIN_SETCMD_HIDDEN))
                    continue;
                if((MAIN_Options[j].setcmd&MAIN_SETCMD_STR) && 
                    ( (MAIN_Options[j].setcmd&MAIN_SETCMD_STRCS) && DPMI_LStrcmp(opt[j].value, DPMI_PTR2L((char*)MAIN_Options[j].value)) != 0
                        || DPMI_LStricmp(opt[j].value, DPMI_PTR2L((char*)MAIN_Options[j].value)) != 0 ) )
                {
                    uint32_t len = DPMI_LStrlen(opt[j].value);
                    char* buf = alloca(len+1);
                    DPMI_LMemcpy(DPMI_PTR2L(buf), opt[j].value, len+1);
                    MAIN_CPrintf(WHITE, "%8s", MAIN_Options[j].option); MAIN_CPrintf(LIGHTGRAY, " changed from ");
                    MAIN_CPrintf(RED, "%s", buf); MAIN_CPrintf(LIGHTGRAY, " to "); MAIN_CPrintf(LIGHTGREEN, "%s\n", (const char*)MAIN_Options[j].value);
                    opt[j].value = DPMI_PTR2L((char*)MAIN_Options[j].value);
                }
                else if(!(MAIN_Options[j].setcmd&MAIN_SETCMD_STR) && MAIN_Options[j].value != opt[j].value)
                {
                    int radix10 = MAIN_Options[j].setcmd&MAIN_SETCMD_BASE10;
                    MAIN_CPrintf(WHITE, "%8s", MAIN_Options[j].option); MAIN_CPrintf(LIGHTGRAY, " changed from ");
                    MAIN_CPrintf(RED, radix10 ? "%d" : "%x", opt[j].value); MAIN_CPrintf(LIGHTGRAY, " to ");
                    MAIN_CPrintf(LIGHTGREEN, radix10 ? "%d\n" : "%x\n", MAIN_Options[j].value);
                    opt[j].value = MAIN_Options[j].value;
                }
            }

#if SBEMU_VMPU //special handling for VMPU. the TSR has freed its RM translation buffer so it cannot do IO in PM.
            opt[OPT_VMPU_SFBUF].value = 0;
            if(MAIN_Options[OPT_VMPU_VOICES].value && MAIN_Options[OPT_MPUADDR].value)
            {
                char path[_MAX_PATH];
                const char* sf = (const char*)MAIN_Options[OPT_VMPU_SF].value;
                uint32_t len = 0;
                void* sf_data = load_file(get_abs_path(path, sizeof(path), sf), sizeof(uint32_t), &len); //freed on exit
                if(!sf_data)
                {
                    MAIN_CPrintf(RED, "Failed to load Soundfont file to memory: %s\n", path); //OOM? should be streaming but unable to do that.
                    exit(-1);
                }
                *(uint32_t*)sf_data = len;
                opt[OPT_VMPU_SFBUF].value = DPMI_PTR2L(sf_data);
            }
#endif

            if(MAIN_Options[OPT_NOVCPI].setcmd&MAIN_SETCMD_SET)
            {
                printf("VCPI (JEMM): ");
                if(MAIN_JemmSetNoVCPI(MAIN_Options[OPT_NOVCPI].value) >= 0)
                    MAIN_CPrintf(LIGHTGREEN, "%s.\n", MAIN_Options[OPT_NOVCPI].value ? "off" : "on");
                else
                    MAIN_CPrintf(LIGHTRED, "JEMM not found.\n");
            }
            if(MAIN_Options[OPT_RESET].value)
                printf("Resetting sound card driver...\n");
            printf("\n");

            if((MAIN_Options[OPT_ADDR].setcmd&MAIN_SETCMD_SET) || (MAIN_Options[OPT_IRQ].setcmd&MAIN_SETCMD_SET) || (MAIN_Options[OPT_DMA].setcmd&MAIN_SETCMD_SET) || (MAIN_Options[OPT_TYPE].setcmd&MAIN_SETCMD_SET) || (MAIN_Options[OPT_HDMA].setcmd&MAIN_SETCMD_SET))
                MAIN_SetBlasterEnv(opt);

            r.h.ah = i;
            r.h.al = 0x02; //set new settings
            r.d.ebx = DPMI_PTR2L(opt);
            r.d.ecx = sizeof(MAIN_Options);
            DPMI_CallRealModeINT(MAIN_TSR_INT, &r);
            if((r.w.flags&CPU_CFLAG))
            {
                MAIN_CPrintf(RED, "Failed to update settings to SBEMU TSR.\n");
                exit(-1);
            }

            r.h.ah = i;
            r.h.al = 0x01; //read back to confirm
            r.d.ebx = DPMI_PTR2L(opt);
            r.d.ecx = sizeof(MAIN_Options);
            DPMI_CallRealModeINT(MAIN_TSR_INT, &r);
            if((r.w.flags&CPU_CFLAG))
            {
                MAIN_CPrintf(RED, "Failed to get SBEMU TSR information.\n");
                exit(-1);
            }

            printf("Current settings:\n");
            for(int i = OPT_Help+1; i < OPT_COUNT; ++i)
            {
                if(!(MAIN_Options[i].setcmd&MAIN_SETCMD_HIDDEN))
                {
                    if((MAIN_Options[i].setcmd&MAIN_SETCMD_STR))
                    {
                        uint32_t len = DPMI_LStrlen(opt[i].value);
                        char* buf = alloca(len+1);
                        DPMI_LMemcpy(DPMI_PTR2L(buf), opt[i].value, len+1);
                        printf("%-8s: %s\n", MAIN_Options[i].option, buf);
                    }
                    else
                        printf( (MAIN_Options[i].setcmd&MAIN_SETCMD_BASE10) ? "%-8s: %d\n":"%-8s: %x\n", MAIN_Options[i].option, opt[i].value);
                }
            }
            exit(0);
        }
    }
}

static void MAIN_TSR_Interrupt()
{
    if(MAIN_TSRREG.h.ah != MAIN_TSR_INT_FNO)
    {
        //DBG_Log("chain %x ",MAIN_TSRREG.d.eax);
        //DBG_DumpREG(&MAIN_TSRREG);
        DPMI_CallRealModeOldISR(&MAIN_TSRIntHandle, &MAIN_TSRREG);
        return;
    }
    //DBG_Log("call tsr\n");
    switch(MAIN_TSRREG.h.al)
    {
        case 0x00:  //AMIS installation check
            MAIN_TSRREG.h.al = 0xFF;
            MAIN_TSRREG.w.cx = 0x0100; //bcd version
            MAIN_TSRREG.w.dx = (MAIN_ISR_DOSID&0xFFFF); //a real mode string buffer of identification: segment
            MAIN_TSRREG.w.di = 0; //offset
        return;
        case 0x01: //query
        {
            if(MAIN_TSRREG.d.ecx != sizeof(MAIN_Options)) //different version of SBEMU?
            {
                MAIN_TSRREG.w.flags |= CPU_CFLAG;
                return;
            }
            MAIN_TSRREG.w.flags &= ~CPU_CFLAG;
            //convert string to linear addr, for the SBEMU client to access
            for(int i = 0; i < countof(MAIN_Options); ++i)
            {
                if((MAIN_Options[i].setcmd&MAIN_SETCMD_STR))
                    MAIN_Options[i].value = DPMI_PTR2L((char*)MAIN_Options[i].value);
            }
            DPMI_LMemcpy(MAIN_TSRREG.d.ebx, DPMI_PTR2L(MAIN_Options), sizeof(MAIN_Options));

            for(int i = 0; i < countof(MAIN_Options); ++i) //convert back
            {
                if((MAIN_Options[i].setcmd&MAIN_SETCMD_STR))
                    MAIN_Options[i].value = (uintptr_t)DPMI_L2PTR(MAIN_Options[i].value);
            }
        }
        return;
        case 0x03: //read-only Geode telemetry, BX=selector
        {
#ifdef AU_CARDS_LINK_GEODE
            uint32_t value;
            if(aui.card_handler == &GEODE_sndcard_info)
            {
                if(MAIN_TSRREG.w.bx == 8)
                    value = MAIN_VirtualIRQCount;
                else if(MAIN_TSRREG.w.bx == 11)
                    value = MAIN_IsrMax;
                else if(MAIN_TSRREG.w.bx == 12)
                    value = MAIN_IsrCount ? (uint32_t)(MAIN_IsrSum / MAIN_IsrCount) : 0;
                else if(MAIN_TSRREG.w.bx == 13)
                    value = MAIN_IsrCount;
                else if(MAIN_TSRREG.w.bx == 24)
                    value = SBEMU_GetUnknownCommands();
#if SBEMU_VMPU
                else if(MAIN_TSRREG.w.bx >= 14 && MAIN_TSRREG.w.bx <= 22)
                {
                    if(!VMPU_Telemetry(MAIN_TSRREG.w.bx - 14, &value))
                    {
                        MAIN_TSRREG.w.flags |= CPU_CFLAG;
                        return;
                    }
                }
#endif
                else if(!GEODE_telemetry_read(&aui, MAIN_TSRREG.w.bx, &value))
                {
                    MAIN_TSRREG.w.flags |= CPU_CFLAG;
                    return;
                }
                MAIN_TSRREG.d.eax = value;
                MAIN_TSRREG.d.edx = 0x47454f31UL; // "GEO1" capability tag
                MAIN_TSRREG.w.flags &= ~CPU_CFLAG;
                return;
            }
#endif
            MAIN_TSRREG.w.flags |= CPU_CFLAG;
            return;
        }
        case 0x02: //set
        {
            if(MAIN_TSRREG.d.ecx != sizeof(MAIN_Options)) //different version of SBEMU?
            {
                MAIN_TSRREG.w.flags |= CPU_CFLAG;
                return;
            }
            MAIN_TSRREG.w.flags &= ~CPU_CFLAG;

            struct MAIN_OPT* _opt = (struct MAIN_OPT*)alloca(sizeof(MAIN_Options));
            DPMI_LMemcpy(DPMI_PTR2L(_opt), MAIN_TSRREG.d.ebx, sizeof(MAIN_Options));
#ifdef __DJ2__
            if(_opt[OPT_XMSLOW].value != MAIN_Options[OPT_XMSLOW].value)
            {
                MAIN_TSRREG.w.flags |= CPU_CFLAG;
                return;
            }
#endif

            for(int i = 0; i < countof(MAIN_Options); ++i)
            {
                if(!(MAIN_Options[i].setcmd&MAIN_SETCMD_STR))
                {
                    MAIN_Options[i].setcmd |= (MAIN_Options[i].value != _opt[i].value) ? MAIN_SETCMD_CHGD : 0;
                    swapi(MAIN_Options[i].value, _opt[i].value);
                    continue;
                }

                if((MAIN_Options[i].setcmd&MAIN_SETCMD_STRCS) && DPMI_LStrcmp(_opt[i].value, DPMI_PTR2L((char*)MAIN_Options[i].value)) == 0)
                    continue;
                if(!(MAIN_Options[i].setcmd&MAIN_SETCMD_STRCS) && DPMI_LStricmp(_opt[i].value, DPMI_PTR2L((char*)MAIN_Options[i].value)) == 0)
                    continue;

                uint32_t len = DPMI_LStrlen(_opt[i].value);
                char* str = malloc(len+1);
                DPMI_LMemcpy(DPMI_PTR2L(str), _opt[i].value, len+1);
                if((MAIN_Options[i].setcmd&MAIN_SETCMD_STRCPY))
                    free((void*)MAIN_Options[i].value);
                MAIN_Options[i].value = (uintptr_t)str;
                MAIN_Options[i].setcmd |= MAIN_SETCMD_STRCPY|MAIN_SETCMD_CHGD;
            }

            char* fpustate = (char*)alloca(108);
            #ifdef DJGPP //make vscode happy
            asm("fsave %0\n\t finit":"=m"(*fpustate));
            #endif
            int irq = aui.card_irq;
            PIC_MaskIRQ(irq);

            struct MAIN_OPT* old_opt = _opt; //readibility. value swapped.
#if MPU_DEBUG
            mpu_debug = MAIN_Options[OPT_MDBG].value;
#endif
            if(OPT_CHANGED(OPT_DEBUG_OUTPUT))
            {
                update_serial_debug_output();
            }
            if(OPT_CHANGED(OPT_MPUCOMPORT))
            {
                unsigned int oldval = old_opt[OPT_MPUCOMPORT].value;
                update_serial_mpu_output();
                MAIN_Options[OPT_MPUCOMPORT].value = oldval; //TODO: don't know why keep the old val.
            }

            if(OPT_CHANGED(OPT_OUTPUT) || OPT_CHANGED(OPT_RATE) || MAIN_Options[OPT_RESET].value)
            {
                if(OPT_CHANGED(OPT_OUTPUT) || MAIN_Options[OPT_RESET].value)
                {
                    _LOG("Reset\n");
                    AU_close(&aui, &fm_aui, &mpu401_aui);
                    memset(&aui, 0, sizeof(aui));
                    memset(&fm_aui, 0, sizeof(fm_aui));
                    memset(&mpu401_aui, 0, sizeof(mpu401_aui));
                    aui.card_select_config = MAIN_Options[OPT_OUTPUT].value;
                    aui.card_select_index =  MAIN_Options[OPT_SC].value;
                    aui.card_controlbits |= AUINFOS_CARDCNTRLBIT_SILENT; //don't print anything in interrupt
                    AU_init(&aui, &fm_aui, &mpu401_aui);
                    AU_ini_interrupts(&aui);
                    AU_setmixer_init(&aui);
                    AU_setmixer_outs(&aui, MIXER_SETMODE_ABSOLUTE, 100);
                    MAIN_Options[OPT_VOL].setcmd |= MAIN_SETCMD_CHGD;   //mark volume dirty
                }
                _LOG("Change sample rate\n");
                _LOG("FLAGS:%x\n",CPU_FLAGS());

                int samplerate = MAIN_Options[OPT_RATE].value;
                mpxplay_audio_decoder_info_s adi = {NULL, 0, 1, samplerate, SBEMU_CHANNELS, SBEMU_CHANNELS, NULL, SBEMU_BITS, SBEMU_BITS/8, 0};
                AU_setrate(&aui, &adi);
#if SBEMU_VMPU
                VMPU_SetOutputRate(aui.freq_card);
#endif
                if(OPT_CHANGED(OPT_RATE))
                    MAIN_InitOPL(aui.freq_card);
                AU_prestart(&aui); //setsamplerate/reset will do stop
                AU_start(&aui);
            }
            if(OPT_CHANGED(OPT_VIRQ))
                VIRQ_SetRealModeVector(MAIN_Options[OPT_VIRQ].value);
#if SBEMU_VMPU
            if(OPT_CHANGED(OPT_MSHARE))
                VMPU_SetIsrShare(MAIN_Options[OPT_MSHARE].value);
#endif
            if(OPT_CHANGED(OPT_VOL))
            {
                _LOG("Reset volume\n");
                AU_setmixer_one(&aui, AU_MIXCHAN_MASTER, MIXER_SETMODE_ABSOLUTE, MAIN_Options[OPT_VOL].value);
            }
            #ifdef DJGPP //make vscode happy
            asm("frstor %0" ::"m"(*fpustate));
            #endif
            PIC_UnmaskIRQ(irq);

            if(OPT_CHANGED(OPT_DMA))
            {
                _LOG("Change DMA\n");
                VDMA_Virtualize(old_opt[OPT_DMA].value, FALSE);
                VDMA_Virtualize(MAIN_Options[OPT_DMA].value, TRUE);
            }
            if(OPT_CHANGED(OPT_HDMA))
            {
                _LOG("Change HDMA\n");
                VDMA_Virtualize(old_opt[OPT_HDMA].value, FALSE);
                VDMA_Virtualize(MAIN_Options[OPT_HDMA].value, TRUE);
            }
            if(OPT_CHANGED(OPT_DMA) || OPT_CHANGED(OPT_HDMA) || OPT_CHANGED(OPT_IRQ) || OPT_CHANGED(OPT_TYPE) || OPT_CHANGED(OPT_FIX_TC))
            {
                _LOG("Reinit SBEMU\n");
                HDPMIPT_LockIRQRouting(FALSE);
                IRQGUARD_Install(MAIN_Options[OPT_IRQ].value);
                HDPMIPT_LockIRQRouting(TRUE);
                SBEMU_Init(
                    MAIN_Options[OPT_IRQ].value,
                    MAIN_Options[OPT_DMA].value,
                    MAIN_Options[OPT_HDMA].value,
                    MAIN_SB_DSPVersion[MAIN_Options[OPT_TYPE].value],
                    MAIN_Options[OPT_FIX_TC].value,
                    &MAIN_SbemuExtFun);
            }

            do
            {
                //Geode: both conditions lacked their '!': any change on the fly re-installed every trap and
                //re-created the synthesizer with the whole soundfont inside this call
                if(!OPT_CHANGED(OPT_OPL) && !OPT_CHANGED(OPT_ADDR) && !OPT_CHANGED(OPT_PM) && !OPT_CHANGED(OPT_RM) && !OPT_CHANGED(OPT_FIX_TC) &&
                !OPT_CHANGED(OPT_MPUADDR) && !OPT_CHANGED(OPT_MPUCOMPORT)
    #if SBEMU_VMPU
                    && !OPT_CHANGED(OPT_VMPU_VOICES) && (!VMPU_ENABLED || !OPT_CHANGED(OPT_VMPU_SF))
    #endif
                )
                {
                    break;
                }

                //re-install all
                if(MAIN_Options[OPT_RM].value)
                {
                    _LOG("uninstall qemm\n");
                    if(MAIN_Options[OPT_OPL].value && OPLRMInstalled) QEMM_Uninstall_IOPortTrap(&OPL3IOPT);
                    if(MAIN_Options[OPT_MPUADDR].value && MAIN_Options[OPT_MPUCOMPORT].value) QEMM_Uninstall_IOPortTrap(&MPUIOPT);
                    QEMM_Uninstall_IOPortTrap(&MAIN_VDMA_IOPT);
                    #if !MAIN_TRAP_RMPIC_ONDEMAND
                    QEMM_Uninstall_IOPortTrap(&MAIN_VIRQ_IOPT);
                    #endif
                    QEMM_Uninstall_IOPortTrap(&MAIN_SB_IOPT);
                }
                if(MAIN_Options[OPT_PM].value)
                {
                    _LOG("uninstall hdpmi\n");
                    if(MAIN_Options[OPT_OPL].value && OPLPMInstalled) HDPMIPT_Uninstall_IOPortTrap(&OPL3IOPT_PM);
                    if(MAIN_Options[OPT_MPUADDR].value && MAIN_Options[OPT_MPUCOMPORT].value) HDPMIPT_Uninstall_IOPortTrap(&MPUIOPT_PM);
                    HDPMIPT_Uninstall_IOPortTrap(&MAIN_VDMA_IOPT_PM1);
                    HDPMIPT_Uninstall_IOPortTrap(&MAIN_VDMA_IOPT_PM2);
                    HDPMIPT_Uninstall_IOPortTrap(&MAIN_VDMA_IOPT_PM3);
                    HDPMIPT_Uninstall_IOPortTrap(&MAIN_VHDMA_IOPT_PM1);
                    HDPMIPT_Uninstall_IOPortTrap(&MAIN_VHDMA_IOPT_PM2);
                    HDPMIPT_Uninstall_IOPortTrap(&MAIN_VHDMA_IOPT_PM3);
                    #if !MAIN_TRAP_PMPIC_ONDEMAND
                    HDPMIPT_Uninstall_IOPortTrap(&MAIN_VIRQ_IOPT_PM1);
                    HDPMIPT_Uninstall_IOPortTrap(&MAIN_VIRQ_IOPT_PM2);
                    #endif
                    HDPMIPT_Uninstall_IOPortTrap(&MAIN_SB_IOPT_PM);
                }
                MAIN_Options[OPT_PM].value = MAIN_Options[OPT_PM].value && MAIN_HDPMI_Present;
                MAIN_Options[OPT_RM].value = MAIN_Options[OPT_RM].value && MAIN_QEMM_Present;

                if(MAIN_Options[OPT_OPL].value && !(fm_aui.fm && fm_aui.fm_port == 0x388))
                {
                    _LOG("install opl\n");
                    QEMM_IODT *iodt = fm_aui.fm ? MAIN_HW_OPL3IODT : MAIN_OPL3IODT;
                    if(MAIN_Options[OPT_RM].value) QEMM_Install_IOPortTrap(iodt, 4, &OPL3IOPT);
                    if(MAIN_Options[OPT_PM].value) HDPMIPT_Install_IOPortTrap(0x388, 0x38B, iodt, 4, &OPL3IOPT_PM);
                }

                if(MAIN_Options[OPT_MPUADDR].value && (MAIN_Options[OPT_MPUCOMPORT].value || VMPU_ENABLED) )
                {
                    _LOG("install mpu\n");
#if SBEMU_VMPU
                    if(VMPU_ENABLED) //NOTE: OPT_VMPU_SF maybe not changed when changed previously but not enabled
                    {
                        #if MAIN_VMPU_HDPMI_MEMFIX
                        MAIN_VMPU_FREEMEM();
                        #endif

                        uint32_t size = DPMI_LoadD(MAIN_Options[OPT_VMPU_SFBUF].value);
                        if(!VMPU_Reset(MAIN_Options[OPT_MPUADDR].value, &MAIN_Options[OPT_VMPU_VOICES].value, aui.freq_card, 
                            (const char*)MAIN_Options[OPT_VMPU_SF].value, MAIN_Options[OPT_VMPU_SFBUF].value+sizeof(size), size) )
                        {
                            MAIN_CPrintf(YELLOW, "VMPU warning: setup failed with SF: %s, functions disabled.\n", (const char*)MAIN_Options[OPT_VMPU_SF].value);
                            #if MAIN_VMPU_HDPMI_MEMFIX
                            MAIN_VMPU_ALLOCMEM();
                            #endif
                        }
                        MAIN_Options[OPT_VMPU_SFBUF].value = 0;
                        //always set
                        MAIN_MPUIODT[0].handler = &VMPU_MPU; //330
                        MAIN_MPUIODT[1].handler = &VMPU_MPU; //331
                    }
                    else
                    {
                        MAIN_MPUIODT[0].handler = &MAIN_MPU_330; //330
                        MAIN_MPUIODT[1].handler = &MAIN_MPU_331; //331
                    }
#endif
                    for(int i = 0; i < countof(MAIN_MPUIODT); ++i) MAIN_MPUIODT[i].port = MAIN_Options[OPT_MPUADDR].value+i;
                    if(MAIN_Options[OPT_RM].value) QEMM_Install_IOPortTrap(MAIN_MPUIODT, 2, &MPUIOPT);
                    if(MAIN_Options[OPT_PM].value) HDPMIPT_Install_IOPortTrap(MAIN_Options[OPT_MPUADDR].value, MAIN_Options[OPT_MPUADDR].value+1, MAIN_MPUIODT, 2, &MPUIOPT_PM);
                }

                QEMM_IODT* SB_Iodt = MAIN_Options[OPT_OPL].value ? MAIN_SB_IODT : MAIN_SB_IODT+4;
                int SB_IodtCount = MAIN_Options[OPT_OPL].value ? countof(MAIN_SB_IODT) : countof(MAIN_SB_IODT)-4;
                if(OPT_CHANGED(OPT_ADDR))
                {
                    for(int i = 0; i < countof(MAIN_SB_IODT); ++i)
                        MAIN_SB_IODT[i].port = MAIN_SB_IODT[i].port - old_opt[OPT_ADDR].value + MAIN_Options[OPT_ADDR].value;
                }

                if(MAIN_Options[OPT_RM].value)
                {
                    _LOG("install qemm\n");
                    QEMM_Install_IOPortTrap(MAIN_VDMA_IODT, countof(MAIN_VDMA_IODT), &MAIN_VDMA_IOPT);
                    QEMM_Install_IOPortTrap(SB_Iodt, SB_IodtCount, &MAIN_SB_IOPT);
                    #if !MAIN_TRAP_RMPIC_ONDEMAND
                    QEMM_Install_IOPortTrap(MAIN_VIRQ_IODT, countof(MAIN_VIRQ_IODT), &MAIN_VIRQ_IOPT);
                    #endif
                }

                if(MAIN_Options[OPT_PM].value)
                {
                    _LOG("install hdpmi\n");
                    HDPMIPT_Install_IOPortTrap(0x0, 0xF, MAIN_VDMA_IODT, 16, &MAIN_VDMA_IOPT_PM1);
                    HDPMIPT_Install_IOPortTrap(0x81, 0x83, MAIN_VDMA_IODT+16, 3, &MAIN_VDMA_IOPT_PM2);
                    HDPMIPT_Install_IOPortTrap(0x87, 0x87, MAIN_VDMA_IODT+19, 1, &MAIN_VDMA_IOPT_PM3);
                    HDPMIPT_Install_IOPortTrap(0xC0, 0xDE, MAIN_VDMA_IODT+20, 16, &MAIN_VHDMA_IOPT_PM1);
                    HDPMIPT_Install_IOPortTrap(0x89, 0x8B, MAIN_VDMA_IODT+36, 3, &MAIN_VHDMA_IOPT_PM2);
                    HDPMIPT_Install_IOPortTrap(0x8F, 0x8F, MAIN_VDMA_IODT+39, 1, &MAIN_VHDMA_IOPT_PM3);
                    #if !MAIN_TRAP_PMPIC_ONDEMAND
                    HDPMIPT_Install_IOPortTrap(0x20, 0x21, MAIN_VIRQ_IODT, 2, &MAIN_VIRQ_IOPT_PM1);
                    HDPMIPT_Install_IOPortTrap(0xA0, 0xA1, MAIN_VIRQ_IODT+2, 2, &MAIN_VIRQ_IOPT_PM2);
                    #endif
                    HDPMIPT_Install_IOPortTrap(MAIN_Options[OPT_ADDR].value, MAIN_Options[OPT_ADDR].value+0x0F, SB_Iodt, SB_IodtCount, &MAIN_SB_IOPT_PM);
                }

                if(MAIN_Options[OPT_RM].value)
                {
                    UntrappedIO_OUT_Handler = &QEMM_UntrappedIO_Write;
                    UntrappedIO_IN_Handler = &QEMM_UntrappedIO_Read;
                }
                else
                {
                    UntrappedIO_OUT_Handler = &HDPMIPT_UntrappedIO_Write;
                    UntrappedIO_IN_Handler = &HDPMIPT_UntrappedIO_Read;
                }
            } while(0);

            //cleanup changed states for all
            for(int i = 0; i < countof(MAIN_Options); ++i)
                MAIN_Options[i].setcmd &= ~MAIN_SETCMD_CHGD;
        }
        return;
    }
}
