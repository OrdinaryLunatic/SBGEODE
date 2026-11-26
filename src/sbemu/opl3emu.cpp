#include "opl3emu.h"
#include "dbopl.h"

#define OPL3EMU_PRIMARY 0
#define OPL3EMU_SECONDARY 1
//primary index read
#define OPL3EMU_TIMER_REG_INDEX 4
#define OPL3EMU_TIMER1_FLAG 0x40
#define OPL3EMU_TIMER2_FLAG 0x20
#define OPL3EMU_TIMER_IRQ 0x80
static uint32_t OPL3EMU_IndexReg[2];

#ifndef OPL3EMU_TSC
static uint64_t OPL3EMU_TSC(void)
{
    uint32_t lo, hi;
    __asm__ __volatile__("rdtsc" : "=a"(lo), "=d"(hi));
    return ((uint64_t)hi << 32) | lo;
}
#endif

static uint32_t OPL3EMU_TimerMHz;
static uint8_t OPL3EMU_TimerValue[2];
static uint8_t OPL3EMU_TimerRunning;
static uint8_t OPL3EMU_TimerMask;
static uint8_t OPL3EMU_TimerStatus;
static uint32_t OPL3EMU_TimerPeriod[2];
static uint64_t OPL3EMU_TimerDeadline[2];

static uint32_t OPL3EMU_TimerCycles(int timer)
{
    return OPL3EMU_TimerMHz * (256 - OPL3EMU_TimerValue[timer]) * (timer ? 320 : 80);
}

static void OPL3EMU_TimerSyncIRQ(void)
{
    OPL3EMU_TimerStatus = (OPL3EMU_TimerStatus & 0x60) |
        ((OPL3EMU_TimerStatus & 0x60) ? OPL3EMU_TIMER_IRQ : 0);
}

static void OPL3EMU_TimerUpdate(void)
{
    const uint64_t now = OPL3EMU_TSC();
    for(int i = 0; i < 2; ++i)
    {
        if(!(OPL3EMU_TimerRunning & (1 << i)))
            continue;
        if(now < OPL3EMU_TimerDeadline[i])
            continue;
        const uint64_t elapsed = now - OPL3EMU_TimerDeadline[i];
        const uint8_t flag = i ? OPL3EMU_TIMER2_FLAG : OPL3EMU_TIMER1_FLAG;
        if(!(OPL3EMU_TimerMask & flag))
            OPL3EMU_TimerStatus |= flag;
        OPL3EMU_TimerDeadline[i] = now + OPL3EMU_TimerPeriod[i] - elapsed % OPL3EMU_TimerPeriod[i];
    }
    OPL3EMU_TimerSyncIRQ();
}

//secondary index read (Adlib Gold). reference: AIL2.0 source code, dosbox
#define OPL3EMU_ADLG_IOBUSY 0x40UL
#define OPL3EMU_ADLG_VOLL_REG_INDEX 9 //left volume
#define OPL3EMU_ADLG_VOLR_REG_INDEX 10 //right volume
//data
#define KEY_ON 0x10 //channel on bit
static uint32_t OPL3EMU_ADLG_CtrlEnable = 0;    //seems not working for Miles Sound, don't use it
static uint32_t OPL3EMU_ADLG_Volume[2] = {0x08,0x08};

static DBOPL::Chip* OPL3EMU_Chip;

/* Geode: optional half-rate rendering. The chip runs at samplerate/2 and the
   output is doubled by linear interpolation (y[2j] = (h[j-1]+h[j])/2,
   y[2j+1] = h[j]); an odd request leaves one output frame for the next call. */
#define OPL3EMU_HALF_CHUNK 512
static int OPL3EMU_Half;
static int OPL3EMU_Chans;      //channel count the state below belongs to
static int OPL3EMU_HasCarry;
static int16_t OPL3EMU_Prev[2], OPL3EMU_Carry[2];
static int16_t OPL3EMU_Tmp[OPL3EMU_HALF_CHUNK*2];

void OPL3EMU_Init(int samplerate, int halfrate, int cpu_mhz)
{
    if(OPL3EMU_Chip)
        delete OPL3EMU_Chip;
    OPL3EMU_Half = halfrate ? 1 : 0;
    OPL3EMU_Chans = 0;
    OPL3EMU_HasCarry = 0;
    OPL3EMU_TimerMHz = cpu_mhz > 0 ? cpu_mhz : 1;
    OPL3EMU_TimerRunning = 0;
    OPL3EMU_TimerMask = 0;
    OPL3EMU_TimerStatus = 0;
    for(int i = 0; i < 2; ++i)
    {
        OPL3EMU_TimerValue[i] = 0;
        OPL3EMU_TimerPeriod[i] = OPL3EMU_TimerCycles(i);
        OPL3EMU_TimerDeadline[i] = 0;
    }
    OPL3EMU_Chip = new DBOPL::Chip(true);
    OPL3EMU_Chip->Setup(OPL3EMU_Half ? samplerate/2 : samplerate);
}

int OPL3EMU_IsActive()
{
    return (OPL3EMU_Chip->oplActive&(OPL3EMU_Chip->opl3Active?0xFFFFFFFF:0x8000FFFF));
}

int OPL3EMU_GetMode()
{
    return OPL3EMU_Chip->opl3Active;
}

int OPL3EMU_GenSamples(int16_t* pcm16, int count)
{
    if(!OPL3EMU_Half)
        return OPL3EMU_Chip->Generate(pcm16, count);
    const int ch = OPL3EMU_Chip->opl3Active ? 2 : 1; //mono in OPL2 mode, as Generate() delivers it
    if(ch != OPL3EMU_Chans)
    {
        OPL3EMU_Chans = ch;
        OPL3EMU_HasCarry = 0;
        OPL3EMU_Prev[0] = OPL3EMU_Prev[1] = 0;
    }
    int done = 0;
    while(done < count)
    {
        if(OPL3EMU_HasCarry)
        {
            for(int c = 0; c < ch; ++c)
                pcm16[done*ch+c] = OPL3EMU_Carry[c];
            OPL3EMU_HasCarry = 0;
            ++done;
            continue;
        }
        int half = (count - done + 1) / 2;
        if(half > OPL3EMU_HALF_CHUNK) half = OPL3EMU_HALF_CHUNK;
        OPL3EMU_Chip->Generate(OPL3EMU_Tmp, half);
        for(int j = 0; j < half; ++j)
        {
            for(int k = 0; k < 2; ++k)
            {
                int16_t y[2];
                for(int c = 0; c < ch; ++c)
                {
                    const int16_t h = OPL3EMU_Tmp[j*ch+c];
                    y[c] = k ? h : (int16_t)(((int)OPL3EMU_Prev[c] + h) >> 1);
                    if(k) OPL3EMU_Prev[c] = h;
                }
                if(done < count)
                {
                    for(int c = 0; c < ch; ++c)
                        pcm16[done*ch+c] = y[c];
                    ++done;
                }
                else
                {
                    for(int c = 0; c < ch; ++c)
                        OPL3EMU_Carry[c] = y[c];
                    OPL3EMU_HasCarry = 1;
                }
            }
        }
    }
    return count;
}

uint32_t OPL3EMU_PrimaryRead(uint32_t val)
{
    OPL3EMU_TimerUpdate();
    return (val & ~0xFF) | OPL3EMU_TimerStatus;
}

uint32_t OPL3EMU_PrimaryWriteIndex(uint32_t val)
{
    OPL3EMU_IndexReg[OPL3EMU_PRIMARY] = OPL3EMU_Chip->WriteAddr(0x388, val);
    return val;
}

uint32_t OPL3EMU_PrimaryWriteData(uint32_t val)
{
    const uint32_t reg = OPL3EMU_IndexReg[OPL3EMU_PRIMARY];
    if(reg == 2 || reg == 3)
    {
        const int timer = reg - 2;
        OPL3EMU_TimerValue[timer] = val;
        OPL3EMU_TimerPeriod[timer] = OPL3EMU_TimerCycles(timer);
    }
    else if(reg == OPL3EMU_TIMER_REG_INDEX)
    {
        OPL3EMU_TimerUpdate();
        if(val & 0x80)
            OPL3EMU_TimerStatus = 0;
        else
        {
            const uint64_t now = OPL3EMU_TSC();
            OPL3EMU_TimerStatus &= ~(val & 0x60);
            OPL3EMU_TimerMask = val & 0x60;
            for(int i = 0; i < 2; ++i)
            {
                const uint8_t start = (val >> i) & 1;
                const uint8_t was_running = OPL3EMU_TimerRunning & (1 << i);
                if(start && !was_running)
                    OPL3EMU_TimerDeadline[i] = now + OPL3EMU_TimerPeriod[i];
                if(start)
                    OPL3EMU_TimerRunning |= 1 << i;
                else
                    OPL3EMU_TimerRunning &= ~(1 << i);
            }
        }
        OPL3EMU_TimerSyncIRQ();
    }
    OPL3EMU_Chip->WriteReg(reg, val);
    return val;
}

uint32_t OPL3EMU_SecondaryRead(uint32_t val)
{
    if(OPL3EMU_IndexReg[OPL3EMU_SECONDARY] == 0x100+OPL3EMU_ADLG_VOLL_REG_INDEX || OPL3EMU_IndexReg[OPL3EMU_SECONDARY] == 0x100+OPL3EMU_ADLG_VOLR_REG_INDEX)
            return OPL3EMU_ADLG_Volume[OPL3EMU_IndexReg[OPL3EMU_SECONDARY]-0x100-OPL3EMU_ADLG_VOLL_REG_INDEX];
    return OPL3EMU_PrimaryRead(val);
}

uint32_t OPL3EMU_SecondaryWriteIndex(uint32_t val)
{
    if(val == 0xFF)
        OPL3EMU_ADLG_CtrlEnable = 1;
    else if(val == 0xFE)
        OPL3EMU_ADLG_CtrlEnable = 0;

    OPL3EMU_IndexReg[OPL3EMU_SECONDARY] = OPL3EMU_Chip->WriteAddr(0x38A, val);
    return val;
}

uint32_t OPL3EMU_SecondaryWriteData(uint32_t val)
{
    if(/*OPL3EMU_ADLG_CtrlEnable && */(OPL3EMU_IndexReg[OPL3EMU_SECONDARY] == 0x100+OPL3EMU_ADLG_VOLL_REG_INDEX || OPL3EMU_IndexReg[OPL3EMU_SECONDARY] == 0x100+OPL3EMU_ADLG_VOLR_REG_INDEX))
        OPL3EMU_ADLG_Volume[OPL3EMU_IndexReg[OPL3EMU_SECONDARY]-0x100-OPL3EMU_ADLG_VOLL_REG_INDEX] = val;
    OPL3EMU_Chip->WriteReg(OPL3EMU_IndexReg[OPL3EMU_SECONDARY], val);
    return val;
}
