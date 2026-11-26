
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <dos.h>
#include <string.h>
#include <stdlib.h>

#include <dpmi/dbgutil.h>
#include "vmpu.h"

#if SBEMU_VMPU

#define TSF_IMPLEMENTATION
#define TSF_FLOAT_LUT
#define TSF_INT_MIX /* Geode: voices mix in 32-bit integers (tsf.h) */
#include "tsf.h"

static unsigned char fpu_buffer[512] __attribute__((aligned(16)));

/* Geode: ISR time budget. A 500 MHz LX cannot render 24+ voices and start many
   notes inside one 6 ms buffer period, and an overrun stalls the game. The ISR
   passes its start time and the measured period (TSC cycles) through
   VMPU_BeginPeriod(). MIDI messages are then processed only while within
   VMPU_MSG_SHARE of the period (the rest stay queued for the next period, at
   least one is always handled), and the voice count is limited so that
   rendering finishes within VMPU_ISR_SHARE of the period (voice stealing). */
#define VMPU_DEF_PERIOD   3000000UL /* until the period has been measured */
#define VMPU_MSG_SHARE    4         /* x/16 of the period for message handling */
#define VMPU_ISR_SHARE    10        /* x/16 of the period for the whole VMPU part */
#define VMPU_MIN_VOICES   4         /* never steal below this */
#define VMPU_COST_INIT    500       /* cycles per voice and rendered frame */
#define VMPU_BACKLOG_PANIC 2048     /* queued bytes: lift the message budget */

static inline uint32_t vmpu_tsc(void)
{
    uint32_t lo, hi;
    __asm__ __volatile__("rdtsc" : "=a"(lo), "=d"(hi));
    return lo;
}

static uint32_t vmpu_t0;                       /* ISR start */
static uint32_t vmpu_period = VMPU_DEF_PERIOD; /* cycles per buffer period */
static uint32_t vmpu_cost = VMPU_COST_INIT;    /* peak-held cost estimate */
static int vmpu_isr_share = VMPU_ISR_SHARE;    /* /MSHARE: x/16 of the period for the VMPU part */
static uint32_t vmpu_msg_cost;                 /* message cycles in this ISR */
static BOOL vmpu_ready;                        /* VMPU_BeginPeriod() seen */

/* telemetry (read through VMPU_Telemetry) */
static uint32_t vmpu_t_msgmax, vmpu_t_notemax, vmpu_t_rendermax;
static uint32_t vmpu_t_backlog, vmpu_t_stolen, vmpu_t_voicesmax, vmpu_t_deferred;

/* 0x330: data port
 * 0x331: read: status port
 *       write: command port
 * status port:
 * bit 6: 0=ready to write cmd or MIDI data; 1=interface busy
 * bit 7: 0=data ready to read; 1=no data at data port
 * command port:
 *  0xff: reset - triggers ACK (FE) to be read from data port
 *  0x3f: set to UART mode - triggers ACK (FE) to be read from data port
 */

/* Geode: the synth renders at half rate (see VMPU_GenSamples). Voices bind the
   output rate when they start, so the rate must already be right when the
   first note-on is processed (before any GenSamples call). */
#ifndef SBEMU_VMPU_HALFRATE
#define SBEMU_VMPU_HALFRATE 1
#endif
#if SBEMU_VMPU_HALFRATE
#define VMPU_RENDER_RATE(f) ((f)/2)
#else
#define VMPU_RENDER_RATE(f) (f)
#endif

/* Input realtime F9 is ignored. Internally it is one ordered Stop operation.
 * The existing writer limit 4092 leaves room for this token even after a
 * synthesized running-status byte. No synth work is done in the port trap. */
static bool bReset = false;
#define VMPU_STOP_MARKER 0xF9
static uint8_t bUART = 0;

static const int midi_lengths[8] = {3, 3, 3, 3, 2, 2, 3, 1};
static unsigned char midi_buffer[4096];
static unsigned int midi_ptr = 0;
static unsigned int midi_available_ptr = 0;
static unsigned int midi_message_cntr = 0;
static bool midi_in_sysex = false;
static unsigned char midi_running = 0; //running status (0 = none)
static unsigned char midi_mpu_status = 0x80;
static const unsigned char gm_reset[6] = {0xF0, 0x7E, 0x7F, 0x09, 0x01, 0xF7};
static const unsigned char gs_reset[11] = {0xF0, 0x41, 0x10, 0x42, 0x12, 0x40, 0x00, 0x7F, 0x00, 0x41, 0xF7};

static char loaded_sf2[512];
static tsf *tsfrenderer;
static int VMPU_base;

static void VMPU_Process_Messages(BOOL budgeted)
{
    unsigned char *temp_buffer = midi_buffer;
    unsigned int index = 0;
    uint32_t tstart = 0, tmsg;
    BOOL first = TRUE;
    uint32_t limit = vmpu_period / 16 * VMPU_MSG_SHARE;
    if(!tsfrenderer)
    {
        midi_ptr = 0;
        midi_available_ptr = 0;
        midi_message_cntr = 0;
        midi_running = 0;
        midi_in_sysex = false;
        return;
    }
    if(midi_available_ptr > VMPU_BACKLOG_PANIC) //game is outrunning the synth
        limit *= 4;

    if(index < midi_available_ptr)
    {
        #if _LOG_ENABLE
        DBG_Logi("[");
        for(int i = 0; i < midi_available_ptr; ++i)
            DBG_Logi("%x ", temp_buffer[i]);
        DBG_Logi("]");
        #endif
    }
    else
        return;

    if(midi_available_ptr > vmpu_t_backlog)
        vmpu_t_backlog = midi_available_ptr;
    tstart = vmpu_tsc();
    while (index < midi_available_ptr)
    {
        if(budgeted && vmpu_ready && !first && (vmpu_tsc() - vmpu_t0 > limit))
        {
            ++vmpu_t_deferred; //rest of the queue is handled in the next period
            break;
        }
        first = FALSE;
        tmsg = vmpu_tsc();
        _LOG("MIDI:%x, i=%d, a=%d\n", *temp_buffer, index, midi_available_ptr);
        switch (*temp_buffer & 0xF0)
        {
        case 0xD0:
            tsf_channel_set_pressure(tsfrenderer, temp_buffer[0] & 0xf, temp_buffer[1] / 127.f);
        case 0xA0:
        {
            index += midi_lengths[(temp_buffer[0] >> 4) - 0x8];
            temp_buffer += midi_lengths[(temp_buffer[0] >> 4) - 0x8];
            break;
        }
        case 0x80:
            tsf_channel_note_off(tsfrenderer, temp_buffer[0] & 0xf, temp_buffer[1]);
            index += midi_lengths[(temp_buffer[0] >> 4) - 0x8];
            temp_buffer += midi_lengths[(temp_buffer[0] >> 4) - 0x8];
            break;
        case 0x90:
            tsf_channel_note_on(tsfrenderer, temp_buffer[0] & 0xf, temp_buffer[1], temp_buffer[2] / 127.0f);
            index += midi_lengths[(temp_buffer[0] >> 4) - 0x8];
            temp_buffer += midi_lengths[(temp_buffer[0] >> 4) - 0x8];
            break;
        case 0xE0:
            tsf_channel_set_pitchwheel(tsfrenderer, temp_buffer[0] & 0xf, (temp_buffer[1] & 0x7f) | ((temp_buffer[2] & 0x7f) << 7));
            index += midi_lengths[(temp_buffer[0] >> 4) - 0x8];
            temp_buffer += midi_lengths[(temp_buffer[0] >> 4) - 0x8];
            break;
        case 0xC0:
            tsf_channel_set_presetnumber(tsfrenderer, temp_buffer[0] & 0xf, temp_buffer[1], (temp_buffer[0] & 0xf) == 0x9);
            index += midi_lengths[(temp_buffer[0] >> 4) - 0x8];
            temp_buffer += midi_lengths[(temp_buffer[0] >> 4) - 0x8];
            break;
        case 0xB0:
            tsf_channel_midi_control(tsfrenderer, temp_buffer[0] & 0xf, temp_buffer[1], temp_buffer[2]);
            index += midi_lengths[(temp_buffer[0] >> 4) - 0x8];
            temp_buffer += midi_lengths[(temp_buffer[0] >> 4) - 0x8];
            break;
        case 0xF0:
        {
            if (*temp_buffer == VMPU_STOP_MARKER)
            {
                for (unsigned ch = 0; ch < 16; ++ch)
                    tsf_channel_midi_control(tsfrenderer, ch, 121, 0);
            }
            if (*temp_buffer == 0xFF)
            {
                {
                    tsf_reset(tsfrenderer);
                    tsf_channel_set_bank_preset(tsfrenderer, 9, 128, 0);
                    tsf_set_volume(tsfrenderer, 1.0f);
                }
            }
            if (*temp_buffer == 0xF0)
            {
                unsigned char *sysexbuf = temp_buffer;
                unsigned int sysexlen = 0;
                while (index < midi_available_ptr && *temp_buffer != 0xF7) //never scan past the queue
                {
                    index++;
                    temp_buffer += 1;
                    sysexlen++;
                }

                index++;
                temp_buffer += 1;
                sysexlen++;

                //						_dprintf("SYSEX MESSAGE: ");
                //						for (i = 0; i < sysexlen; i++) {
                //							_dprintf("0x%02X ", sysexbuf[i]);
                //						}
                //						_dprintf("\n");

                /* Only the single-data-byte GS DT1 volume command is supported.
                 * Address, data and checksum must all be 7-bit, with EOX last. */
                if (sysexlen == 11 && sysexbuf[1] == 0x41 && sysexbuf[2] < 0x80 &&
                    sysexbuf[3] == 0x42 && sysexbuf[4] == 0x12 && sysexbuf[10] == 0xF7)
                {
                    unsigned checksum = 0;
                    bool valid = true;
                    for (unsigned i = 5; i <= 9; ++i)
                    {
                        valid = valid && sysexbuf[i] < 0x80;
                        checksum += sysexbuf[i];
                    }
                    if (valid && !(checksum & 0x7F) && sysexbuf[5] == 0x40 &&
                        sysexbuf[6] == 0 && sysexbuf[7] == 4)
                        tsf_set_volume(tsfrenderer, sysexbuf[8] / 127.f);
                }
                if (tsfrenderer && sysexlen >= 8 && sysexbuf[1] == 0x7f && sysexbuf[2] == 0x7f &&
                    sysexbuf[3] == 0x04 && sysexbuf[4] == 0x01)
                {
                    //                                                        _dprintf("GM Master Vol 0x%02X\n", sysexbuf[6]);
                    tsf_set_volume(tsfrenderer, sysexbuf[6] / 127.f);
                }
                // TODO: Differentiate between GS and GM Resets.
                if ((sysexlen == sizeof(gs_reset) && !memcmp(sysexbuf, gs_reset, sizeof(gs_reset))) ||
                    (sysexlen == sizeof(gm_reset) && !memcmp(sysexbuf, gm_reset, sizeof(gm_reset))))
                {
                    tsf_reset(tsfrenderer);
                    tsf_channel_set_bank_preset(tsfrenderer, 9, 128, 0);
                    tsf_set_volume(tsfrenderer, 1.0f);
                }
            }
            else
            {
                index += midi_lengths[(temp_buffer[0] >> 4) - 0x8];
                temp_buffer += midi_lengths[(temp_buffer[0] >> 4) - 0x8];
            }
            break;
        }
        default: //stray data byte (no status): drop it, never spin in the ISR
            ++index;
            ++temp_buffer;
            break;
        }
        tmsg = vmpu_tsc() - tmsg;
        if(tmsg > vmpu_t_notemax) vmpu_t_notemax = tmsg;
    }
    //safe clamp, not needed, but just in case
    if(index > midi_available_ptr) index = midi_available_ptr;

    memmove(midi_buffer, midi_buffer + index, midi_ptr - index);
    midi_ptr -= index;
    midi_available_ptr -= index;
    midi_message_cntr = midi_ptr - midi_available_ptr; //bytes of the incomplete message
    tstart = vmpu_tsc() - tstart;
    vmpu_msg_cost += tstart;
    if(vmpu_msg_cost > vmpu_t_msgmax) vmpu_t_msgmax = vmpu_msg_cost;
}

static void VMPU_Write(uint16_t port, uint8_t value)
////////////////////////////////////////////////////
{
    //if(VMPU_base == port)
    {
        _LOG("VMPU_Write(%X)=%X\n", port, value);
    }
    if (port == VMPU_base+1)
    {
        if (value == 0x3f)
        {
            midi_mpu_status &= ~0x80;
        }
        if (value == 0xff)
        {
            fpu_save(fpu_buffer);
            VMPU_Process_Messages(FALSE);
            fpu_restore(fpu_buffer);
            bReset = true;
            midi_ptr = 0;
            midi_available_ptr = 0;
            midi_message_cntr = 0;
            midi_running = 0;
            midi_in_sysex = false;
            midi_mpu_status &= ~0x80;
        }
        if (value == 0x05)
        {
            /* Drop only the incomplete tail. The queued reset stays behind all
             * complete earlier messages and ahead of subsequent MIDI data. */
            midi_ptr = midi_available_ptr;
            midi_message_cntr = 0;
            midi_running = 0;
            midi_in_sysex = false;
            /* Consecutive Stops with no intervening event are idempotent. */
            if (!midi_ptr || midi_buffer[midi_ptr - 1] != VMPU_STOP_MARKER)
                midi_buffer[midi_ptr++] = VMPU_STOP_MARKER;
            midi_available_ptr = midi_ptr;
        }
    }
    else
    {
        if (!bReset)
        {
            {
                if (midi_ptr >= 4092)
                {   //Geode: an over-long SysEx must not block the port for good
                    if (midi_in_sysex)
                    {
                        midi_ptr = midi_available_ptr;
                        midi_message_cntr = 0;
                        midi_in_sysex = false;
                    }
                    return;
                }

                //Geode: a status byte other than F7 ends a SysEx that was never
                //closed (the unfinished SysEx is dropped, the byte is handled as
                //usual below); real-time bytes inside a SysEx are ignored
                if (midi_in_sysex && value >= 0xF8)
                    return;
                if (midi_in_sysex && value >= 0x80 && value != 0xF7)
                {
                    midi_ptr = midi_available_ptr;
                    midi_message_cntr = 0;
                    midi_in_sysex = false;
                }

                if (midi_in_sysex)
                {
                    midi_buffer[midi_ptr++] = value;
                    if (value == 0xF7)
                    {
                        midi_available_ptr = midi_ptr;
                        midi_message_cntr = 0;
                        midi_in_sysex = false;
                    }
                    return;
                }

                //Geode: real-time bytes (clock, start/stop, active sensing) carry no
                //synth data and would corrupt a message in progress
                if (value >= 0xF8 && value != 0xFF)
                    return;
                //Geode: running status. A data byte at a message start repeats the
                //last channel status; without one it is stray and dropped (the
                //length lookup below would index with a data byte otherwise).
                if (value < 0x80)
                {
                    if (midi_message_cntr == 0)
                    {
                        if (!midi_running)
                            return;
                        midi_buffer[midi_ptr++] = midi_running;
                        midi_message_cntr++;
                    }
                }
                else if (value < 0xF0)
                    midi_running = value;
                else if (value < 0xF8)
                    midi_running = 0;

                //Handling incomplete messages
                //some game (e.g. PAL) have complete message like (E1, 0), (91,4A,F), where (E1,0) misses one byte data
                //most MIDI softwares discard incomplete messages
                unsigned int data_inserted = 0;
                if ((value & 0xF0) >= 0x80 && midi_message_cntr > 0)
                {
                    unsigned int lastptr = midi_ptr-1;
                    while(lastptr > 0 && (midi_buffer[lastptr] & 0xF0) < 0x80) --lastptr;
                    unsigned int lastcnt = midi_lengths[(midi_buffer[lastptr] >> 4) - 0x8];
                    if((midi_buffer[lastptr] & 0xF0) >= 0x80 && midi_ptr-lastptr < lastcnt)
                    {
                        #if 0 // 0 insertion
                        while(midi_ptr-lastptr < lastcnt)
                        {
                            _LOG("insert: p=%d,%d b=%x s=%d, c=%d\n", lastptr, midi_ptr, midi_buffer[lastptr], midi_lengths[(midi_buffer[lastptr] >> 4) - 0x8], midi_ptr-lastptr);
                            midi_buffer[midi_ptr++] = 0;
                            ++midi_message_cntr;
                            ++data_inserted;
                        }
                        #else // discard
                        midi_ptr = lastptr;
                        midi_message_cntr = 0;
                        #endif
                    }
                }

                midi_buffer[midi_ptr++] = value;
                midi_message_cntr++;

                unsigned int cnt = midi_lengths[(midi_buffer[midi_available_ptr] >> 4) - 0x8];
                if (midi_message_cntr >= cnt)
                {
                    if (value == 0xF0)
                    {
                        midi_message_cntr = 0;
                        midi_in_sysex = true;
                        return;
                    }
                    midi_available_ptr = midi_ptr - data_inserted;
                    //_LOG("a=%d b=%x, m=%d\n", midi_available_ptr, midi_buffer[midi_available_ptr], midi_message_cntr);
                    midi_message_cntr -= cnt;
                }
            }
        }
    }
    return;
}

static uint8_t VMPU_Read(uint16_t port)
///////////////////////////////////////
{
    _LOG("VMPU_Read(%X)\n", port);
    if (port == VMPU_base)
    {
        midi_mpu_status |= 0x80;
        if (bReset)
        {
            bReset = false;
            return 0xfe;
        }
        return 0xfe; // Always return Active Sensing.
    }
    else
    {
        return midi_mpu_status | (midi_ptr >= 4092 ? 0x40 : 0);
    }
}


static int tsf_stream_linear_memory_read(struct tsf_stream_memory* m, void* ptr, unsigned int size) { if (size > m->total - m->pos) size = m->total - m->pos; DPMI_LMemcpy(DPMI_PTR2L(ptr), (uintptr_t)m->buffer+m->pos, size); m->pos += size; return size; }
static int tsf_stream_linear_memory_skip(struct tsf_stream_memory* m, unsigned int count) { if (m->pos + count > m->total) return 0; m->pos += count; return 1; }
static tsf* tsf_load_linear_memory(uint32_t linear_mem, int size)
{
	struct tsf_stream stream = { TSF_NULL, (int(*)(void*,void*,unsigned int))&tsf_stream_linear_memory_read, (int(*)(void*,unsigned int))&tsf_stream_linear_memory_skip };
	struct tsf_stream_memory f = { 0, 0, 0 };
	f.buffer = (const char*)linear_mem;
	f.total = size;
	stream.data = &f;
	return tsf_load(&stream);
}

BOOL VMPU_Init(int baseaddr, int* voices, int freq, const char* sf2)
{
    VMPU_base = 0x330;
    if (*voices < 4) /* Geode: a 500 MHz LX cannot afford 32+ voices */
        *voices = 4;
    if (*voices > 256)
        *voices = 256;
    
    tsfrenderer = tsf_load_filename(sf2);

    if (tsfrenderer)
    {
        int channel = 0;
        tsf_set_max_voices(tsfrenderer, 256); //pre-alloc
        tsf_set_max_voices(tsfrenderer, *voices);
        tsf_set_output(tsfrenderer, TSF_STEREO_INTERLEAVED, VMPU_RENDER_RATE(freq), 0);
        for (channel = 15; channel >= 0; --channel)
            tsf_channel_midi_control(tsfrenderer, channel, 121, 0);
        tsf_channel_set_bank_preset(tsfrenderer, 9, 128, 0);

        VMPU_base = baseaddr;

        memcpy(loaded_sf2, sf2, min(strlen(sf2)+1, sizeof(loaded_sf2)));
        loaded_sf2[sizeof(loaded_sf2)-1] = '\0';
        return TRUE;
    }
    else
    {
        memset(loaded_sf2, 0, sizeof(loaded_sf2));
        return FALSE;
    }
}

BOOL VMPU_Reset(int baseaddr, int* voices, int freq, const char* sf2, uint32_t sf2_linear_mem, int bytes)
{
    VMPU_base = 0x330;
    if (*voices < 4) /* Geode: a 500 MHz LX cannot afford 32+ voices */
        *voices = 4;
    if (*voices > 256)
        *voices = 256;

    if(tsfrenderer && stricmp(loaded_sf2, sf2) != 0)
    {
        tsf_close(tsfrenderer);
        tsfrenderer = NULL;
    }

    if(!tsfrenderer)
        tsfrenderer = tsf_load_linear_memory(sf2_linear_mem, bytes);

    if (tsfrenderer)
    {
        int channel = 0;
        tsf_set_max_voices(tsfrenderer, 256); //pre-alloc
        tsf_set_max_voices(tsfrenderer, *voices);
        tsf_set_output(tsfrenderer, TSF_STEREO_INTERLEAVED, VMPU_RENDER_RATE(freq), 0);
        for (channel = 15; channel >= 0; --channel)
            tsf_channel_midi_control(tsfrenderer, channel, 121, 0);
        tsf_channel_set_bank_preset(tsfrenderer, 9, 128, 0);

        VMPU_base = baseaddr;

        memcpy(loaded_sf2, sf2, min(strlen(sf2)+1, sizeof(loaded_sf2)));
        loaded_sf2[sizeof(loaded_sf2)-1] = '\0';
        return TRUE;
    }
    else
    {
        memset(loaded_sf2, 0, sizeof(loaded_sf2));
        return FALSE;
    }
}

void VMPU_SetOutputRate(int freq)
{
    /* Voices bind the output rate when a note starts: the first note-on is
       processed before any VMPU_GenSamples call, so the rate must be set here. */
    if (tsfrenderer && freq >= 4000)
        tsf_set_samplerate_output(tsfrenderer, VMPU_RENDER_RATE(freq));
}

/* /MSHARE: the longest audio interrupt under MIDI holds every other interrupt (HDPMI32i does not nest
   interrupts), so the share is a trade-off between that hold and the notes cut by the voice limiter. */
void VMPU_SetIsrShare(int sixteenths)
{
    if(sixteenths >= 4 && sixteenths <= 14)
        vmpu_isr_share = sixteenths;
}

void VMPU_BeginPeriod(uint32_t t0, uint32_t period_cycles)
{
    vmpu_t0 = t0;
    vmpu_msg_cost = 0;
    if(period_cycles)
        vmpu_period = period_cycles;
    vmpu_ready = TRUE;
}

BOOL VMPU_IsActive()
{
    if(!tsfrenderer) return FALSE;
    fpu_save(fpu_buffer);
    VMPU_Process_Messages(TRUE);
    fpu_restore(fpu_buffer);
    return !!tsf_active_voice_count(tsfrenderer);
}

/* Voice stealing by time: keep at most 'allowed' voices playing. The victim is
   the quietest note (envelope level x note gain; whole note-ons, so layered
   regions of one note leave together and the stereo image holds). A note is
   as loud as its loudest layer, and a layer still in its delay or attack
   counts at full level: its envelope starts from zero, so a note that has
   just been struck would otherwise be the first one cut. Stolen
   voices get a 1.5 ms release: rendered for one more 64-frame block, then
   freed, instead of the 10 ms default fast release that would keep them in
   the next two periods as well. Voices already in such a release are not
   counted. */
#define VMPU_STEAL_RELEASE 0.0015f
static void VMPU_LimitVoices(int active, int allowed)
{
    float score[256];
    unsigned char taken[256], head[256], slot[128];
    int used = 0;
    struct tsf_voice *v, *vEnd = tsfrenderer->voices + tsfrenderer->voiceNum;
    int n = (int)(vEnd - tsfrenderer->voices);
    if(n > 255) n = 255; //0xFF marks a free slot below
    for(int i = 0; i < n; ++i)
    {
        v = tsfrenderer->voices + i;
        taken[i] = 0;
        head[i] = (unsigned char)i;
        score[i] = -1.0f;
        if(v->playingPreset == -1)
            continue;
        if(v->ampenv.segment >= TSF_SEGMENT_RELEASE && v->ampenv.parameters.release <= VMPU_STEAL_RELEASE * 1.01f)
        {   //already being stolen: ends within this block
            taken[i] = 1;
            --active;
            continue;
        }
        score[i] = (v->ampenv.segment < TSF_SEGMENT_HOLD ? 1.0f : v->ampenv.level) * tsf_decibelsToGain(v->noteGainDB);
        if(v->ampenv.segment >= TSF_SEGMENT_RELEASE)
            score[i] *= 0.5f; //a tail goes before a held note
    }
    //every layer takes the level of the loudest layer of its note (open-addressed table keyed by playIndex)
    for(int i = 0; i < 128; ++i)
        slot[i] = 0xFF;
    for(int i = 0; i < n; ++i)
    {
        if(score[i] < 0.0f)
            continue;
        unsigned int h = tsfrenderer->voices[i].playIndex & 127;
        while(slot[h] != 0xFF && tsfrenderer->voices[slot[h]].playIndex != tsfrenderer->voices[i].playIndex)
            h = (h + 1) & 127;
        if(slot[h] == 0xFF)
        {
            slot[h] = (unsigned char)i; //the first layer found keeps the maximum
            if(++used >= 96) //table nearly full: leave the remaining layers on their own level
                break;
        }
        else if(score[i] > score[slot[h]])
            score[slot[h]] = score[i];
        head[i] = slot[h];
    }
    for(int i = 0; i < n; ++i)
        if(score[i] >= 0.0f && head[i] != i)
            score[i] = score[head[i]];
    while(active > allowed)
    {
        int pick = -1;
        for(int i = 0; i < n; ++i)
            if(!taken[i] && score[i] >= 0.0f && (pick < 0 || score[i] < score[pick]))
                pick = i;
        if(pick < 0)
            break;
        const unsigned int idx = tsfrenderer->voices[pick].playIndex;
        for(int i = 0; i < n; ++i)
        {
            v = tsfrenderer->voices + i;
            if(taken[i] || score[i] < 0.0f || v->playIndex != idx)
                continue;
            taken[i] = 1;
            --active;
            if(score[i] < 0.0005f) //inaudible (-66 dB): just drop it
                tsf_voice_kill(v);
            else
            {
                v->ampenv.parameters.release = VMPU_STEAL_RELEASE;
                tsf_voice_envelope_nextsegment(&v->ampenv, TSF_SEGMENT_SUSTAIN, tsfrenderer->outSampleRate);
            }
        }
        ++vmpu_t_stolen;
    }
}

uint32_t VMPU_Telemetry(unsigned int selector, uint32_t* value)
{
    switch(selector)
    {
        case 0: *value = vmpu_t_msgmax; break;     //max message cycles per ISR
        case 1: *value = vmpu_t_notemax; break;    //max cycles of one MIDI message
        case 2: *value = vmpu_t_rendermax; break;  //max render cycles per ISR
        case 3: *value = vmpu_t_backlog; break;    //max queued bytes
        case 4: *value = vmpu_t_stolen; break;     //voices stolen for the time budget
        case 5: *value = vmpu_t_voicesmax; break;  //max simultaneous voices
        case 6: *value = vmpu_t_deferred; break;   //periods that left messages queued
        case 7: *value = vmpu_period; break;       //period estimate in TSC cycles
        case 8: *value = vmpu_cost; break;         //peak-held cycles per voice-frame
        default: return 0;
    }
    return 1;
}

#if SBEMU_VMPU_HALFRATE
#define VMPU_HALF_CHUNK 256 /* half-rate frames per render call */
static int16_t vmpu_half[VMPU_HALF_CHUNK*2];
static int16_t vmpu_prev[2];  /* last half-rate frame of the previous chunk */
static int16_t vmpu_carry[2]; /* output frame left over by an odd request */
static int vmpu_has_carry = 0;
#endif

void VMPU_GenSamples(int16_t* pcm16, int samples, int freq, BOOL domix)
{
    if (tsfrenderer)
    {
        uint32_t trender;
        int nvoices, frames, allowed;
        fpu_save(fpu_buffer);
        VMPU_Process_Messages(TRUE);
        nvoices = tsf_active_voice_count(tsfrenderer);
        frames = (samples + 1) / 2;
        if(nvoices > (int)vmpu_t_voicesmax) vmpu_t_voicesmax = nvoices;
        if(vmpu_ready && nvoices > VMPU_MIN_VOICES)
        {   //time left until the ISR share is used up, in voices of the measured cost
            uint32_t deadline = vmpu_period / 16 * vmpu_isr_share;
            uint32_t used = vmpu_tsc() - vmpu_t0;
            uint32_t left = used < deadline ? deadline - used : 0;
            /* B = voices that fit in the time left. A stolen voice still renders one
               64-frame block (fraction r of a full voice), so with N voices
               S + r (N - S) <= B  =>  S = (B - r N) / (1 - r) */
            int b = (int)(left / (vmpu_cost * (uint32_t)frames + 1));
            int r16 = (int)(64 * 16 / (frames > 64 ? frames : 64)); //r in 1/16
            allowed = r16 >= 16 ? b : (b * 16 - r16 * nvoices) / (16 - r16);
            if(allowed > b) allowed = b;
            if(allowed < VMPU_MIN_VOICES) allowed = VMPU_MIN_VOICES;
            if(nvoices > allowed)
            {
                VMPU_LimitVoices(nvoices, allowed);
                nvoices = tsf_active_voice_count(tsfrenderer);
            }
        }
        trender = vmpu_tsc();
#if SBEMU_VMPU_HALFRATE
        /* TinySoundFont voices dominate the ISR on a 500 MHz Geode LX (24
           voices needed 2.6M cycles per 6 ms period). Render at half rate and
           interpolate: y[2j] = (h[j-1]+h[j])/2, y[2j+1] = h[j]. */
        tsf_set_samplerate_output(tsfrenderer, freq/2);
        int done = 0;
        while (done < samples)
        {
            if (vmpu_has_carry)
            {
                for (int c = 0; c < 2; ++c)
                    pcm16[done*2+c] = domix ? (int16_t)max(-32768, min(32767, pcm16[done*2+c] + vmpu_carry[c])) : vmpu_carry[c];
                vmpu_has_carry = 0;
                ++done;
                continue;
            }
            int want = samples - done;
            int half = (want + 1) / 2;
            if (half > VMPU_HALF_CHUNK) half = VMPU_HALF_CHUNK;
            tsf_render_short(tsfrenderer, vmpu_half, half, 0);
            for (int j = 0; j < half; ++j)
            {
                int16_t h[2] = { vmpu_half[j*2], vmpu_half[j*2+1] };
                int16_t y0[2], y1[2];
                for (int c = 0; c < 2; ++c)
                {
                    y0[c] = (int16_t)(((int)vmpu_prev[c] + h[c]) >> 1);
                    y1[c] = h[c];
                    vmpu_prev[c] = h[c];
                }
                for (int k = 0; k < 2; ++k)
                {
                    const int16_t* y = k ? y1 : y0;
                    if (done < samples)
                    {
                        for (int c = 0; c < 2; ++c)
                            pcm16[done*2+c] = domix ? (int16_t)max(-32768, min(32767, pcm16[done*2+c] + y[c])) : y[c];
                        ++done;
                    }
                    else
                    {
                        vmpu_carry[0] = y[0]; vmpu_carry[1] = y[1];
                        vmpu_has_carry = 1;
                    }
                }
            }
        }
#else
        tsf_set_samplerate_output(tsfrenderer, freq);
        tsf_render_short(tsfrenderer, pcm16, samples, domix);
#endif
        trender = vmpu_tsc() - trender;
        if(trender > vmpu_t_rendermax) vmpu_t_rendermax = trender;
        if(nvoices > 0)
        {   //cost per voice and frame: rises at once, decays slowly (peak hold)
            uint32_t c = trender / ((uint32_t)nvoices * (uint32_t)frames);
            if(c > vmpu_cost) vmpu_cost = c;
            else vmpu_cost -= (vmpu_cost - c) / 16;
        }
        fpu_restore(fpu_buffer);
    }
}

/* SB-MIDI data written with DSP cmd 0x38 */

void VMPU_SBMidi_RawWrite(uint8_t value)
//////////////////////////////////////////
{
}

uint32_t VMPU_MPU(uint32_t port, uint32_t val, uint32_t out)
////////////////////////////////////////////////////////////
{
    return out ? (VMPU_Write(port, val), val) : (val &= ~0xff, val |= VMPU_Read(port));
}
#endif
