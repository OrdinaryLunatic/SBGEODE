#ifndef _VMPU_H_
#define _VMPU_H_

#include <sbemucfg.h>

#ifdef __cplusplus
extern "C"
{
#endif

#if SBEMU_VMPU

#define VMPU_DEF_SF2 "sbemusf.sf2"

//intialization route, called if option enabled.
BOOL VMPU_Init(int baseaddr, int* voices, int freq, const char* sf2);

//for TSR with pre-loaded memory
//don't change VMPU_Init to pre-loaded memory because streaming take less memory,
//but a full pre-load may fail if the SF is too large.
BOOL VMPU_Reset(int baseaddr, int* voices, int freq, const char* sf2, uint32_t sf2_linear_mem, int bytes);

BOOL VMPU_IsActive();
void VMPU_SetIsrShare(int sixteenths);

void VMPU_GenSamples(int16_t* pcm16, int samples, int freq, BOOL domix);
//tell the synth the real card rate once it is known (VMPU_Init runs before AU_setrate)
void VMPU_SetOutputRate(int freq);
//Geode: ISR start time and measured buffer period (TSC cycles) for the time budget
void VMPU_BeginPeriod(uint32_t t0, uint32_t period_cycles);
//Geode: read-only counters; returns 0 for an unknown selector
uint32_t VMPU_Telemetry(unsigned int selector, uint32_t* value);

uint32_t VMPU_MPU(uint32_t port, uint32_t val, uint32_t out);
void VMPU_SBMidi_RawWrite( uint8_t value );

#else //SBEMU_VMPU

#define VMPU_IsActive() FALSE
#define VMPU_SetIsrShare(x)
#define VMPU_BeginPeriod(t0, period) ((void)0)
#define VMPU_Telemetry(selector, value) 0

#endif //SBEMU_VMPU

#ifdef __cplusplus
}
#endif

#endif//_VMPU_H_
