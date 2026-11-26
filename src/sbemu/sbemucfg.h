#ifndef _SBEMUCFG_H_
#define _SBEMUCFG_H_

#define SBEMU_SAMPLERATE    22050 //not used anymore.
//#define SBEMU_SAMPLERATE    44100 //not used anymore.

#define SBEMU_CHANNELS 2

#define SBEMU_BITS 16

//enable Virtual MPU by using TinySoundFont
#define SBEMU_VMPU 1

//enable PC Speaker (internal beeper) emulation.
//traps PIT channel 2 (0x42/0x43) and the system control port (0x61).
//Geode fork: disabled. VPCSPEAKER_GenSamples() runs in the audio interrupt on every
//period and uses x87 doubles without saving the interrupted program's FPU state;
//ports 42h/43h/61h get trapped (61h is polled by many timing loops); and the Geode
//boards already route the hardware beeper through the codec (PC_BEEP).
#define SBEMU_PCSPEAKER 0

//amplify OPL volume by 1.5. should be 0 or 1
//NOTE: the DBOPL emulation has lower volume, and DOSBox will set the volume to 1.5x too
//reference: https://github.com/dosbox-staging/dosbox-staging/issues/278 : OPL audio 1.5x scaling
//reference: https://github.com/dosbox-staging/dosbox-staging/blob/main/src/hardware/audio/opl.cpp
//
//FIX (clipping + channel imbalance):
//  Enabling the 1.5x amplification here is only correct when OPL and digital SFX are
//  NOT summed in the same linear mix stage. With SBEMU_LINEAR_MIX=1 (see below), the
//  effective OPL gain becomes:
//      OPL_VOLUME_AMPLICATION(1.5x) * OPL_RATIO(12/16) = 1.125x
//  which already exceeds unity on its own, and reaches ~1.375x when summed with the
//  digital SFX path (SFX_RATIO=4/16) in the worst case. Because the linear mix path
//  in main.c has no saturation, this produced hard clipping (audible crackle).
//  Since the SBEMU_LINEAR_MIX path now also applies a proper clamp (see main.c), the
//  extra 1.5x is no longer needed and would only eat headroom. Keep it disabled.
//Geode fork: kept at 1. The levels below (15/16 PCM, 18/16 OPL) were calibrated
//against DOSBox-X on hardware with the 1.5x in place, and the mix saturates.
#define SBEMU_OPL_VOLUME_AMPLICATION 1

//threshold for fixtc. if sample rate difference larger than this, fixtc will be disabled
//set to 0 to alwasy use fixtc ignoring the difference
//this is the default threshold for fixtc, can be overridded by command line.
#define SBEMU_FIXTC_THRESHOLD 1000

//swap left/right chanel (SFX only)
//it has historical reasons related to the legacy SB cards so previously I was reluctant to do it.
#define SBEMU_SWAP_STEREO 0

//mixing method 2: (SFX*a+PCM*b)
#define SBEMU_LINEAR_MIX 1

//static volume balancing (because DBOPL volume is lower than real HW, even with SBEMU_OPL_VOLUME_AMPLICATION=1)
//FIX (SFX/OPL level balance):
//  The previous split (SFX=4/16, OPL=(16-4)/16 * 1.5x amplification) made OPL ~4.5x
//  louder than digital SFX, so games that use both (e.g. SB digital + FM music) had
//  the FX buried under the music. With OPL amplification disabled above, a 50/50
//  split gives SFX=OPL=0.5 and a theoretical maximum sum of 1.0 (no overflow when
//  clamped), restoring perceptual balance without touching either engine's output.
#define SBEMU_LINEAR_MIX_FRACTION 16
// Level calibration (2026-10-10): with the AC'97 PCM Out at 0 dB the 10/16
// and 12/16 coefficients measured 4.75 dB (PCM) and 3.5 dB (OPL) below
// DOSBox-X at the same capture gain; 15/16 and 18/16 (+3.5 dB) bring both
// within ~1 dB of it. Mix output is saturated, not wrapped.
#define SBEMU_LINEAR_MIX_SFX_RATIO 15
// Keep digital voice gain constant when OPL starts or stops. The combined
// 10/16 path was measured close to DOSBox-X; OPL retains its separate 12/16.
#define SBEMU_LINEAR_MIX_COMBINED_SFX_RATIO SBEMU_LINEAR_MIX_SFX_RATIO
#define SBEMU_LINEAR_MIX_OPL_RATIO 18

#if SBEMU_LINEAR_MIX
#define SBEMU_SFX_RATIO SBEMU_LINEAR_MIX_SFX_RATIO/SBEMU_LINEAR_MIX_FRACTION //don't use () for int math, careful on use
#define SBEMU_OPL_RATIO SBEMU_LINEAR_MIX_OPL_RATIO/SBEMU_LINEAR_MIX_FRACTION //don't use () for int math, careful on use
#else
#define SBEMU_SFX_RATIO 1
#define SBEMU_OPL_RATIO 1
#endif

//sound card master volume
#define SBEMU_VOLUME_MAX 100

#endif//_SBEMUCFG_H_
