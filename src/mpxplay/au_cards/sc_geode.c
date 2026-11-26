// SPDX-License-Identifier: GPL-2.0-only
/* AMD Geode CS5536 AC97 playback backend for SBEMU. */

#include "au_cards.h"

#ifdef AU_CARDS_LINK_GEODE

#include <stdio.h>
#include "ac97_def.h"
#include "dmairq.h"
#include "pcibios.h"

#define GEODE_ACC_GPIO_STATUS  0x00
#define GEODE_ACC_STATUS       0x08
#define GEODE_ACC_CONTROL      0x0c
#define GEODE_ACC_IRQ_STATUS   0x12
#define GEODE_BM0_CMD          0x20
#define GEODE_BM0_STATUS       0x21
#define GEODE_BM0_PRD          0x24
#define GEODE_BM0_POINTER      0x60

#define GEODE_PRM_READY        0x00800000UL
#define GEODE_CMD_NEW          0x00010000UL
#define GEODE_STS_NEW          0x00020000UL
#define GEODE_CODEC_READ       0x80000000UL
#define GEODE_LNK_WARM_RESET   0x00020000UL
#define GEODE_IRQ_CODEC_WU     0x0003 /* codec status, wakeup: cleared by reading ACC_GPIO_STATUS */
#define GEODE_IRQ_BM0          0x0004
#define GEODE_IRQ_BM_OTHER     0x03f8 /* BM1..BM7: cleared by reading their status (29h, 31h, ... 59h) */
#define GEODE_BM_EOP           0x01
#define GEODE_BM_EOP_ERR       0x02
#define GEODE_BM_ENABLE        0x01

#define GEODE_PRD_JUMP         0x2000
#define GEODE_PRD_EOP          0x4000
#define GEODE_PERIODS          4
/* Each 6 ms period is two PRDs, and only one PRD in the ring carries EOP: the
   one after the playing one, moved on every interrupt. The bus
   master pauses when it reaches a second EOP before BM0_STATUS was read
   (data book 5.3.7.2); with a single armed EOP a CPU stall (VSA/SMI: video
   mode switches, VGA emulation) no longer stops the output - the rendered
   audio in the ring keeps playing. The interrupt still comes every period. */
#define GEODE_SUBS             2
#define GEODE_PRDS             (GEODE_PERIODS*GEODE_SUBS)
#define GEODE_CODEC_RETRIES    1000
#define GEODE_CODEC_RETRIES_ISR 20   /* 200 us: bounded stall inside the interrupt */

typedef struct geode_prd_s {
 uint32_t address;
 uint16_t size;
 uint16_t flags;
} __attribute__((packed)) geode_prd_s;

typedef struct geode_card_s {
 unsigned long iobase;
 unsigned int irq;
 uint16_t pci_command;
 int pci_master_enabled;
 struct pci_config_s *pci_dev;
 cardmem_t *dm;
 geode_prd_s *prd;
 char *pcm_buffer;
 unsigned int pcm_capacity;
 unsigned int period_bytes;
 int bm0_running;
 uint32_t pcm_physical;
 volatile uint32_t irq_eop;
 volatile uint32_t irq_eop_error;
 volatile uint32_t pointer_invalid;
 volatile uint32_t irq_spurious;
 volatile uint32_t min_dmafilled;
 unsigned int eop_index;          /* the one PRD that carries EOP */
 volatile uint32_t eop_skipped;   /* interrupts that found the bus master more than a period on: a stall ridden out */
 volatile uint32_t pointer_wrap_recovered;
} geode_card_s;

/* Codec register access from the interrupt handler (CD-IN level changed by
   the guest) must not spin for the full 10 ms a dead codec would take.
   GEODE_set_isr_context(1) bounds every wait; a write that could not be
   completed in time sets geode_isr_failed so the caller retries next period. */
static volatile unsigned int geode_retry_limit=GEODE_CODEC_RETRIES;
static volatile unsigned int geode_isr_failed;

void GEODE_set_isr_context(int isr)
{
 geode_retry_limit=isr ? GEODE_CODEC_RETRIES_ISR : GEODE_CODEC_RETRIES;
 geode_isr_failed=0;
}

int GEODE_isr_context_failed(void)
{
 return geode_isr_failed!=0;
}

static pci_device_s geode_devices[] = {
 {"CS5536", 0x1022, 0x2093, 0},
 {"CS5535", 0x100b, 0x002e, 0}, /* same ACC block; not tested on hardware */
 {NULL, 0, 0, 0}
};

static int geode_wait_control(geode_card_s *card)
{
 unsigned int retry;
 for(retry=0; retry<geode_retry_limit; retry++){
  if(!(inl(card->iobase+GEODE_ACC_CONTROL)&GEODE_CMD_NEW))
   return 1;
  pds_delay_10us(1);
 }
 geode_isr_failed=1;
 return 0;
}

static int geode_codec_read_raw(geode_card_s *card,unsigned int reg,uint16_t *value)
{
 unsigned int retry;
 uint32_t status;

 outl(card->iobase+GEODE_ACC_CONTROL,
      GEODE_CODEC_READ|GEODE_CMD_NEW|((uint32_t)(reg&0x7f)<<24));
 if(!geode_wait_control(card))
  return 0;
 for(retry=0; retry<geode_retry_limit; retry++){
  status=inl(card->iobase+GEODE_ACC_STATUS);
  if((status&GEODE_STS_NEW) && ((status>>24)&0x7f)==(reg&0x7f)){
   *value=(uint16_t)status;
   return 1;
  }
  pds_delay_10us(1);
 }
 geode_isr_failed=1;
 return 0;
}

static int geode_codec_write_raw(geode_card_s *card,unsigned int reg,uint16_t value)
{
 uint32_t command=((uint32_t)(reg&0x7f)<<24)|GEODE_CMD_NEW|value;
 command&=~GEODE_CODEC_READ;
 outl(card->iobase+GEODE_ACC_CONTROL,command);
 return geode_wait_control(card);
}

/* Generic SBEMU mixer code does read-modify-write of single channel halves
   in a 6-bit attenuation domain. Keep a shadow of what it believes it wrote
   for the three output registers and translate to the codec's 5-bit fields
   only on the way out. */
static uint16_t geode_shadow[3];
static unsigned int geode_shadow_valid;

static int geode_shadow_index(unsigned long reg)
{
 switch(reg){
  case AC97_MASTER_VOL_STEREO: return 0;
  case AC97_HEADPHONE_VOL: return 1;
  case AC97_PCMOUT_VOL: return 2;
 }
 return -1;
}

static uint16_t geode_hw_value(unsigned long reg,uint16_t v)
{
 uint16_t l=(v>>8)&0x3f,r=v&0x3f;
 l=(l+1)>>1;
 r=(r+1)>>1;
 if(l>0x1f) l=0x1f;
 if(r>0x1f) r=0x1f;
 if(reg==AC97_PCMOUT_VOL){
  /* 0 dB is 08h; anything below that is analog gain up to +12 dB */
  if(l<8) l=8;
  if(r<8) r=8;
 }
 return (v&0x8000)|(l<<8)|r;
}

static void GEODE_writeMIXER(struct mpxplay_audioout_info_s *aui,
                             unsigned long reg,unsigned long value)
{
 geode_card_s *card=aui->card_private_data;
 uint16_t v=(uint16_t)value;
 int idx=geode_shadow_index(reg);
 /* The generic SBEMU mixer drives every AC'97 output to 100 %. On AC'97
    PCM Out (18h) that register value is +12 dB of analog gain, which
    overdrives the codec mixer. Keep PCM Out at unity and let /VOL drive
    Master and Headphone together so /VOL follows either analog output. The generic code also
    writes 6-bit attenuation (0..63) while the ALC203 has 5-bit fields;
    values above 31 would wrap around and get louder, so rescale. */
 if(idx>=0){
  geode_shadow[idx]=v;
  if(reg==AC97_HEADPHONE_VOL)
   return; /* follows Master */
  if(reg==AC97_MASTER_VOL_STEREO)
   geode_codec_write_raw(card,AC97_HEADPHONE_VOL,geode_hw_value(reg,v));
  v=geode_hw_value(reg,v);
 }
 geode_codec_write_raw(card,(unsigned int)reg,v);
}

static unsigned long GEODE_readMIXER(struct mpxplay_audioout_info_s *aui,
                                     unsigned long reg)
{
 geode_card_s *card=aui->card_private_data;
 uint16_t value=0xffff;
 int idx=geode_shadow_index(reg);
 if(idx>=0){
  if(!(geode_shadow_valid&(1u<<idx))){
   geode_codec_read_raw(card,(unsigned int)reg,&value);
   geode_shadow[idx]=value;
   geode_shadow_valid|=1u<<idx;
  }
  return geode_shadow[idx];
 }
 geode_codec_read_raw(card,(unsigned int)reg,&value);
 return value;
}

/* CD-IN level from the audio interrupt, integer only. The generic
   AU_setmixer_one() computes in floating point, and the interrupt must not
   touch the x87 state of the interrupted program. percent 0..100, 100 = no
   attenuation (as the generic code maps it), 0 = mute. Returns 0 when the
   codec did not accept the write in time (caller retries next period). */
int GEODE_set_cdin_isr(struct mpxplay_audioout_info_s *aui,unsigned int percent)
{
 geode_card_s *card=aui->card_private_data;
 unsigned int att;
 uint16_t value;
 int ok;
 if(!card)
  return 1;
 if(percent>100)
  percent=100;
 att=(100-percent)*31/100;
 value=(uint16_t)((att<<8)|att);
 if(!percent)
  value|=0x8000;
 GEODE_set_isr_context(1);
 ok=geode_codec_write_raw(card,AC97_CD_VOL,value);
 GEODE_set_isr_context(0);
 return ok;
}

static void GEODE_stop(struct mpxplay_audioout_info_s *aui)
{
 geode_card_s *card=aui->card_private_data;
 if(card && card->bm0_running){
  outb(card->iobase+GEODE_BM0_CMD,0);
  inb(card->iobase+GEODE_BM0_STATUS); /* Acknowledge pending BM0 IRQ before teardown. */
  card->bm0_running=0;
 }
}

static void GEODE_free(struct mpxplay_audioout_info_s *aui)
{
 geode_card_s *card=aui->card_private_data;
 if(!card)
  return;
 if(card->dm)
  MDma_free_cardmem(card->dm);
 if(card->pci_master_enabled){
  uint16_t command=pcibios_ReadConfig_Word(card->pci_dev,PCIR_PCICMD);
  pcibios_WriteConfig_Word(card->pci_dev,PCIR_PCICMD,(command&~5u)|(card->pci_command&5u));
 }
 if(card->pci_dev)
  pds_free(card->pci_dev);
 pds_free(card);
 aui->card_private_data=NULL;
 aui->card_DMABUFF=NULL;
 aui->card_pci_dev=NULL;
}

static void GEODE_close(struct mpxplay_audioout_info_s *aui)
{
 GEODE_stop(aui);
 GEODE_free(aui);
}

static void GEODE_card_info(struct mpxplay_audioout_info_s *aui)
{
 geode_card_s *card=aui->card_private_data;
 char text[100];
 sprintf(text,"Geode CS5536 AC97 at %4.4lX irq:%u, 48kHz S16LE stereo",
         card->iobase,card->irq);
 pds_textdisplay_printf(text);
}

static int GEODE_detect(struct mpxplay_audioout_info_s *aui)
{
 geode_card_s *card;
 uint32_t bar;
 uint16_t vendor1,vendor2;
 unsigned int descriptor_bytes=(GEODE_PRDS+1)*sizeof(geode_prd_s);

 card=(geode_card_s *)pds_calloc(1,sizeof(geode_card_s));
 if(!card)
  return 0;
 aui->card_private_data=card;
 card->pci_dev=(struct pci_config_s *)pds_calloc(1,sizeof(struct pci_config_s));
 if(!card->pci_dev)
  goto error;
 if(pcibios_search_devices(geode_devices,card->pci_dev)!=PCI_SUCCESSFUL)
  goto error;

 bar=pcibios_ReadConfig_Dword(card->pci_dev,PCIR_NAMBAR);
 uint32_t base=bar&~3u;
 if(!(bar&1) || base<0x100 || base>0x10000u-GEODE_BM0_POINTER-4)
  goto error;
 card->iobase=base;
 card->irq=pcibios_ReadConfig_Byte(card->pci_dev,PCIR_INTR_LN);
 if(card->irq<2 || card->irq>15)
  goto error;
 card->pci_command=pcibios_ReadConfig_Word(card->pci_dev,PCIR_PCICMD);
 pcibios_set_master(card->pci_dev);
 card->pci_master_enabled=1;
 /* Firmware may leave the AC link down. A warm reset wakes the codec without
    touching its registers (the Linux driver always issues one); then wait up
    to 50 ms for the primary codec to report ready. */
 if(!(inl(card->iobase+GEODE_ACC_STATUS)&GEODE_PRM_READY)){
  unsigned int retry;
  outl(card->iobase+GEODE_ACC_CONTROL,GEODE_LNK_WARM_RESET);
  for(retry=0; retry<5000; retry++){
   if(inl(card->iobase+GEODE_ACC_STATUS)&GEODE_PRM_READY)
    break;
   pds_delay_10us(1);
  }
 }
 if(!(inl(card->iobase+GEODE_ACC_STATUS)&GEODE_PRM_READY))
  goto error;
 if(!geode_codec_read_raw(card,AC97_VENDOR_ID1,&vendor1) ||
    !geode_codec_read_raw(card,AC97_VENDOR_ID2,&vendor2) ||
    vendor1==0xffff || vendor2==0xffff)
  goto error;

 card->pcm_capacity=MDma_get_max_pcmoutbufsize(aui,0,512,2,48000);
 card->dm=MDma_alloc_cardmem(descriptor_bytes+card->pcm_capacity);
 if(!card->dm)
  goto error;
 card->prd=(geode_prd_s *)card->dm->linearptr;
 card->pcm_buffer=card->dm->linearptr+descriptor_bytes;
 card->pcm_physical=(uint32_t)pds_cardmem_physicalptr(card->dm,card->pcm_buffer);
 aui->card_DMABUFF=card->pcm_buffer;
 aui->card_irq=(unsigned char)card->irq;
 aui->card_pci_dev=card->pci_dev;
 return 1;

error:
 GEODE_free(aui);
 return 0;
}

static void GEODE_setrate(struct mpxplay_audioout_info_s *aui)
{
 geode_card_s *card=aui->card_private_data;
 uint32_t prd_physical;
 unsigned int i;

 aui->card_wave_id=MPXPLAY_WAVEID_PCM_SLE;
 aui->freq_card=48000;
 aui->chan_card=2;
 aui->bits_card=16;
 geode_shadow_valid=0;
 geode_codec_write_raw(card,AC97_PCM_FRONT_DAC_RATE,48000);
 /* The generic SBEMU mixer changes attenuation fields but preserves AC97's
    bit-15 mute. Firmware leaves this codec muted, so establish an audible
    baseline before the generic mixer applies the requested percentages. */
 geode_codec_write_raw(card,AC97_MASTER_VOL_STEREO,0x0808);
 geode_codec_write_raw(card,AC97_HEADPHONE_VOL,0x0808);
 geode_codec_write_raw(card,AC97_PCMOUT_VOL,0x0808);
 card->period_bytes=MDma_init_pcmoutbuf(aui,card->pcm_capacity,512,48000)
                    /GEODE_PERIODS;
 card->period_bytes-=card->period_bytes%(4*GEODE_SUBS);
 card->irq_eop=0;
 card->irq_eop_error=0;
 card->pointer_invalid=0;
 card->pointer_wrap_recovered=0;
 card->irq_spurious=0;
 card->min_dmafilled=0xffffffffUL;
 for(i=0;i<GEODE_PRDS;i++){
  card->prd[i].address=card->pcm_physical+i*(card->period_bytes/GEODE_SUBS);
  card->prd[i].size=(uint16_t)(card->period_bytes/GEODE_SUBS);
  card->prd[i].flags=(i==GEODE_SUBS-1) ? GEODE_PRD_EOP : 0; /* first IRQ: end of period 0 */
 }
 card->eop_index=GEODE_SUBS-1;
 prd_physical=(uint32_t)pds_cardmem_physicalptr(card->dm,card->prd);
 card->prd[GEODE_PRDS].address=prd_physical;
 card->prd[GEODE_PRDS].size=0;
 card->prd[GEODE_PRDS].flags=GEODE_PRD_JUMP;
 aui->card_samples_per_int=card->period_bytes/4;
 outb(card->iobase+GEODE_BM0_CMD,0);
 card->bm0_running=0;
 inb(card->iobase+GEODE_BM0_STATUS);
 outl(card->iobase+GEODE_BM0_PRD,
      prd_physical+GEODE_PRDS*sizeof(geode_prd_s));
}

static void GEODE_start(struct mpxplay_audioout_info_s *aui)
{
 geode_card_s *card=aui->card_private_data;
 inb(card->iobase+GEODE_BM0_STATUS);
 card->bm0_running=1;
 outb(card->iobase+GEODE_BM0_CMD,GEODE_BM_ENABLE);
}

static long GEODE_getbufpos(struct mpxplay_audioout_info_s *aui)
{
 geode_card_s *card=aui->card_private_data;
 const unsigned long port=card->iobase+GEODE_BM0_POINTER;
 const uint32_t base=card->pcm_physical;
 const uint32_t size=aui->card_dmasize;
 uint32_t pointer=inl(port);
 uint32_t position;

 if(pointer<base)
  goto invalid;
 position=pointer-base;
 if(position>=size){
  /* BM0 can briefly report one-past PCM at the ring transition.
     The JMP target may appear before the first PCM descriptor is ready. */
  if(position!=size)
   goto invalid;
  pointer=inl(port);
  if(pointer==card->prd[GEODE_PRDS].address)
   pointer=inl(port);
  position=pointer-base;
  if(pointer<base || position>=size || position>=card->period_bytes/GEODE_SUBS)
   goto invalid;
  card->pointer_wrap_recovered++;
 }
 aui->card_dma_lastgoodpos=position;
 return position;

invalid:
 card->pointer_invalid++;
 return aui->card_dma_lastgoodpos;
}

static int GEODE_irq(struct mpxplay_audioout_info_s *aui)
{
 geode_card_s *card=aui->card_private_data;
 uint8_t status;
 uint16_t irq_status=inw(card->iobase+GEODE_ACC_IRQ_STATUS);
 unsigned int n;
 if(!irq_status)
  return 0;
 /* Sources this driver never enables. Firmware of another board may have
    left one enabled; an unacknowledged level interrupt would repeat forever,
    so clear them the way the Linux driver does. */
 if(irq_status&GEODE_IRQ_CODEC_WU)
  inl(card->iobase+GEODE_ACC_GPIO_STATUS);
 if(irq_status&GEODE_IRQ_BM_OTHER)
  for(n=1;n<8;n++)
   if(irq_status&(GEODE_IRQ_BM0<<n))
    inb(card->iobase+GEODE_BM0_STATUS+8*n);
 if(!(irq_status&GEODE_IRQ_BM0)){
  card->irq_spurious++;
  return 1;
 }
 /* Reading BM0_STATUS clears EOP, EOP_ERR and the BM0 bit of ACC_IRQ_STATUS
    (data book 6.3.2.5/6.3.2.8, read-to-clear), and resumes a bus master that
    paused on EOP_ERR. The source is gone after this read, so the interrupt
    is always ours: claim it (and send EOI) even if neither flag was set,
    instead of chaining it to the previous handler. */
 status=inb(card->iobase+GEODE_BM0_STATUS);
 if(status&GEODE_BM_EOP)
  card->irq_eop++;
 if(status&GEODE_BM_EOP_ERR)
  card->irq_eop_error++;
 if(!(status&(GEODE_BM_EOP|GEODE_BM_EOP_ERR)))
  card->irq_spurious++;
 { /* move the single EOP to the PRD after the one playing now (one period
     after the PRD that raised this interrupt) */
  uint32_t base=card->pcm_physical,pointer=inl(card->iobase+GEODE_BM0_POINTER);
  unsigned int sub=card->period_bytes/GEODE_SUBS,i,cur,next,extra;
  volatile geode_prd_s *prd=card->prd; /* the bus master reads these: keep every store */
  if(sub && pointer>=base && pointer-base<sub*GEODE_PRDS){
   cur=(pointer-base)/sub;
   next=(cur+1)%GEODE_PRDS;
   if(cur!=(card->eop_index+1)%GEODE_PRDS)
    card->eop_skipped++;
   prd[next].flags=GEODE_PRD_EOP;
   /* if the bus master crossed into 'next' meanwhile it may have read that PRD
      without EOP: arm the one after it too, or no EOP would be left */
   pointer=inl(card->iobase+GEODE_BM0_POINTER);
   extra=(pointer>=base && pointer-base<sub*GEODE_PRDS && (pointer-base)/sub!=cur) ? (next+1)%GEODE_PRDS : next;
   prd[extra].flags=GEODE_PRD_EOP;
   for(i=0;i<GEODE_PRDS;i++)
    if(i!=next && i!=extra)
     prd[i].flags=0;
   card->eop_index=extra;
  }
 }
 if(aui->card_dmafilled<card->min_dmafilled)
  card->min_dmafilled=aui->card_dmafilled;
 return 1;
}

/* Read-only counters for the diagnostic TSR multiplex call. No diagnostic
   UART or DOS I/O runs in the hardware interrupt. Counters wrap at 32 bits. */
int GEODE_telemetry_read(struct mpxplay_audioout_info_s *aui,
                         unsigned int selector,uint32_t *value)
{
 geode_card_s *card=aui->card_private_data;
 unsigned int i;
 uint32_t count;
 int16_t sample;
 uint32_t peak;
 if(!card || !value)
  return 0;
 switch(selector){
  case 0: *value=2; break; /* telemetry ABI version */
  case 1: *value=card->irq_eop; break;
  case 2: *value=card->irq_eop_error; break;
  case 3: *value=card->pointer_invalid; break;
  case 4: *value=card->min_dmafilled; break;
  case 5: *value=aui->card_dmafilled; break;
  case 6: *value=aui->card_dmasize; break;
  case 7: *value=card->period_bytes; break;
  case 25: *value=card->eop_skipped; break; /* 22 is VMPU; read with an NSEL=32 launcher */
  case 26: *value=card->pointer_wrap_recovered; break;
  case 23: *value=card->irq_spurious; break; /* IRQ without EOP/EOP_ERR, or from another ACC source */
  case 9:
  case 10:
   /* Snapshot the final S16LE ring while idle. An IRQ may refill it during
      this scan, so repeat the query; zero on every scan rules out a steady
      nonzero digital idle stream without touching BM0 or codec state. */
   count=0;
   peak=0;
   for(i=0; i<aui->card_dmasize/2; i++){
    sample=((int16_t *)card->pcm_buffer)[i];
    if(sample){
     uint32_t magnitude=sample<0 ? -(int32_t)sample : sample;
     count++;
     if(magnitude>peak)
      peak=magnitude;
    }
   }
   *value=selector==9 ? count : peak;
   break;
  default: return 0;
 }
 return 1;
}

one_sndcard_info GEODE_sndcard_info={
 "Geode CS5536",
 SNDCARD_LOWLEVELHAND|SNDCARD_INT08_ALLOWED,
 NULL,
 NULL,
 &GEODE_detect,
 &GEODE_card_info,
 &GEODE_start,
 &GEODE_stop,
 &GEODE_close,
 &GEODE_setrate,
 &MDma_writedata,
 &GEODE_getbufpos,
 &MDma_clearbuf,
 NULL,
 &GEODE_irq,
 &GEODE_writeMIXER,
 &GEODE_readMIXER,
 &mpxplay_aucards_ac97chan_mixerset[0]
};

#endif
