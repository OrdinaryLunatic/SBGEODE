# SBGEODE

**SBGEODE is a resident Sound Blaster emulator for DOS on the AMD Geode LX / CS5536.** When a game programs the Sound Blaster DSP and an ISA DMA channel, SBGEODE traps the port writes, reads the sample data from the game's buffer and raises the virtual Sound Blaster IRQ at the DSP block boundary. OPL register writes feed the FM synthesizer. The mixer combines PCM and FM with optional SoundFont MIDI and feeds the result to the CS5536 AC’97 playback engine.

On the Geode, the playback interrupt can arrive while DOS is busy changing video modes. SBGEODE uses a four-period DMA ring and moves a single end-of-period marker as playback advances. That gives the controller queued samples to play through a short delay in interrupt service. On the game side, the virtual sound IRQ can reach a real-mode handler even when the game itself runs under a 32-bit DOS extender.

We checked the analog output of a **500 MHz Geode LX with a CS5536 and a Realtek ALC203 codec**. Recordings cover FM with digital effects, speech streamed from XMS, General MIDI through SoundFont banks, and tracker playback through the emulated DSP. [Hardware results by sound system](#audio-systems-exercised-on-hardware)

[Download SBGEODE 1.0 for DOS](https://github.com/OrdinaryLunatic/SBGEODE/releases/download/v1.0/SBGEODE-1.0.zip) · [Полная версия на русском](#на-русском)

## Requirements and first run

**MS-DOS 6.22 is the primary platform for SBGEODE.** Operation has also been confirmed on FreeDOS and MS-DOS 7.1. The physical MS-DOS 6.22 tests used JemmEx 5.87, QPIEMU and HDPMI32i.

| Application | Required services |
| --- | --- |
| Real-mode DOS | I/O trapping through JemmEx or Jemm386 with QPIEMU loaded, or through a compatible QEMM installation. JemmEx supplies both XMS and expanded-memory services. With Jemm386, load a separate XMS manager such as HIMEMX first. |
| 32-bit protected-mode DOS | HDPMI32i supplies protected-mode I/O trapping. The DOS extender must run the application through DPMI for this path to work. |
| Both types | Configure the real-mode memory manager at boot and load HDPMI32i before SBGEODE. |

With JemmEx or Jemm386 already loaded, put JLOAD, HDPMI32i and SBGEODE on the DOS path. Run the following once after boot from the directory containing `QPIEMU.DLL`, or give JLOAD the full path to the DLL.

```dos
JLOAD.EXE QPIEMU.DLL
HDPMI32I.EXE -r -x
SET BLASTER=A220 I5 D1 H5 T6
SBGEODE.EXE /T=6
```

| Audio interface | Application and driver settings |
| --- | --- |
| Digital effects and speech | **Sound Blaster 16**, address **220h**, IRQ **5**, DMA **1**, high DMA **5**. For an older title that only offers SB Pro, select **Sound Blaster Pro** and start SBGEODE with `/T=4`. |
| FM music | **AdLib** or **OPL** at port **388h**. |
| General MIDI music | Select **General MIDI / MPU-401** at **330h**. Supply a SoundFont 2 bank and start the driver with `SBGEODE.EXE /T=6 /VMPU=1 /VMSF=C:\MIDI\BANK.SF2` for 64 voices. Without `/VMSF`, the driver looks for `SBEMUSF.SF2`. |

**Start with the default XMS setting.** Speech and effects can play directly from XMS whenever the allocated sample buffer fits below 16 MiB. If FM music plays but digital voices or effects are missing or garbled, first check the game's digital device, IRQ and DMA settings. If those match, reboot and try `SBGEODE.EXE /T=6 /XMSLOW` as the first driver invocation. The switch changes where legacy XMS clients can allocate memory. It can help one program and leave too little low memory for another.

Run `SBGEODE.EXE` again to inspect the resident settings.

Memory-manager downloads: [Jemm and QPIEMU](https://github.com/Baron-von-Riedesel/Jemm) · [HDPMI32i](https://github.com/crazii/HX)

## Command-line options

<details>
<summary>Sound Blaster settings, mixer controls, MIDI and compatibility switches</summary>

| Option | What it controls |
| --- | --- |
| `/A`, `/I`, `/D`, `/H` | Emulated base address (220h or 240h), IRQ (5, 7 or 9), 8-bit DMA (0, 1 or 3), and 16-bit DMA (5, 6 or 7). These can also be supplied through `BLASTER`. |
| `/T` | Sound Blaster type. Values 1–3 select the early SB types, 4–5 select SB Pro types, and 6 selects SB16. |
| `/RM`, `/PM` | Enable or disable real-mode and protected-mode support independently. For example, `/RM=0` selects a protected-mode-only configuration. |
| `/NOVCPI` | Disable VCPI in Jemm after loading. Use this when an extender chooses VCPI ahead of DPMI. The default leaves the current Jemm setting alone. |
| `/XMSLOW` | Limit new legacy XMS allocations to blocks entirely below 16 MiB. Off by default. Select this at the first driver invocation after boot. |
| `/VOL`, `/SFXVOL` | Master volume and digital-effects volume, each 0–100. Lower `/SFXVOL` to reduce effects while preserving the FM level. |
| `/OPL`, `/FMQ` | Enable or disable FM emulation. FM quality 0 selects the synthesis rate automatically, 1 forces half rate and 2 forces full rate. |
| `/K`, `/FILT`, `/SWAP` | Internal sample rate in Hz, PCM filtering and stereo channel order. Filtering ranges from 0 (linear) to 2 (sinc interpolation). Swap mode 1 reverses PCM only, and 2 reverses the entire mix. |
| `/FIXTC`, `/VIRQ` | Correct legacy DSP time constants for common sample rates, and select the virtual sound IRQ route. `/VIRQ=1` uses the real-mode vector for protected-mode clients with a real-mode sound handler. `/VIRQ=0` uses the protected-mode vector. |
| `/P`, `/MCOM` | MPU-401 UART address and external MIDI routing. The default address is 330h. For serial MIDI output, `/MCOM=1` through `4` select COM1 through COM4. Value 9 selects the hardware MPU. |
| `/VMPU`, `/VMSF`, `/MSHARE` | Enable software MIDI synthesis and select a SoundFont 2 bank. `/VMPU=0` disables the synth, `/VMPU=1` selects 64 voices, and 32–256 sets an explicit voice limit. `/MSHARE` assigns 4–14 sixteenths of an audio period to MIDI synthesis (default 10). A smaller budget leaves more CPU time for the application but can cut notes under load. |
| `/?` | Display the executable’s option list and accepted values. |

</details>

## How SBGEODE differs from SBEMU

SBGEODE builds on crazii’s SBEMU. The upstream CS5535/CS5536 driver submission reports a successful DJGPP build and lists testing on physical hardware as pending. [Upstream test plan](https://github.com/crazii/SBEMU/pull/196)

We wrote the CS5536 AC’97 playback driver from scratch for SBGEODE, independently of SBEMU’s upstream CS5535/CS5536 driver and without using its code. SBGEODE 1.0 also changes how DOS applications interact with the emulated Sound Blaster. On the DOS side, they cover interrupt delivery, sample decoding and XMS buffer placement. On the controller side, they cover DMA scheduling and the analog mixer. The results below come from a physical **500 MHz AMD Geode LX, CS5536 controller and Realtek ALC203 AC’97 codec** under MS-DOS 6.22. [SBEMU](https://github.com/crazii/SBEMU) · [Upstream CS5535/CS5536 driver](https://github.com/crazii/SBEMU/blob/43722f7318212dc50bac2df852ff16a3b3097067/mpxplay/au_cards/sc_cs5535.c)

| Area | What SBGEODE changes | Why it matters |
| --- | --- | --- |
| CS5536 playback | The controller can keep playing queued audio through a short CPU stall. The driver also handles a boundary condition when the hardware advances to the next playback buffer. | A brief interruption in CPU service need not become a gap at the speakers. Long stalls can still exhaust the queued audio. |
| Digital effects | The PCM mixer preserves sampling phase across blocks, offers eight-tap sinc interpolation, and holds the last DAC level when a transfer ends. Effects have their own volume control. | Short effects can end without an abrupt jump to zero. You can turn down effects while keeping FM music at the same level. |
| Sample playback compatibility | SBGEODE handles the SB16 DSP 42h sample-rate command and preserves the decoder state between blocks of Creative ADPCM audio. | Software using that rate command can play samples at the intended pitch and tempo. Compressed effects can continue across block boundaries without restarting the decoder. |
| FM and General MIDI | FM can run at half or full synthesis rate. The SoundFont mixer uses a Geode-optimized voice loop and lets you set the share of CPU time spent rendering MIDI. | You can favor richer music or leave more processor time for the application. Dense arrangements can still lose notes. |
| DOS sound drivers | A protected-mode application's virtual sound interrupt can reach a resident real-mode handler. With Jemm, VCPI can be disabled after loading so compatible extenders use DPMI. | This covers software that runs in protected mode but relies on a real-mode sound driver, as well as extenders that otherwise bypass DPMI sound trapping. |
| Audio buffers in XMS | The optional `/XMSLOW` mode keeps new legacy XMS allocations entirely below the 16 MiB ISA DMA boundary. | Speech or effects streamed directly from XMS can play alongside FM music even when the machine has much more than 16 MiB of RAM. |

## Audio systems exercised on hardware

These results were measured with the 599,552-byte `SBGEODE.EXE` shipped in **1.0**, under MS-DOS 6.22 on the Geode LX / CS5536.

| Sound system or output path | Result with the v1.0 executable |
| --- | --- |
| DMX · Sound Blaster PCM with OPL music | Digital effects were audible at startup, after a level change, and after a normal return to DOS and relaunch on the same boot. |
| Real-mode Sound Blaster PCM | Individually triggered effects were identifiable in the analog recording. |
| Protected-mode digital audio | The recorded passages matched the reference audio. |
| XMS-backed digital effects | With `/XMSLOW`, the DMA address pointed to the original sample below 16 MiB and the effect was audible. |
| Miles · XMS-backed digitized speech | Speech played with the default XMS policy when the DMA address pointed to the original sample below 16 MiB. The voice was identified in the analog recording. |
| Creative ADPCM · block continuation | The recorded tone continued across the transition from the reference block to the continuation block. |
| Stereo PCM · channel routing | Left-only, right-only and two-channel passages played in sequence. The measured tones were 997 Hz and 1,499 Hz. |
| OPL timer registers | The timer flags observed by the DOS application matched the expected 80 µs and 320 µs intervals. |
| Sound Blaster DSP · sample transfer modes | The transfers produced identifiable tones. With `/FIXTC=0`, the measured frequencies matched the programmed rates. |
| Sound Blaster 16 · DSP rate command 42h | Substituting 42h for 41h when setting the sample rate preserved the measured pitch with `/FIXTC=0`. |
| HMI OPL2 · default XMS and `/XMSLOW` | The recorded passages matched across both XMS settings. |

**Application playback on SBGEODE 1.0.** Each output listed below was selected in a DOS application and confirmed by comparing the Geode’s analog output with a reference recording.

| DOS sound system | Confirmed audio output |
| --- | --- |
| Miles | MPU-401 UART music rendered through a SoundFont bank |
| Tracker, S3M modules | Samples mixed to Sound Blaster PCM |
| Tracker, XM modules | Samples mixed to Sound Blaster PCM |
| WAV player | Sound Blaster 16 digital audio |
| SCI | FM synthesis |
| SCI | General MIDI rendered through a SoundFont bank |
| HMI Sound Operating System | OPL2 synthesis |
| HMI Sound Operating System | General MIDI rendered through a SoundFont bank |
| SCUMM | AdLib music |

## Compatibility and workarounds

| Symptom | Cause and action |
| --- | --- |
| Sample pitch is slightly high in a legacy Sound Blaster mode | The default time-constant correction rounds common sample rates. With `/FIXTC=0`, the driver follows the programmed divisor. Measured pitch error on high-speed commands 90h and 91h and on SB Pro stereo fell from about +1.4% to within 0.02% in the test tones. |
| A WAV player returns DOS code `74h` after playback | The analog output matched the expected WAV audio even though the player returned this code. |
| The CS5536 reports `EOP_ERR` during HMI OPL2 playback | The status flag appeared under both XMS policies. The recorded passages matched across settings, and the DMA pointer remained valid. |
| A real-mode program starts but has no sound | Its port accesses need a real-mode trap provider. With Jemm, load QPIEMU through JLOAD before starting SBGEODE and enable real-mode support. |
| A protected-mode application is silent while real-mode audio works | An extender that chooses VCPI can bypass HDPMI32i. If the extender also supports DPMI, load HDPMI32i and start the driver with `SBGEODE.EXE /NOVCPI`. A VCPI-only extender cannot use that DPMI route. |
| Music plays but digitized speech or effects are missing or garbled | A program using a 24-bit DMA address can address the wrong memory when its XMS audio buffer extends above 16 MiB. Reboot and use `SBGEODE.EXE /XMSLOW` on first installation. |
| A legacy XMS allocation fails with error A0h | With `/XMSLOW`, the entire allocation must fit in a single block below 16 MiB. An 8 MiB request failed in this configuration and succeeded after a clean boot with the default policy. XMS 3.0 requests pass through to the manager. |
| An application stops before audio with `/XMSLOW` and reports only “insufficient memory” | Try the default XMS policy on a clean boot. One application reached its intro and generated 25 sound IRQs with the default policy, then stopped before the first sound IRQ with `/XMSLOW`. The largest free conventional block was 594,672 bytes in both cases. The application did not identify the allocation that failed. |
| An SCI application stops before FM playback and asks for more free memory | FM playback was confirmed on 1.0 under the normal configuration. With the largest free conventional block reduced to 594,704 bytes, the same application reported a shortfall of roughly 4 KiB under both XMS policies. Check the largest free block with `MEM /C` and free conventional memory before retrying. |


## Package and source

The DOS download includes `SBGEODE.EXE`, a plain-text quick start, the user guide, corresponding source and license notices. The release asset `SHA256SUMS` verifies the ZIP. Inside the ZIP, `SHA256.TXT` verifies the individual files. The SHA-256 of the tested DOS executable is `97df10a4661b36ec2347fcc2c1f4e9d7c592177dde708ff06089febbb1b384f4`.

To build it on Linux, extract `SOURCE.TAR` and enter `src/`, or open `src/` in the release checkout. With the DJGPP GCC 12.2.0 cross-toolchain and NASM 2.16.03 on your path, run `make -j8 VERSION=1.0`. The result is `output/sbemu.exe`, distributed as `SBGEODE.EXE`.

## Contributors

OrdinaryLunatic developed SBGEODE with assistance from Claude, ChatGPT and KIMI.

SBGEODE derives from crazii’s SBEMU, with sound drivers from MPXPlay, OPL emulation from DOSBox and SoundFont support derived from Cacodemon345’s work. Upstream also credits jiyunomegami for Linux driver ports and andersrodrig for the PC speaker implementation. The corresponding source retains the component licenses and copyright notices. [License](COPYING) · [SBEMU](https://github.com/crazii/SBEMU) · [MPXPlay](https://mpxplay.sourceforge.net/) · [SoundFont implementation](https://github.com/Cacodemon345/VSBHDASF)

## Interesting facts

These figures describe the recording archive at the time SBGEODE 1.0 was accepted on November 26, 2026.

- **78 hours, 42 minutes and 38 seconds of audio captured throughout the project.** These recordings preserve what actually came out of the PC. We used them to check whether a voice line played, whether samples ran at the right speed, and whether a change introduced clicks or gaps. We compared matched passages from repeated runs, and listened to selected A/B pairs ourselves. A quiet passage on a graph could be an intentional rest in the music; hearing the same passage in the reference recording helped settle the question.
- **2,165 distinct hardware recordings, totaling 54.40 GB.** Playing time includes the quiet parts of each take. Identical PCM copies count once. Listening excerpts, emulator references and audio supplied with software are excluded from the totals.
- **2,141 timed hardware test runs took 79 hours and 4 minutes in total.** Test runtime and audio recording time overlap.
- **About 67 days** elapsed between the first build that played Sound Blaster audio on a physical Geode LX and the final 1.0 release.

## На русском

**SBGEODE — резидентный эмулятор Sound Blaster для DOS на AMD Geode LX с контроллером CS5536.** Когда игра программирует DSP Sound Blaster и канал ISA DMA, драйвер перехватывает запись в порты, читает сэмплы из буфера игры и вызывает виртуальное прерывание Sound Blaster на границе блока DSP. Записи в регистры OPL управляют FM-синтезатором. Микшер объединяет цифровой звук PCM, FM и, при включённом MIDI-синтезаторе, звук из банка SoundFont. Готовый поток поступает в контроллер воспроизведения CS5536 AC’97.

На Geode прерывание звукового контроллера может прийти, пока DOS переключает видеорежим. SBGEODE использует кольцевой DMA-буфер на четыре периода и по мере воспроизведения перемещает единственную метку конца периода. Благодаря запасу данных контроллер продолжает воспроизведение при краткой задержке обслуживания прерывания. Со стороны игры виртуальное звуковое прерывание может получить обработчик реального режима, даже если сама игра работает под 32-разрядным расширителем DOS.

Звук проверен на аналоговом выходе **AMD Geode LX 500 МГц с контроллером CS5536 и кодеком Realtek ALC203**. В записях подтверждены FM-музыка с цифровыми эффектами, речь из XMS, General MIDI через банки SoundFont и воспроизведение трекерных модулей через эмулируемый DSP. [Результаты по звуковым системам](#проверка-звуковых-систем-на-реальном-железе)

[Скачать SBGEODE 1.0 для DOS](https://github.com/OrdinaryLunatic/SBGEODE/releases/download/v1.0/SBGEODE-1.0.zip) · [English version](#sbgeode)

### Требования и первый запуск

**Основная система для SBGEODE — MS-DOS 6.22.** Работа также подтверждена под FreeDOS и MS-DOS 7.1. При аппаратной проверке под MS-DOS 6.22 использовались JemmEx 5.87, QPIEMU и HDPMI32i.

| Режим приложения | Что требуется |
| --- | --- |
| Реальный режим DOS | Для перехвата обращений к портам нужен JemmEx или Jemm386 с загруженным QPIEMU либо совместимая версия QEMM. JemmEx предоставляет и XMS, и EMS. Для Jemm386 сначала загрузите отдельный менеджер XMS, например HIMEMX. |
| 32-разрядный защищённый режим DOS | Обращения к портам перехватывает HDPMI32i. Для этого расширитель DOS должен запускать приложение через DPMI. |
| Оба режима | Настройте загрузку менеджера памяти реального режима при старте DOS и загрузите HDPMI32i до SBGEODE. |

После загрузки JemmEx или Jemm386 убедитесь, что JLOAD, HDPMI32i и SBGEODE доступны через `PATH`. Выполните следующие команды один раз после загрузки DOS из каталога с `QPIEMU.DLL` либо укажите JLOAD полный путь к DLL.

```dos
JLOAD.EXE QPIEMU.DLL
HDPMI32I.EXE -r -x
SET BLASTER=A220 I5 D1 H5 T6
SBGEODE.EXE /T=6
```

| Звуковой интерфейс | Настройки приложения и драйвера |
| --- | --- |
| Цифровые эффекты и речь | Выберите **Sound Blaster 16**, адрес **220h**, IRQ **5**, 8-битный DMA **1**, 16-битный DMA **5**. Если из подходящих устройств старая программа предлагает только **Sound Blaster Pro**, выберите его и запустите SBGEODE с `/T=4`. |
| FM-музыка | Выберите **AdLib** или **OPL** на порту **388h**. |
| General MIDI | Выберите **General MIDI / MPU-401** на порту **330h**. Подготовьте банк SoundFont 2 и запустите драйвер командой `SBGEODE.EXE /T=6 /VMPU=1 /VMSF=C:\MIDI\BANK.SF2` для синтеза с полифонией до 64 голосов. Без `/VMSF` драйвер ищет файл `SBEMUSF.SF2`. |

**Начните со штатных настроек XMS.** Речь и эффекты могут воспроизводиться напрямую из XMS, если буфер сэмпла целиком расположен ниже границы 16 МиБ. Если FM-музыка играет, а речь или эффекты пропадают либо искажаются, сначала проверьте выбранное в игре цифровое устройство, IRQ и DMA. Если параметры совпадают, перезагрузите DOS и при первом запуске драйвера выполните `SBGEODE.EXE /T=6 /XMSLOW`. Ключ меняет область, из которой старые XMS-клиенты могут выделять память. Одной программе это помогает, а другой может не хватить свободного блока ниже 16 МиБ.

Повторный запуск `SBGEODE.EXE` покажет действующие настройки резидентного драйвера.

Загрузить компоненты можно со страниц проектов. [Jemm и QPIEMU](https://github.com/Baron-von-Riedesel/Jemm) · [HDPMI32i](https://github.com/crazii/HX)

### Ключи командной строки

<details>
<summary>Настройки Sound Blaster, микшера, MIDI и совместимости</summary>

| Ключ | Назначение |
| --- | --- |
| `/A`, `/I`, `/D`, `/H` | Базовый адрес эмулируемой карты (220h или 240h), IRQ (5, 7 или 9), 8-битный DMA (0, 1 или 3) и 16-битный DMA (5, 6 или 7). Эти параметры также можно задать через `BLASTER`. |
| `/T` | Тип Sound Blaster. Значения 1–3 выбирают ранние модели SB, 4–5 — модели SB Pro, 6 — SB16. |
| `/RM`, `/PM` | Независимо включают и отключают поддержку реального и защищённого режимов. Например, `/RM=0` оставляет только защищённый режим. |
| `/NOVCPI` | Отключает VCPI в Jemm после загрузки драйвера. Полезен для расширителей DOS, которые при наличии обоих интерфейсов выбирают VCPI перед DPMI. По умолчанию сохраняется текущая настройка Jemm. |
| `/XMSLOW` | Ограничивает новые выделения памяти через старые функции XMS блоками, целиком расположенными ниже 16 МиБ. По умолчанию выключен. Задаётся при первом запуске драйвера после загрузки DOS. |
| `/VOL`, `/SFXVOL` | Общая громкость и громкость цифровых эффектов, обе в диапазоне 0–100. Уменьшите `/SFXVOL`, чтобы приглушить эффекты, сохранив уровень FM-музыки. |
| `/OPL`, `/FMQ` | Первый ключ включает или отключает эмуляцию FM. Второй задаёт частоту синтеза: 0 — автоматический выбор, 1 — половинная частота, 2 — полная. |
| `/K`, `/FILT`, `/SWAP` | Внутренняя частота дискретизации в герцах, фильтрация PCM и порядок стереоканалов. Уровни фильтрации — от 0 (линейная интерполяция) до 2 (sinc-интерполяция). Значение `/SWAP=1` меняет местами каналы PCM, `/SWAP=2` — каналы всего выходного сигнала. |
| `/FIXTC`, `/VIRQ` | Первый ключ корректирует постоянные времени DSP для распространённых частот дискретизации. Второй выбирает маршрут виртуального звукового прерывания. При `/VIRQ=1` программа защищённого режима может вызвать звуковой обработчик через вектор реального режима. При `/VIRQ=0` используется вектор защищённого режима. |
| `/P`, `/MCOM` | Адрес MPU-401 UART и выбор внешнего MIDI-выхода. Адрес по умолчанию — 330h. Значения `/MCOM` от 1 до 4 направляют MIDI в последовательные порты COM1–COM4 соответственно. Значение 9 выбирает аппаратный MPU. |
| `/VMPU`, `/VMSF`, `/MSHARE` | Настройки программного MIDI-синтезатора. `/VMSF` задаёт банк SoundFont 2. При `/VMPU=0` синтез отключён, `/VMPU=1` выбирает 64 голоса, а значения 32–256 задают предел полифонии напрямую. `/MSHARE` выделяет на MIDI от 4 до 14 шестнадцатых долей периода воспроизведения (по умолчанию 10). Меньшая доля оставляет больше процессорного времени приложению, но при высокой нагрузке ноты могут обрываться. |
| `/?` | Выводит список ключей и допустимых значений. |

</details>

### Отличия SBGEODE от SBEMU

В основе SBGEODE лежит SBEMU от crazii. В описании драйвера CS5535/CS5536, предложенного для включения в SBEMU, подтверждена успешная сборка с DJGPP. Аппаратная проверка в плане испытаний отмечена как предстоящая. [План проверки драйвера SBEMU](https://github.com/crazii/SBEMU/pull/196)

Драйвер воспроизведения **CS5536 AC’97 написан для SBGEODE с нуля**, независимо от драйвера CS5535/CS5536 из SBEMU и без использования его кода. На стороне DOS в SBGEODE 1.0 изменены доставка звуковых прерываний, декодирование сэмплов и размещение буферов XMS. На стороне контроллера — организация передач DMA и управление аналоговым микшером. Результаты ниже получены под MS-DOS 6.22 на физическом **AMD Geode LX 500 МГц с контроллером CS5536 и кодеком Realtek ALC203 AC’97**. [SBEMU](https://github.com/crazii/SBEMU) · [Драйвер CS5535/CS5536 в SBEMU](https://github.com/crazii/SBEMU/blob/43722f7318212dc50bac2df852ff16a3b3097067/mpxplay/au_cards/sc_cs5535.c)

| Область | Что реализовано в SBGEODE | Как это сказывается на работе |
| --- | --- | --- |
| Воспроизведение через CS5536 | Контроллер продолжает читать подготовленные данные, пока процессор ненадолго занят. Драйвер учитывает граничный случай при переходе контроллера к следующему буферу. | Короткая задержка обслуживания может пройти без слышимого провала. При длительной задержке запас данных всё же исчерпывается. |
| Цифровые эффекты | PCM-микшер сохраняет фазу дискретизации при переходе между блоками, поддерживает восьмиотводную sinc-интерполяцию и удерживает последний уровень ЦАП после завершения передачи. Громкость эффектов регулируется отдельно. | Короткий эффект может закончиться без резкого скачка сигнала к нулю. Эффекты можно приглушить, сохранив громкость FM-музыки. |
| Совместимость при воспроизведении сэмплов | Драйвер обрабатывает команду DSP SB16 `42h` для задания частоты дискретизации и сохраняет состояние декодера между блоками Creative ADPCM. | Программы, использующие `42h`, могут воспроизводить сэмплы с заданной высотой тона и скоростью. Декодирование сжатого эффекта продолжается со следующего блока без перезапуска. |
| FM и General MIDI | FM-синтез работает на половинной или полной частоте. Цикл обработки голосов SoundFont оптимизирован для Geode. Долю процессорного времени, выделяемую MIDI, можно задать вручную. | Можно выбрать более насыщенное звучание или оставить больше времени приложению. В плотных аранжировках отдельные ноты всё ещё могут обрываться. |
| Звуковые драйверы DOS | Виртуальное звуковое прерывание программы защищённого режима может получить резидентный обработчик реального режима. В Jemm можно отключить VCPI после загрузки, чтобы совместимый расширитель выбрал DPMI. | Поддерживаются программы защищённого режима со звуковым драйвером реального режима, а также расширители, которые иначе обходили бы перехват звуковых портов через DPMI. |
| Звуковые буферы в XMS | Ключ `/XMSLOW` размещает новые блоки, запрошенные через старые функции XMS, целиком ниже 16 МиБ — границы адресации ISA DMA. | Речь и эффекты из XMS могут звучать одновременно с FM-музыкой, даже если в машине гораздо больше 16 МиБ памяти. |

### Проверка звуковых систем на реальном железе

Измерения выполнены под MS-DOS 6.22 на Geode LX / CS5536 с файлом `SBGEODE.EXE` размером 599 552 байта из релиза **1.0**.

| Звуковая система или режим вывода | Результат с релизным файлом 1.0 |
| --- | --- |
| DMX · Sound Blaster PCM и OPL-музыка | Цифровые эффекты слышны при старте, после перехода на следующий уровень и при повторном запуске после штатного выхода в DOS без перезагрузки компьютера. |
| Sound Blaster PCM в реальном режиме | Каждый отдельно вызванный эффект удалось опознать в записи аналогового выхода. |
| Цифровой звук в защищённом режиме | Записанные фрагменты совпали с эталонной фонограммой. |
| Цифровые эффекты из XMS | С ключом `/XMSLOW` адрес DMA указывал на исходный сэмпл ниже 16 МиБ, и эффект был слышен. |
| Miles · оцифрованная речь из XMS | При штатных настройках XMS речь воспроизводилась, когда адрес DMA указывал на исходный сэмпл ниже 16 МиБ. Реплика опознана в записи аналогового выхода. |
| Creative ADPCM · продолжение блока | Записанный тон сохранялся при переходе от опорного блока к блоку продолжения. |
| Стерео PCM · распределение каналов | Последовательно прозвучали левый канал, правый канал и оба вместе. Измеренные частоты тонов — 997 и 1 499 Гц. |
| Регистры таймеров OPL | Флаги таймеров, которые читала DOS-программа, соответствовали ожидаемым интервалам 80 и 320 мкс. |
| Sound Blaster DSP · режимы передачи сэмплов | Тоны от передач удалось опознать в записи. С ключом `/FIXTC=0` измеренные частоты совпали с заданными. |
| Sound Blaster 16 · команда DSP `42h` | При замене команды `41h` на `42h` для задания частоты дискретизации измеренная высота тона с `/FIXTC=0` сохранилась. |
| HMI OPL2 · штатный режим XMS и `/XMSLOW` | Фрагменты записи совпали при обеих настройках XMS. |

**Воспроизведение в DOS-приложениях с SBGEODE 1.0.** Каждый из перечисленных режимов был выбран в самом приложении. Воспроизведение подтверждено сравнением записи аналогового выхода Geode с эталонной записью.

| Звуковая система DOS | Подтверждённый вывод |
| --- | --- |
| Miles | Музыка через MPU-401 UART с синтезом по банку SoundFont |
| Трекер, модули S3M | Сведение сэмплов с выводом через Sound Blaster PCM |
| Трекер, модули XM | Сведение сэмплов с выводом через Sound Blaster PCM |
| Проигрыватель WAV | Цифровой звук через Sound Blaster 16 |
| SCI | FM-синтез |
| SCI | General MIDI с синтезом по банку SoundFont |
| HMI Sound Operating System | Синтез OPL2 |
| HMI Sound Operating System | General MIDI с синтезом по банку SoundFont |
| SCUMM | Музыка AdLib |

### Совместимость и способы решения проблем

| Проявление | Причина и что можно сделать |
| --- | --- |
| В старом режиме Sound Blaster сэмпл звучит чуть выше положенного | В штатном режиме коррекция постоянной времени округляет распространённые частоты дискретизации. С `/FIXTC=0` драйвер использует запрограммированный делитель. Для команд высокоскоростного режима 90h и 91h и стереорежима SB Pro отклонение высоты тестового тона уменьшилось примерно с +1,4 % до 0,02 % или меньше. |
| Проигрыватель WAV после воспроизведения возвращает в DOS код `74h` | При таком коде возврата запись аналогового выхода совпала с ожидаемым звуком WAV. |
| При воспроизведении HMI OPL2 контроллер CS5536 выставляет `EOP_ERR` | Флаг появлялся при обеих настройках XMS. Записанные фрагменты совпали, указатель DMA оставался корректным. |
| Программа реального режима запускается без звука | Нужен перехват её обращений к портам. Если используется Jemm, загрузите QPIEMU через JLOAD до запуска SBGEODE и включите поддержку реального режима. |
| Звук есть в реальном режиме, а программа защищённого режима молчит | Расширитель DOS может выбрать VCPI и обойти HDPMI32i. Если расширитель поддерживает DPMI, загрузите HDPMI32i и запустите `SBGEODE.EXE /NOVCPI`. Для расширителя, поддерживающего только VCPI, этот способ недоступен. |
| Музыка играет, но оцифрованная речь или эффекты пропадают либо искажаются | Если звуковой буфер XMS выходит за границу 16 МиБ, 24-битный адрес DMA может указывать на другую область памяти. Перезагрузите DOS и при первой установке драйвера выполните `SBGEODE.EXE /XMSLOW`. |
| Старая функция выделения XMS возвращает ошибку A0h | При `/XMSLOW` весь запрошенный объём должен помещаться в один блок ниже 16 МиБ. Запрос на 8 МиБ в этой конфигурации завершился отказом, а после чистой загрузки со штатными настройками — успешно. Запросы XMS 3.0 передаются менеджеру памяти напрямую. |
| С `/XMSLOW` приложение до начала воспроизведения сообщает только «insufficient memory» | Попробуйте штатные настройки XMS после чистой загрузки. При них одно приложение дошло до вступления и сгенерировало 25 звуковых прерываний. С `/XMSLOW` оно остановилось до первого прерывания. В обоих случаях крупнейший свободный блок обычной памяти составлял 594 672 байта. Приложение не указало, какой запрос памяти завершился отказом. |
| Приложение SCI останавливается до начала FM-музыки и просит освободить память | В штатной конфигурации воспроизведение FM подтверждено на версии 1.0. Когда крупнейший свободный блок обычной памяти уменьшили до 594 704 байт, приложение сообщило о нехватке примерно 4 КиБ при обеих настройках XMS. Проверьте размер крупнейшего свободного блока командой `MEM /C` и освободите обычную память перед повторным запуском. |

### Комплект поставки и исходный код

В архиве для DOS находятся `SBGEODE.EXE`, краткая инструкция в обычном текстовом формате, руководство пользователя, соответствующие исходники и тексты лицензий. Файл `SHA256SUMS` на странице релиза позволяет проверить ZIP целиком, а `SHA256.TXT` внутри архива — каждый файл по отдельности. SHA-256 проверенного исполняемого файла DOS — `97df10a4661b36ec2347fcc2c1f4e9d7c592177dde708ff06089febbb1b384f4`.

Для сборки под Linux распакуйте `SOURCE.TAR` и перейдите в `src/` либо откройте `src/` в рабочей копии релиза. Добавьте кросс-компилятор DJGPP GCC 12.2.0 и NASM 2.16.03 в `PATH` и выполните `make -j8 VERSION=1.0`. Результат сборки — `output/sbemu.exe`, в релизе он называется `SBGEODE.EXE`.

### Участники проекта

OrdinaryLunatic разработал SBGEODE при участии Claude, ChatGPT и KIMI.

В основе проекта — SBEMU от crazii, звуковые драйверы из MPXPlay, эмуляция OPL из DOSBox и поддержка SoundFont на основе работы Cacodemon345. В SBEMU также отмечен вклад jiyunomegami в перенос драйверов из Linux и andersrodrig в реализацию PC Speaker. В исходниках сохранены лицензии компонентов и уведомления об авторских правах. [Лицензия](COPYING) · [SBEMU](https://github.com/crazii/SBEMU) · [MPXPlay](https://mpxplay.sourceforge.net/) · [Реализация SoundFont](https://github.com/Cacodemon345/VSBHDASF)

### Интересные факты

Данные приведены на момент приёмки SBGEODE 1.0 — 26 ноября 2026 года.

- **78 часов 42 минуты 38 секунд звука записано за время работы над проектом.** Записи сохранили то, что действительно звучало на выходе компьютера. По ним проверяли, прозвучала ли реплика, с нужной ли скоростью воспроизводились сэмплы, появились ли после изменений щелчки или провалы. Соответствующие фрагменты повторных прогонов сравнивали между собой, а отдельные пары лично прослушивали. Тихий участок на графике мог оказаться паузой в самой музыке. Разобраться помогало прослушивание того же фрагмента эталонной записи.
- **2 165 уникальных записей с реального железа общим объёмом 54,40 ГБ.** Длительность включает тихие участки. Идентичные копии PCM учитывались один раз. Подготовленные для прослушивания нарезки, эталоны из эмуляторов и звук из комплектов программ в эти итоги не входят.
- **2 141 аппаратный прогон с замером времени занял в сумме 79 часов 4 минуты.** Время прогонов и длительность аудиозаписей частично перекрываются.
- **Около 67 дней** прошло от первой сборки, воспроизведшей звук Sound Blaster на физическом Geode LX, до финального релиза 1.0.
