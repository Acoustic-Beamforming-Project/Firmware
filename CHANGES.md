# Firmware Optimization Log

This document lists the changes made to the STM32F401 firmware to achieve 2000 pkt/s Ethernet throughput at a 48kHz ADC sampling rate.

## Core Performance Fixes

### 1. Zero-Copy DMA Data Path
- **Change:** Switched SPI2 and SPI3 (ADC) to 8-bit mode.
- **Impact:** Allowed DMA to write data directly into the Ethernet packet buffer in big-endian format.
- **Benefit:** Eliminated multiple redundant `memcpy` operations and manual byte-swapping logic, saving significant CPU cycles per sample.

### 2. Triple Buffering for ADC Batches
- **Change:** Implemented a 3-stage batch buffer system (`AD7606_Batch _batches[3]`).
- **Impact:** Decoupled ADC acquisition from Ethernet transmission.
- **Benefit:** Acquisition can fill one buffer while the W5500 is still transmitting the previous one, preventing packet drops due to SPI bus contention.

### 3. Non-Blocking Ethernet Driver
- **Change:** Removed the blocking `Sn_IR_SENDOK` polling loop in `wiz5500.c`.
- **Impact:** The driver now uses a single DMA phase for the payload and handles the `Sn_CR_SEND` command asynchronously in the DMA completion callback.
- **Benefit:** Frees the CPU to return to acquisition tasks immediately after the Ethernet DMA starts.

### 4. Adjusted Batch Size for 2000 pkt/s
- **Change:** Set `AD7606_BATCH_FRAMES` to 24.
- **Impact:** At 48,000 Hz sample rate, 48,000 / 24 = 2000 batches per second.
- **Benefit:** Directly meets the user's throughput requirement of 2000 pkt/s.

### 5. Main Loop Streamlining
- **Change:** Removed the millivolt conversion and reconstruction loop in `main.c`.
- **Impact:** Drastically reduced CPU load in the background thread.
- **Benefit:** Ensures the main loop can process 2000 `AD7606_IsBatchReady()` checks per second without lag.

## Structural Improvements
- Cleaned up diagnostic variables and commented-out `printf` calls in ISRs.
- Improved peripheral flushing in `ad7606.c` to prevent DMA stalls.
- Updated `AD7606_Init` to pre-fill packet headers (sync bytes `0xDEADBEEF`), moving overhead out of the hot path.

## Unchanged Logic (Preservation)
- **Sample Rate:** Remained at exactly 48,000 Hz (TIM2 ARR=1332).
- **ADC Triggering:** Sequential X-bank then Y-bank conversion sequence preserved to maintain phase alignment and datasheet timing requirements.
- **Hardware Config:** GPIO pin assignments and clock speeds remain within original design specs.
