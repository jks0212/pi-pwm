# pi-pwm

## Overview
On a Xenomai-patched Raspberry Pi, the Mailbox interface to VideoCore can be problematic. This project allocates uncached memory from the Linux kernel and uses DMA to generate PWM signals directly from that buffer.

## Features
- **1 µs resolution**  
- **32-bit support** (64-bit not yet supported)  
- **Up to 1 MHz frequency**  

## Dependency
- [pi-dma-buffer](https://github.com/jks0212/pi-dma-buffer)  
  A Linux kernel module that provides uncached memory allocation for DMA.

## Tips
- You can configure different frequencies on each GPIO and run them simultaneously.  
- Memory use is more efficient when the frequencies you choose are close multiples of each other. For example, pairing 1 Hz with 5 Hz or 500 Hz with 1 kHz uses less buffer than pairing 1 Hz with 1 kHz.  
- When all frequencies are identical, memory usage is the most efficient and jitter is minimized. Using different frequencies may introduce up to ~1 µs of jitter.

## Build and Run the Example
1. Build the example:
   ```bash
   make
   ```

2. Run the example:
   ```bash
   sudo ./example
   ```
