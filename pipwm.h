/*
 * MIT License
 * 
 * Copyright (c) 2025 ksjo
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef PIPWM_H
#define PIPWM_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#define BUS_BASE_REG 0x7E000000

#define BCM2835_BASE_REG 0x20000000 // Pi 1, Pi Zero
#define BCM2837_BASE_REG 0x3F000000 // Pi 2, Pi 3, Pi Zero 2 W
#define BCM2711_BASE_REG 0xFE000000 // Pi 4

#define CLOCK_KHZ_250000 250000     // Pi 0-3
#define CLOCK_KHZ_375000 375000     // Pi 4

#define PHYS_DMA_BASE   (phys_base_reg + 0x007000)
#define PHYS_CLOCK_BASE (phys_base_reg + 0x101000)
#define PHYS_GPIO_BASE  (phys_base_reg + 0x200000)
#define PHYS_PWM_BASE   (phys_base_reg + 0x20C000)
#define BUS_GPIO_BASE   (BUS_BASE_REG + 0x200000)
#define BUS_PWM_BASE    (BUS_BASE_REG + 0x20C000)

#define PAGE_SIZE 0x1000

#define MILLION 1000000

#define REG_IDX(x) (x / 4)

#define PROC_DEVICE_TREE_PATH "/proc/device-tree/model"
#define SYS_DEVICE_TREE_PATH "/sys/firmware/devicetree/base/model"

#define MODEL_PI_1        "Raspberry Pi"
#define MODEL_PI_4        "Raspberry Pi 4"
#define MODEL_PI_3        "Raspberry Pi 3"
#define MODEL_PI_2        "Raspberry Pi 2"
#define MODEL_PI_ZERO_2_W "Raspberry Pi Zero 2 W"
#define MODEL_PI_ZERO     "Raspberry Pi Zero"

#define MIN_GPIO_NUM 1
#define MAX_GPIO_NUM 32

#define MIN_FREQ  1
#define MAX_FREQ  1000000

#define MIN_RANGE 1
#define MAX_RANGE 10000

#define DEVICE_PATH_NAME "/dev/dma_buffer"
#define DMA_BUF_IOCTL_GET_PHYS_ADDR _IOR (0xF0, 1, uint32_t)
#define DMA_BUF_SIZE PAGE_SIZE * 256 // 1MB

#define WAVE_NUM MAX_GPIO_NUM
#define CB_MAX 10000

#define DMA_CHAN        5
#define DMA_CHAN_OFFSET 0x100

// DMA Registers
#define DMA_CS        0x00
#define DMA_CB_AD     0x04
#define DMA_TI        0x08
#define DMA_SRCE_AD   0x0C
#define DMA_DEST_AD   0x10
#define DMA_TXFR_LEN  0x14
#define DMA_STRIDE    0x18
#define DMA_NEXT_CB   0x1C
#define DMA_DEBUG     0x20
#define DMA_ENABLE    0xFF0
// Clock Registers
#define CM_GP0CTL   0x70
#define CM_GP0DIV   0x74
#define CM_GP1CTL   0x78
#define CM_GP1DIV   0x7C
#define CM_GP2CTL   0x80
#define CM_GP2DIV   0x84
#define CM_PCMCTL   0x98
#define CM_PCMDIV   0x9C
#define CM_PWMCTL   0xA0
#define CM_PWMDIV   0xA4
// GPIO Registers
#define GPFSEL0     0x00
#define GPFSEL1     0x04
#define GPFSEL2     0x08
#define GPFSEL3     0x0C
#define GPFSEL4     0x10
#define GPFSEL5     0x14
#define GPSET0      0x1C
#define GPSET1      0x20
#define GPCLR0      0x28
#define GPCLR1      0x2C
#define GPLEV0      0x34
#define GPLEV1      0x38
#define GPEDS0      0x40
#define GPEDS1      0x44
#define GPREN0      0x4C
#define GPREN1      0x50
#define GPFEN0      0x58
#define GPFEN1      0x5C
#define GPHEN0      0x64
#define GPHEN1      0x68
#define GPLEN0      0x70
#define GPLEN1      0x74
#define GPAREN0     0x7C
#define GPAREN1     0x80
#define GPAFEN0     0x88
#define GPAFEN1     0x8C
#define GPPUD       0x94
#define GPPUDCLK0   0x98
#define GPPUDCLK1   0x9C
// PWM Registers
#define PWM_CTL     0x00
#define PWM_STA     0x04
#define PWM_DMAC    0x08
#define PWM_RNG1    0x10
#define PWM_DAT1    0x14
#define PWM_FIF1    0x18
#define PWM_RNG2    0x20
#define PWM_DAT2    0x24

#define DMA_PWM_DREQ        5
#define DMA_NO_WIDE_BURSTS  (1 << 26)
#define DMA_WAIT_RESP       (1 << 3)
#define DMA_CB_DEST_INC     (1<<4)
#define DMA_CB_SRC_INC      (1<<8)
#define DELAY_TI            DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP
#define PWM_TI              ((1 << 6) | (DMA_PWM_DREQ << 16) | DELAY_TI)

#define PWM_ENAB         1
#define PWM_CTL_RPTL1    (1 << 2)
#define PWM_CTL_USEF1    (1 << 5)
#define PWM_CTL_USEF2    (1 << 13)
#define PWM_DMAC_ENAB    (1 << 31)
#define PWM_DMAC_THRSHLD ((15<<8) | (15<<0))

#define CM_PASSWORD (0x5A << 24)

typedef enum {
    RP_NONE = 0,
    RP_UNKNOWN,
    RP_PI_1,
    RP_PI_2,
    RP_PI_3,
    RP_PI_4,
    RP_PI_ZERO,
    RP_PI_ZERO_2_W
} PiVersion;

typedef struct {
    uint32_t ti;
    uint32_t srce_ad;
    uint32_t dest_ad;
    uint32_t txfr_len;
    uint32_t stride;
    uint32_t next_cb;
    uint32_t debug;
    uint32_t unused;
} DMA_CB __attribute__ ((aligned(32)));

typedef struct {
    uint32_t gpio;
    int freq;
    int range;
    int period;
    int range_value;
    int period_value;
    int spot_num;
    int *set_spots;
    int *clr_spots;
    bool active;
    bool always_on;
    bool always_off;
    bool need_update;
} Wave;

typedef struct {
    uint32_t gpio;
    uint32_t gpio_reg;
    int spot;
} WaveformEvent;

PiVersion detect_version(void);
PiVersion get_raspberry_pi_version(void);
void    set_base_addresses(void);
int     init_pipwm(void);
void    exit_pipwm(void);
int     set_pwm_gpio(int gpio);
int     set_pwm_frequency(int gpio, int frequency);
int     set_pwm_range(int gpio, int range);
int     set_pwm_value(int gpio, int value);
int     apply_pwm(void);

void *get_dmabuf(void);
int init_dmabuf(void);
void exit_dmabuf(void);
uint32_t convert_to_busaddr(void *ptr);

int map_all_registers(void);
void unmap_all_registers(void);
uint32_t get_dma_cb_ad(void);
void enable_dma(void);
void stop_dma(void);
void start_dma(DMA_CB *dma_cb);
void set_gpio_mode_input(int gpio_num);
void set_gpio_mode_output(int gpio_num);
void set_gpio(int gpio_num);
void clear_gpio(int gpio_num);
void clear_gpios(uint32_t reg);
void init_pwm(void);
void start_pwm(void);
void stop_pwm(void);

void *get_gpio_buff_front(void);
void *get_gpio_buff_rear(void);
void *get_dma_cb_buff_front(void);
void *get_dma_cb_buff_rear(void);
void *get_dummy_buff(void);
int gcd(int a, int b);
int lcm(int a, int b);
int convert_freq_to_period(int freq);
void init_waves(void);
Wave *get_wave_by_gpio(int gpio);
int cal_waveform_period(Wave *t_waves);
void set_wave_gpio(int gpio);
int set_frequency(int gpio, int freq);
int set_range(int gpio, int range);
int set_wave_value(int gpio, int value);
int apply_waves(void);
bool is_staying_previous_buffer(void);
int update_asynced_waveform(void);
int update_synced_waveform(int freq);
void sort_events(WaveformEvent events[], int n);
void print_dma_buff(int len);
void start_waveform(DMA_CB *dma_cb);
void stop_waveform(void);
void free_all_spots(void);
void clear_all_waves(void);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Map a physical address into user space.
 * @param addr Physical address (page-aligned)
 * @param size Length in bytes (multiple of page size)
 * @return Pointer to mapped memory or NULL on failure.
 */
static inline void* map_segment(uint32_t addr, size_t size) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("/dev/mem open error");
        return NULL;
    }
    void* ptr = mmap(NULL, size, PROT_READ | PROT_WRITE,
                     MAP_SHARED, fd, addr);
    close(fd);
    return (ptr == MAP_FAILED ? NULL : ptr);
}

#ifdef __cplusplus
}
#endif

#endif
