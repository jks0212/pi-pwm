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
 
#include "pipwm.h"

bool initialized = false;

uint32_t phys_base_reg;
int clock_khz;
// DMA Buffer
void *dma_buf;
uint32_t phys_addr;
// Peripherals
volatile uint32_t *dma_reg;
volatile uint32_t *pwm_reg;
volatile uint32_t *gpio_reg;
volatile uint32_t *clock_reg;
// Waveform
BufferIdx buffer_idx = BUF_0;
bool waveform_started = false;
int waveform_period = 0;
int curr_cb_num = 0;
int prev_cb_num = 0;
DMA_CB *prev_dma_cbs;
DMA_CB *prev_last_cb;
Wave waves[WAVE_NUM];

PiVersion detect_version(void) {
    FILE* fp = fopen(PROC_DEVICE_TREE_PATH, "r");
    if (!fp) fp = fopen(SYS_DEVICE_TREE_PATH, "r");
    if (!fp) return RP_NONE;

    char model[128] = {0};
    fread(model, 1, sizeof(model) - 1, fp);
    fclose(fp);

    if (strstr(model, MODEL_PI_4))        return RP_PI_4;
    if (strstr(model, MODEL_PI_3))        return RP_PI_3;
    if (strstr(model, MODEL_PI_2))        return RP_PI_2;
    if (strstr(model, MODEL_PI_ZERO_2_W)) return RP_PI_ZERO_2_W;
    if (strstr(model, MODEL_PI_ZERO))     return RP_PI_ZERO;
    if (strstr(model, MODEL_PI_1))        return RP_PI_1;
    return RP_UNKNOWN;
}

PiVersion get_raspberry_pi_version(void) {
    return detect_version();
}

int init_pipwm(void) {
    if(!initialized){
        set_base_addresses();
        if(map_all_registers() < 0) {
            return -1;
        }

        if(init_dmabuf() < 0){
            return -1;
        }

        enable_dma();
        init_waves();

        initialized = true;
    }
    return 0;
}

void exit_pipwm(void) {
    if(initialized){
        stop_pwm();
        stop_dma();
        clear_all_waves();
        free_all_spots();
        unmap_all_registers();
        exit_dmabuf();
    }
}

void set_base_addresses(void) {
    struct {
        PiVersion ver;
        uint32_t base;
        int      clk_khz;
    } const map[] = {
        { RP_PI_4,       BCM2711_BASE_REG, CLOCK_KHZ_375000 },
        { RP_PI_3,       BCM2837_BASE_REG, CLOCK_KHZ_250000 },
        { RP_PI_2,       BCM2837_BASE_REG, CLOCK_KHZ_250000 },
        { RP_PI_ZERO_2_W, BCM2837_BASE_REG, CLOCK_KHZ_250000 },
        { RP_PI_ZERO,    BCM2835_BASE_REG, CLOCK_KHZ_250000 },
    };

    PiVersion v = detect_version();
    for (size_t i = 0; i < sizeof(map)/sizeof(map[0]); ++i) {
        if (map[i].ver == v) {
            phys_base_reg = map[i].base;
            clock_khz     = map[i].clk_khz;
            return;
        }
    }
    fprintf(stderr, "Unsupported Pi version %d\n", v);
}

int validate_gpio(int gpio) {
    if (gpio < MIN_GPIO_NUM || gpio > MAX_GPIO_NUM) {
        printf("Supported GPIO %d ~ %d\n", MIN_GPIO_NUM, MAX_GPIO_NUM);
        return -1;
    }
    return 0;
}

int validate_range(int range) {
    if (range < MIN_RANGE || range > MAX_RANGE) {
        printf("Supported Range %d ~ %d\n", MIN_RANGE, MAX_RANGE);
        return -1;
    }
    return 0;
}

int validate_frequency(int freq) {
    if (freq < MIN_FREQ || freq > MAX_FREQ) {
        printf("Supported Frequency %d ~ %dHz\n", MIN_FREQ, MAX_FREQ);
        return -1;
    }
    if (MILLION % freq != 0) {
        printf("Not supported frequency\n");
        return -1;
    }
    return 0;
}

int set_pwm_gpio(int gpio) {
    if (validate_gpio(gpio) < 0) return -1;
    set_wave_gpio(gpio);
    return 0;
}

int set_pwm_frequency(int gpio, int freq) {
    if (validate_gpio(gpio) < 0 || validate_frequency(freq) < 0) return -1;
    return set_frequency(gpio, freq);
}

int set_pwm_range(int gpio, int range) {
    if (validate_gpio(gpio) < 0 || validate_range(range) < 0) return -1;
    return set_range(gpio, range);
}

int set_pwm_value(int gpio, int value) {
    if (validate_gpio(gpio) < 0) return -1;
    return set_wave_value(gpio, value);
}

int apply_pwm(void) {
    return apply_waves();
}

// DMA Buffer

void *get_dmabuf(void){
    return dma_buf;
}

int init_dmabuf(void){
    int fd = open(DEVICE_PATH_NAME, O_RDWR | O_SYNC);

    if (fd < 0) {
        perror("open error(dmabuf)");
        return -1;
    } else{
        dma_buf = mmap(NULL, DMA_BUF_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (dma_buf == MAP_FAILED) {
            perror("mmap error(dmabuf)");
            return -1;
        }

        if(ioctl(fd, DMA_BUF_IOCTL_GET_PHYS_ADDR, &phys_addr) < 0){
            perror("ioctl error(physaddr)");
            return -1;
        }
    }

    close(fd);

    memset(dma_buf, 0, DMA_BUF_SIZE);

    return 0;
}

void exit_dmabuf(void){
    if(dma_buf){
        munmap(dma_buf, DMA_BUF_SIZE);
    }
    phys_addr = 0;
}

uint32_t convert_to_bus_addr(void *ptr){
    uintptr_t offset = (uintptr_t)ptr - (uintptr_t)dma_buf;
    uintptr_t offset_phys_addr = phys_addr + offset;
    uintptr_t bus_addr = offset_phys_addr | 0xC0000000;
    return bus_addr;
}


// Peripherals

int map_all_registers(void) {
    #define MAP_OR_FAIL(ptr, addr) \
        do { \
            (ptr) = map_segment((addr), PAGE_SIZE); \
            if (!(ptr)) { fprintf(stderr, "Failed mmap at 0x%08X\n", (addr)); return -1; } \
        } while (0)

    MAP_OR_FAIL(dma_reg,   PHYS_DMA_BASE);
    MAP_OR_FAIL(pwm_reg,   PHYS_PWM_BASE);
    MAP_OR_FAIL(gpio_reg,  PHYS_GPIO_BASE);
    MAP_OR_FAIL(clock_reg, PHYS_CLOCK_BASE);

    return 0;
    #undef MAP_OR_FAIL
}

void unmap_all_registers(void){
    if (dma_reg)      munmap((void*)dma_reg,      PAGE_SIZE);
    if (pwm_reg)      munmap((void*)pwm_reg,      PAGE_SIZE);
    if (gpio_reg)     munmap((void*)gpio_reg,     PAGE_SIZE);
    if (clock_reg)    munmap((void*)clock_reg,    PAGE_SIZE);
}

uint32_t *get_dma_chan_reg(void){
    return (uint32_t *)((uint8_t *)dma_reg + (DMA_CHAN * DMA_CHAN_OFFSET));
}

void enable_dma(void){
    uint32_t *dma_chan_reg = get_dma_chan_reg();

    dma_reg[REG_IDX(DMA_ENABLE)] |= 1 << DMA_CHAN;
    usleep(10);
    dma_chan_reg[REG_IDX(DMA_CS)] = 1 << 31;
    usleep(10);
}

void start_dma(DMA_CB *dma_cb){
    uint32_t *dma_chan_reg = get_dma_chan_reg();

    dma_chan_reg[REG_IDX(DMA_CB_AD)] = (uint32_t)convert_to_bus_addr(dma_cb);
    usleep(10);
    dma_chan_reg[REG_IDX(DMA_CS)] = 2;
    usleep(10);
    dma_chan_reg[REG_IDX(DMA_DEBUG)] = 7;
    usleep(10);
    dma_chan_reg[REG_IDX(DMA_CS)] = 1;
    usleep(10);
}

void stop_dma(void){
    uint32_t *dma_chan_reg = get_dma_chan_reg();

    dma_chan_reg[REG_IDX(DMA_CS)] = 1 << 31;
    usleep(10);
}

uint32_t get_dma_cb_ad(void){
    uint32_t *dma_chan_reg = get_dma_chan_reg();

    return dma_chan_reg[REG_IDX(DMA_CB_AD)];
}


void set_gpio_mode_input(int gpio_num){
    int fsel_idx = gpio_num / 10;
    int shift = (gpio_num % 10) * 3;
    uint32_t *fsel = gpio_reg + fsel_idx;
    *fsel = *fsel & ~(7 << shift);
}

void set_gpio_mode_output(int gpio_num){
    int fsel_idx = gpio_num / 10;
    int shift = (gpio_num % 10) * 3;
    uint32_t *fsel = gpio_reg + fsel_idx;
    *fsel = (*fsel & ~(7 << shift)) | (1 << shift);
}

void set_gpio(int gpio_num){
    gpio_reg[REG_IDX(GPSET0)] = 1 << gpio_num;
}

void clear_gpio(int gpio_num){
    gpio_reg[REG_IDX(GPCLR0)] = 1 << gpio_num;
}

void clear_gpios_by_reg(uint32_t reg){
    gpio_reg[REG_IDX(GPCLR0)] = reg;
}


void init_pwm(void){
    int div = clock_khz / 1000;

    stop_pwm();
    clock_reg[REG_IDX(CM_PWMCTL)] = CM_PASSWORD | (1 << 5);
    usleep(10);
    clock_reg[REG_IDX(CM_PWMDIV)] = CM_PASSWORD | (div << 12);
    usleep(10);
    clock_reg[REG_IDX(CM_PWMCTL)] = CM_PASSWORD | 6 | (1 << 4);
    usleep(10);
    pwm_reg[REG_IDX(PWM_RNG1)] = 1;
    usleep(10);
}

void start_pwm(void){
    pwm_reg[REG_IDX(PWM_DMAC)] = PWM_DMAC_ENAB | PWM_DMAC_THRSHLD;
    usleep(10);
    pwm_reg[REG_IDX(PWM_CTL)] = PWM_CTL_USEF1 | PWM_ENAB;
    usleep(10);
}

void stop_pwm(void){
    pwm_reg[REG_IDX(PWM_CTL)] = 0;
    usleep(10);
}

// Waveform

void *get_gpio_buff(BufferIdx buf_i){
    int offset = (sizeof(uint32_t) * (CB_MAX * 2)) * buf_i;
    return get_dmabuf() + offset;
}

void *get_dma_cb_buff(BufferIdx buf_i){
    int offset = (sizeof(DMA_CB) * (CB_MAX + 1)) * buf_i;
    return get_gpio_buff(BUF_2 + 1) + offset;
}

void *get_dummy_buff(void){
    return get_dma_cb_buff(BUF_2 + 1);
}

int gcd(int a, int b) {
    while (b != 0) {
        int temp = b;
        b = a % b;
        a = temp;
    }
    return a;
}

int lcm(int a, int b) {
    return a / gcd(a, b) * b;
}

int convert_freq_to_period(int freq){
    return MILLION / freq;
}

void init_waves(void){
    for(int i=0; i<WAVE_NUM; i++){
        Wave *wave = waves + i;
        wave->gpio = 1 << (i + 1);
        wave->freq = 0;
        wave->range = 0;
        wave->period = 0;
        wave->range_value = 0;
        wave->period_value = 0;
        wave->spot_num = 0;
        wave->set_spots = NULL;
        wave->clr_spots = NULL;
        wave->always_on = false;
        wave->always_off = false;
        wave->active = false;
        wave->need_update = false;
    }
}

Wave *get_wave_by_gpio(int gpio){
    return (waves + (gpio - 1));
}

int cal_waveform_period(Wave *t_waves){
    int combined_period = 0;
    for(int i=0; i<WAVE_NUM; i++){
        Wave *wave = t_waves + i;

        if(wave->freq > 0){
            int period = convert_freq_to_period(wave->freq);
            if(combined_period == 0){
                combined_period = period;
            } else{
                combined_period = lcm(combined_period, period);
            }
        }
    }

    return combined_period;
}

void set_wave_gpio(int gpio){
    set_gpio_mode_output(gpio);
}

int set_frequency(int gpio, int freq){
    int t_waveform_period = 0;
    Wave t_waves[WAVE_NUM];

    for(int i=0; i<WAVE_NUM; i++){
        t_waves[i] = *(waves + i);
    }

    t_waves[gpio - 1].freq = freq;

    t_waveform_period = cal_waveform_period(t_waves);

    for(int i=0; i<WAVE_NUM; i++){
        Wave *t_wave = t_waves + i;

        if(t_wave->freq > 0){
            int period = convert_freq_to_period(t_wave->freq);
            int spot_num = t_waveform_period / period;

            t_wave->spot_num = spot_num;
        }
    }

    int total_spot_num = 0;

    for(int i=0; i<WAVE_NUM; i++){
        Wave *t_wave = t_waves + i;
        total_spot_num += t_wave->spot_num;
    }

    // Two events(SET, CLR) per spot, 3 DMA_CB per event
    int required_cb_count = total_spot_num * 2 * 3;
    // printf("Required DMA Control Blocks: %d\n", required_cb_count);
    if (required_cb_count > CB_MAX) {
        printf("Cannot set GPIO %d to %d Hz: requires %d DMA blocks\n", gpio, freq, required_cb_count);
        return -1;
    }

    Wave *target_wave = get_wave_by_gpio(gpio);

    target_wave->freq = freq;
    target_wave->range_value = 0;
    target_wave->period = convert_freq_to_period(freq);;
    target_wave->spot_num = t_waveform_period / target_wave->period;

    waveform_period = cal_waveform_period(waves);

    for(int i=0; i<WAVE_NUM; i++){
        Wave *wave = waves + i;

        if(wave->freq > 0){
            wave->spot_num = waveform_period / wave->period;

            if(wave->clr_spots){
                free(wave->clr_spots);
                wave->clr_spots = NULL;
            }
            if(wave->set_spots){
                free(wave->set_spots);
                wave->set_spots = NULL;
            }

            wave->clr_spots = (int *)malloc(sizeof(int) * wave->spot_num);
            wave->set_spots = (int *)malloc(sizeof(int) * wave->spot_num);
            if(!wave->clr_spots || !wave->set_spots){
                fprintf(stderr, "memory allocation fail on spots\n");
                return -1;
            }
        }
    }

    return 0;
}

int set_range(int gpio, int range){
    Wave *wave = get_wave_by_gpio(gpio);
    
    if(wave->freq <= 0){
        printf("Set frequency first on GPIO %d\n", gpio);
        return -1;
    }

    wave->range = range;
    wave->range_value = 0;
    wave->active = true;

    return 0;
}

int set_wave_value(int gpio, int value){
    Wave *wave = get_wave_by_gpio(gpio);

    if(value < 0 || wave->range < value){
        printf("GPIO %d: value %d is out of range (0~%d)\n", gpio, value, wave->range);
        return -1;
    }

    wave->range_value = value;
    wave->need_update = true;

    return 0;
}

void free_all_spots(void){
    for(int i=0; i<WAVE_NUM; i++){
        Wave *wave = waves + i;
        if(wave->clr_spots){
            free(wave->clr_spots);
            wave->clr_spots = NULL;
        }
        if(wave->set_spots){
            free(wave->set_spots);
            wave->set_spots = NULL;
        }
    }
}

int apply_waves(void){
    int synced_freq = 0;
    bool synced = true;

    for(int i=0; i<WAVE_NUM; i++){
        Wave *wave = waves + i;
        
        if(wave->active){
            if(synced_freq == 0){
                synced_freq = wave->freq;
            } else{
                if(synced_freq != wave->freq){
                    synced = false;
                    break;
                }
            }
        }
    }

    if(synced){
        return update_synced_waveform(synced_freq);
    } else{
        return update_asynced_waveform();
    }
}

bool is_staying_previous_buffer(void){
    if(prev_dma_cbs){
        uintptr_t start_addr = convert_to_bus_addr(prev_dma_cbs);
        uintptr_t end_addr = convert_to_bus_addr(prev_dma_cbs + CB_MAX);
        uintptr_t curr_addr = get_dma_cb_ad();

        if(curr_addr != 0 && (curr_addr >= start_addr && curr_addr <= end_addr)){
            return true;
        }
    }
    return false;
}

int update_asynced_waveform(void){

    if(is_staying_previous_buffer()) return -1;

    DMA_CB *curr_last_cb;
    DMA_CB *dma_cbs;
    uint32_t *gpios;
    uint32_t *dummy = get_dummy_buff();

    int event_count = 0;
    for(int i=0; i<WAVE_NUM; i++){
        Wave *wave = waves + i;

        if(wave->active){
            if((wave->range > wave->period)){
                wave->period_value = wave->range_value / (wave->range / wave->period);
            } else{
                wave->period_value = wave->range_value * (wave->period / wave->range);
            }

            wave->always_on = wave->period_value == wave->period;
            wave->always_off = wave->period_value == 0;

            if(wave->always_on || wave->always_off){
                event_count += wave->spot_num;
            } else{
                event_count += (wave->spot_num * 2);
            }
            
            for(int n=0; n<wave->spot_num; n++){

                int np = wave->period * n;
                int set_np = wave->always_off ? 0 : np;
                int clr_np = wave->always_on ? 0 : np;

                if(!wave->always_on && !wave->always_off){
                    clr_np += wave->period_value;
                }
                
                wave->set_spots[n] = set_np;
                wave->clr_spots[n] = clr_np;
            }
        }
    }

    int idx = 0;
    WaveformEvent events[event_count];

    for(int i=0; i<WAVE_NUM; i++){
        Wave *wave = waves + i;

        if(wave->active){
            if(wave->always_on){
                for(int k=0; k<wave->spot_num; k++){
                    events[idx].gpio = wave->gpio;
                    events[idx].gpio_reg = GPSET0;
                    events[idx].spot = wave->set_spots[k];
                    idx++;
                }

            } else if(wave->always_off){
                for(int k=0; k<wave->spot_num; k++){
                    events[idx].gpio = wave->gpio;
                    events[idx].gpio_reg = GPCLR0;
                    events[idx].spot = wave->clr_spots[k];
                    idx++;
                }

            } else{
                for(int k=0; k<wave->spot_num; k++){
                    events[idx].gpio = wave->gpio;
                    events[idx].gpio_reg = GPSET0;
                    events[idx].spot = wave->set_spots[k];
                    idx++;

                    events[idx].gpio = wave->gpio;
                    events[idx].gpio_reg = GPCLR0;
                    events[idx].spot = wave->clr_spots[k];
                    idx++;
                }
            }
        }
    }

    sort_events(events, event_count);

    switch(buffer_idx){
        case BUF_0:
            dma_cbs = get_dma_cb_buff(BUF_0);
            gpios = get_gpio_buff(BUF_0);
            prev_dma_cbs = get_dma_cb_buff(BUF_1);
            buffer_idx = BUF_1;
            break;

        case BUF_1:
            dma_cbs = get_dma_cb_buff(BUF_1);
            gpios = get_gpio_buff(BUF_1);
            prev_dma_cbs = get_dma_cb_buff(BUF_2);
            buffer_idx = BUF_2;
            break;

        case BUF_2:
            dma_cbs = get_dma_cb_buff(BUF_2);
            gpios = get_gpio_buff(BUF_2);
            prev_dma_cbs = get_dma_cb_buff(BUF_0);
            buffer_idx = BUF_0;
            break;
    }


    uint32_t merged_set_gpio = 0;
    uint32_t merged_clr_gpio = 0;
    
    prev_cb_num = curr_cb_num;
    curr_cb_num = 0;
    idx = 0;

    for(int i=0; i<event_count; i++){

        WaveformEvent event = events[i];
        int spot_diff = 0;

        if(i == event_count - 1){
            spot_diff = waveform_period - event.spot;
        } else{
            spot_diff = events[i + 1].spot - event.spot;   
        }

        if(event.gpio_reg == GPSET0){
            merged_set_gpio |= event.gpio;
        } else{
            merged_clr_gpio |= event.gpio;
        }

        if(spot_diff > 0){
            uint32_t *set_gpio = gpios + (idx * 2);
            uint32_t *clr_gpio = set_gpio + 1;
            DMA_CB *set_gpio_cb = dma_cbs + (idx * 3);
            DMA_CB *clr_gpio_cb = set_gpio_cb + 1;
            DMA_CB *delay_cb = clr_gpio_cb + 1;

            *set_gpio = merged_set_gpio;
            *clr_gpio = merged_clr_gpio;

            set_gpio_cb->ti = PWM_TI;
            set_gpio_cb->srce_ad = convert_to_bus_addr(set_gpio);
            set_gpio_cb->dest_ad = BUS_GPIO_BASE + GPSET0;
            set_gpio_cb->txfr_len = 4;
            set_gpio_cb->stride = 0;
            set_gpio_cb->next_cb = convert_to_bus_addr(clr_gpio_cb);

            clr_gpio_cb->ti = PWM_TI;
            clr_gpio_cb->srce_ad = convert_to_bus_addr(clr_gpio);
            clr_gpio_cb->dest_ad = BUS_GPIO_BASE + GPCLR0;
            clr_gpio_cb->txfr_len = 4;
            clr_gpio_cb->stride = 0;
            clr_gpio_cb->next_cb = convert_to_bus_addr(delay_cb);

            delay_cb->ti = PWM_TI;
            delay_cb->srce_ad = convert_to_bus_addr(dummy);
            delay_cb->dest_ad = BUS_PWM_BASE + PWM_FIF1;
            delay_cb->txfr_len = 4 * spot_diff;
            delay_cb->stride = 0;

            if(i == event_count - 1){
                delay_cb->next_cb = convert_to_bus_addr(dma_cbs);
                curr_last_cb = delay_cb;
            } else{
                delay_cb->next_cb = convert_to_bus_addr(delay_cb + 1);
            }

            merged_set_gpio = 0;
            merged_clr_gpio = 0;
            
            idx++;

            curr_cb_num += 3;
        }

    }

    int update_period = 0;
    for(int i=0; i<WAVE_NUM; i++){
        Wave *wave = waves + i;
        if(wave->need_update){
            update_period = update_period > wave->period ? update_period : wave->period;
            wave->need_update = false;
        }
    }

    int curr_accum = 0;
    int prev_accum = 0;

    int curr_dcb_num = curr_cb_num / 3;
    int prev_dcb_num = prev_cb_num / 3;

    for(int i=0, k=0; i<curr_dcb_num; i++){
        DMA_CB *curr_dcb = dma_cbs + (i * 3) + 2;

        curr_accum += curr_dcb->txfr_len;

        DMA_CB *prev_dcb;

        while(curr_accum > prev_accum && k < prev_dcb_num){
            prev_dcb = prev_dma_cbs + (k * 3) + 2;
            prev_accum += prev_dcb->txfr_len;
            k++;
        }

        if(curr_accum == prev_accum && prev_dcb && ((curr_accum / 4) % update_period == 0)){
            
            if(i == curr_dcb_num - 1){
                prev_dcb->next_cb = convert_to_bus_addr(dma_cbs);
            } else{
                prev_dcb->next_cb = convert_to_bus_addr(curr_dcb + 1);
            }
        }
    }

    if(prev_last_cb){
        prev_last_cb->next_cb = convert_to_bus_addr(dma_cbs);
    }
    prev_last_cb = curr_last_cb;

    start_waveform(dma_cbs);

    return 0;
}

int update_synced_waveform(int freq){

    if(is_staying_previous_buffer()) return -1;

    DMA_CB *dma_cbs;
    uint32_t *gpios;
    uint32_t *dummy = get_dummy_buff();

    int period = convert_freq_to_period(freq);
    int event_count = 0;

    WaveformEvent events[WAVE_NUM * 2];

    for(int i=0; i<WAVE_NUM; i++){
        Wave *wave = waves + i;

        if(wave->active){
            int period_value;

            if(wave->range > period){
                period_value = wave->range_value / (wave->range / period);
            } else{
                period_value = wave->range_value * (period / wave->range);
            }

            if(period_value == period){
                events[event_count].gpio = wave->gpio;
                events[event_count].gpio_reg = GPSET0;
                events[event_count].spot = 0;
                event_count++;
                
            } else if(period_value == 0){
                events[event_count].gpio = wave->gpio;
                events[event_count].gpio_reg = GPCLR0;
                events[event_count].spot = 0;
                event_count++;

            } else{
                events[event_count].gpio = wave->gpio;
                events[event_count].gpio_reg = GPSET0;
                events[event_count].spot = 0;
                event_count++;

                events[event_count].gpio = wave->gpio;
                events[event_count].gpio_reg = GPCLR0;
                events[event_count].spot = period_value;
                event_count++;
            }
        }
    }

    sort_events(events, event_count);

    switch(buffer_idx){
        case BUF_0:
            dma_cbs = get_dma_cb_buff(BUF_0);
            gpios = get_gpio_buff(BUF_0);
            prev_dma_cbs = get_dma_cb_buff(BUF_1);
            buffer_idx = BUF_1;
            break;

        case BUF_1:
            dma_cbs = get_dma_cb_buff(BUF_1);
            gpios = get_gpio_buff(BUF_1);
            prev_dma_cbs = get_dma_cb_buff(BUF_2);
            buffer_idx = BUF_2;
            break;

        case BUF_2:
            dma_cbs = get_dma_cb_buff(BUF_2);
            gpios = get_gpio_buff(BUF_2);
            prev_dma_cbs = get_dma_cb_buff(BUF_0);
            buffer_idx = BUF_0;
            break;
    }

    DMA_CB *curr_last_cb;
    DMA_CB *all_set_cb = dma_cbs;
    DMA_CB *all_clr_cb = dma_cbs + 1;
    uint32_t *all_set_gpio = gpios;
    uint32_t *all_clr_gpio = gpios + 1;
    uint32_t merged_gpio = 0;

    *all_set_gpio = 0;
    *all_clr_gpio = 0;

    all_set_cb->ti = PWM_TI;
    all_set_cb->srce_ad = convert_to_bus_addr(all_set_gpio);
    all_set_cb->dest_ad = BUS_GPIO_BASE + GPSET0;
    all_set_cb->txfr_len = 4;
    all_set_cb->stride = 0;
    all_set_cb->next_cb = convert_to_bus_addr(dma_cbs + 1);

    all_clr_cb->ti = PWM_TI;
    all_clr_cb->srce_ad = convert_to_bus_addr(all_clr_gpio);
    all_clr_cb->dest_ad = BUS_GPIO_BASE + GPCLR0;
    all_clr_cb->txfr_len = 4;
    all_clr_cb->stride = 0;
    all_clr_cb->next_cb = convert_to_bus_addr(dma_cbs + 2);

    int idx = 0;

    for(int i=0; i<event_count; i++){

        WaveformEvent event = events[i];
        uint32_t *gpio = gpios + 2 + idx;
        int spot_diff;
        
        if(i == event_count - 1){
            spot_diff = period - event.spot;
        } else{
            spot_diff = events[i + 1].spot - event.spot;
        }

        if(event.spot == 0){
            if(event.gpio_reg == GPSET0){
                *all_set_gpio |= event.gpio;
            } else{
                *all_clr_gpio |= event.gpio;
            }

        } else{
            merged_gpio |= event.gpio;
        }

        if(spot_diff > 0){

            if(event.spot == 0){
                DMA_CB *delay_cb = dma_cbs + 2;

                delay_cb->ti = PWM_TI;
                delay_cb->srce_ad = convert_to_bus_addr(dummy);
                delay_cb->dest_ad = BUS_PWM_BASE + PWM_FIF1;
                delay_cb->txfr_len = 4 * spot_diff;
                delay_cb->stride = 0;

                if(i == event_count - 1){
                    delay_cb->next_cb = convert_to_bus_addr(dma_cbs);
                    curr_last_cb = delay_cb;
                } else{
                    delay_cb->next_cb = convert_to_bus_addr(delay_cb + 1);
                }

            } else{
                DMA_CB *gpio_cb = dma_cbs + 3 + (idx * 2);
                DMA_CB *delay_cb = gpio_cb + 1;

                *gpio = merged_gpio;

                gpio_cb->ti = PWM_TI;
                gpio_cb->srce_ad = convert_to_bus_addr(gpio);
                gpio_cb->dest_ad = BUS_GPIO_BASE + event.gpio_reg;
                gpio_cb->txfr_len = 4;
                gpio_cb->stride = 0;
                gpio_cb->next_cb = convert_to_bus_addr(delay_cb);

                delay_cb->ti = PWM_TI;
                delay_cb->srce_ad = convert_to_bus_addr(dummy);
                delay_cb->dest_ad = BUS_PWM_BASE + PWM_FIF1;
                delay_cb->txfr_len = 4 * spot_diff;
                delay_cb->stride = 0;

                if(i == event_count - 1){
                    delay_cb->next_cb = convert_to_bus_addr(dma_cbs);
                    curr_last_cb = delay_cb;
                } else{
                    delay_cb->next_cb = convert_to_bus_addr(delay_cb + 1);
                }
                
                merged_gpio = 0;

                idx++;
            }
        }
    }

    if(prev_last_cb){
        prev_last_cb->next_cb = convert_to_bus_addr(dma_cbs);
    }
    prev_last_cb = curr_last_cb;

    start_waveform(dma_cbs);
    
    return 0;
}

void sort_events(WaveformEvent events[], int n) {
    int i, j;
    WaveformEvent key;

    for (i = 1; i < n; i++) {
        key = events[i];
        j = i - 1;

        while (j >= 0 && events[j].spot > key.spot) {
            events[j + 1] = events[j];
            j--;
        }
        events[j + 1] = key;
    }
}

void print_dma_buff(int len){
    printf("\nDMA CB Buffer\n");
        
    DMA_CB *dma_cb_ptr = get_dma_cb_buff(buffer_idx);

    for(int i=0; i<len; i++){
        DMA_CB *dma_cb = dma_cb_ptr + i;
        printf("CB[%d] ti 0x%x, src 0x%x, dst 0x%x, txlen 0x%x, stride 0x%x, nxtcb 0x%x\n",
                i, dma_cb->ti, dma_cb->srce_ad, dma_cb->dest_ad, dma_cb->txfr_len,
                dma_cb->stride, dma_cb->next_cb);
    }
}

void start_waveform(DMA_CB *dma_cb){
    if(!waveform_started){
        init_pwm();
        start_pwm();
        start_dma(dma_cb);
    } 
    waveform_started = true;
}

void stop_waveform(void){
    stop_pwm();
    stop_dma();
}

void clear_all_waves(void){
    uint32_t reg = 0;

    for(int i=0; i<WAVE_NUM; i++){
        Wave *wave = waves + i;

        if(wave->freq > 0){
            reg |= wave->gpio;
        }
    }

    clear_gpios_by_reg(reg);
}
