/*
Simple execution code to check library works.
You can use LED blinking to check PWM signal.
*/

#include "pipwm.h"

int main(void){

    int pin1 = 21;
    int pin2 = 20;

    printf("--PiPWM Start--\n");
  
    if(init_pipwm() < 0){
        printf("Failed to init pipwm\n");
        return -1;
    }

    set_pwm_gpio(pin1);
    set_pwm_gpio(pin2);
    set_pwm_frequency(pin1, 2);
    set_pwm_frequency(pin2, 10);
    set_pwm_range(pin1, 100);
    set_pwm_range(pin2, 100);

    printf("Single PWM test..\n");

    set_pwm_value(pin1, 10);
    apply_pwm();

    sleep(5);

    printf("Double PWM test..\n");

    set_pwm_value(pin1, 50);
    set_pwm_value(pin2, 50);
    apply_pwm();

    sleep(5);

    exit_pipwm();

    printf("--PiPWM End--\n");

    return 0;
}
