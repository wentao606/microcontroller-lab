/**
 * V. Hunter Adams (vha3@cornell.edu)
 * PWM demo code with serial input
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pwm.h"
#include "hardware/irq.h"

#include "pt_cornell_rp2040_v1.h"

// Midpoint of potentiometer
#define WRAPVAL 25000
#define CLKDIV 5.0f

// Variable to hold PWM slice number
uint slice_num ;

// PWM duty cycle
volatile int control ;
volatile int old_control ;


void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(4));

    if (control!=old_control) {
        old_control = control ;
        pwm_set_chan_level(slice_num, PWM_CHAN_B, control);
    }
}


static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static int test_in ;
    while(1) {
        sprintf(pt_serial_out_buffer, "input a duty cycle (0-25000): ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &test_in) ;
        if (test_in > 25000) continue ;
        else if (test_in < 0) continue ;
        else control = test_in ;
    }
    PT_END(pt) ;
}

int main() {

    stdio_init_all();

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO 4 that it is allocated to the PWM
    gpio_set_function(4, GPIO_FUNC_PWM);

    // // Find out which PWM slice is connected to GPIO 4 (it's slice 2)
    slice_num = pwm_gpio_to_slice_num(4);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // Invert channel B
    pwm_set_output_polarity(slice_num, 0, 1)

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 3125);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}
