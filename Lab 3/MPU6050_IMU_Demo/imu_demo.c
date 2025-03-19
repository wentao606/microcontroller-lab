/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
// Include custom libraries
#include "vga16_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_3.h"

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// character array
char screentext[40];

// draw speed
int threshold = 10 ;

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphore
static struct pt_sem vga_semaphore ;

// Some paramters for PWM
#define WRAPVAL 5000
#define CLKDIV  25.0
uint slice_num ;
fix15 accel_angle;
fix15 gyro_angle_delta;
fix15 complementary_angle;
fix15 reference_angle = int2fix15(30);
fix15 Kp, kd;
fix15 previous_a [3];
fix15 filteredax = float2fix15(0.);
fix15 filtereday = float2fix15(0.);
fix15 filteredaz = float2fix15(0.);
float PI=3.14;
// PWM duty cycle
volatile float kp_value = 30. ;
volatile float old_kp_value = 0.;
volatile fix15 pwm_control;
volatile fix15 error;

// Interrupt service routine
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);
    //angle += gyro[0];
    //printf( "angle:%f, %f, %f\n", fix2float15(gyro[0]), fix2float15(gyro[1]), fix2float15(gyro[2]) );
    
    filteredax = previous_a[0] + ((acceleration[0]-previous_a[0])>>4);
    filtereday = previous_a[1] + ((acceleration[1]-previous_a[1])>>4);
    filteredaz = previous_a[2] + ((acceleration[2]-previous_a[2])>>4);
    
    previous_a[0] = filteredax;
    previous_a[1] = filtereday;
    previous_a[2] = filteredaz;
    // Calculate angle

    // Accelerometer angle (degrees - 15.16 fixed point) 
    // Only ONE of the two lines below will be used, depending whether or not a small angle approximation is appropriate

    // SMALL ANGLE APPROXIMATION
    //accel_angle = multfix15(divfix(acceleration[1], acceleration[2]), oneeightyoverpi) ;
    // NO SMALL ANGLE APPROXIMATION_
    
    accel_angle = multfix15(float2fix15(atan2(fix2float15(-filtereday), fix2float15(filteredaz)) + PI), oneeightyoverpi);
    
    // Gyro angle delta (measurement times timestep) (15.16 fixed point)
    // gyro_angle_delta = multfix15(gyro[1], zeropt001) ;
    gyro_angle_delta = multfix15(gyro[1], float2fix15(0.5)) ;
    // Complementary angle (degrees - 15.16 fixed point)
    // complementary_angle = multfix15(complementary_angle - gyro_angle_delta, zeropt999) 
    //                     + multfix15(accel_angle, zeropt001);
    complementary_angle = multfix15(complementary_angle - gyro_angle_delta, float2fix15(0.01)) 
                        + multfix15(accel_angle, float2fix15(0.99));
    complementary_angle = (complementary_angle > float2fix15(180))?(complementary_angle-float2fix15(360)):complementary_angle;
    // Increase the range of the angle
    //complementary_angle = multfix15(complementary_angle, float2fix15(1.6));
    // Proportional control
    // if (kp_value != old_kp_value){
    //     old_kp_value = kp_value;
    //     pwm_control = multfix15(float2fix15(kp_value), (reference_angle-complementary_angle));
    //     //printf("pwm duty cycle:%d, angle:%f\n",fix2int15(pwm_control), fix2float15(complementary_angle));
    //     if (pwm_control < int2fix15(0)) {
    //         pwm_control = int2fix15(0);
    //     }
    //     else if (pwm_control > int2fix15(5000)){
    //         pwm_control = int2fix15(5000);
    //     }
    //     pwm_set_chan_level(slice_num, PWM_CHAN_A, fix2int15(pwm_control));
    // //     printf("pwm duty cycle:%d, angle:%f\n",fix2int15(pwm_control), fix2float15(complementary_angle));
    // }
    
    pwm_control = multfix15(float2fix15(kp_value), (reference_angle-complementary_angle));


    if (pwm_control < int2fix15(0)) {
            pwm_control = int2fix15(0);
        }
    else if (pwm_control > int2fix15(5000)){
        pwm_control = int2fix15(5000);
    }
    //printf("pwm duty cycle:%d, angle:%f\n",fix2int15(pwm_control), fix2float15(complementary_angle));
    pwm_set_chan_level(slice_num, PWM_CHAN_A, fix2int15(pwm_control));
    
    
   
    
    // printf("angle:%f\n", fix2float15(complementary_angle));
    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // We will start drawing at column 81
    static int xcoord = 81 ;
    
    // Rescale the measurements for display
    static float OldRange = 360. ; // (+/- 180)
    static float NewRange = 150. ; // (looks nice on VGA)
    static float OldMin = -180. ;
    static float OldMax = 180. ;

    // Control rate of drawing
    static int throttle ;

    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "+2") ;
    setCursor(50, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "-2") ;
    setCursor(50, 425) ;
    writeString(screentext) ;

    // Draw top plot
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "+250") ;
    setCursor(45, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "-250") ;
    setCursor(45, 225) ;
    writeString(screentext) ;
    

    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1 ;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) { 
            // Zero drawspeed controller
            throttle = 0 ;

            // Erase a column
            drawVLine(xcoord, 0, 480, BLACK) ;

            // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
            drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(filteredax)*120.0)-OldMin)/OldRange)), WHITE) ;
            drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(filtereday)*120.0)-OldMin)/OldRange)), RED) ;
            drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(filteredaz)*120.0)-OldMin)/OldRange)), GREEN) ;

            // Draw top plot
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(complementary_angle))-OldMin)/OldRange)), WHITE) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(complementary_angle))-OldMin)/OldRange)), RED) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(complementary_angle))-OldMin)/OldRange)), GREEN) ;

            //angle += (230 - NewRange*((float)((fix2float15(gyro[0]))-OldMin)/OldRange));
            //printf( "gyro: %f, angle:%f\n", fix2float15(gyro[0]), angle);
            

            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
            }
        }
    }
    // Indicate end of thread
    PT_END(pt);
}

// User input thread. User can change draw speed
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char classifier ;
    static float test_in ;
    
    while(1) {
        sprintf(pt_serial_out_buffer, "input Kp value (0-60): ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%f", &test_in) ;
        kp_value = test_in;

        // num_independents = test_in ;
        // if (classifier=='t') {
        //     sprintf(pt_serial_out_buffer, "timestep: ");
        //     serial_write ;
        //     serial_read ;
        //     // convert input string to number
        //     sscanf(pt_serial_in_buffer,"%d", &test_in) ;
        //     if (test_in > 0) {
        //         threshold = test_in ;
        //     }
        // }
    }
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;
}

int main() {

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;

    // Pullup resistors on breakout board, don't need to turn on internals
    // gpio_pull_up(SDA_PIN) ;
    // gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    
    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
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

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 3125);

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    //pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}
