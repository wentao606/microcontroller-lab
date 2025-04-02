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

static char text1[40];

// Some paramters for PWM
#define WRAPVAL 5000
#define CLKDIV  25.0
// Button gpio 
#define BUTTON_PIN 28

typedef enum {
    RESET,
    BUTTON_NOT_PRESSED,
    BUTTON_MAYBE_PRESSED,
    BUTTON_PRESSED,
    BUTTON_MAYBE_NOT_PRESSED
} ButtonState;
static ButtonState button_state = RESET;
static int button_value = -1;
static int possible = -1;
static int button_control = 0;
static int button_control_prev = 0;

uint slice_num ;
fix15 accel_angle;
fix15 gyro_angle_delta;
fix15 complementary_angle;
fix15 reference_angle = int2fix15(60);
fix15 Kp, kd;
fix15 previous_a [3];
fix15 filteredax = float2fix15(0.);
fix15 filtereday = float2fix15(0.);
fix15 filteredaz = float2fix15(0.);
float PI=3.14;
// PWM duty cycle
volatile float kp_value = 50. ;
volatile float old_kp_value = 0.;
volatile float kd_value = 1.0;
volatile float ki_value = 0.03;
volatile float ki_value_set = 0.03;
volatile float i_term;
volatile int pwm_control;
volatile int filter_pwm;
volatile int prev_pwm_control;
volatile float error;
volatile float filter_error;
volatile float prev_error;
volatile float prev_time;
volatile float curr_time;
volatile float delta_time;

void button_pressing()
{
    switch (button_state)
    {

    case RESET:
        button_value = gpio_get(BUTTON_PIN);
        button_control = 0;
        button_state = BUTTON_NOT_PRESSED;
    break;

    case BUTTON_NOT_PRESSED:
        button_control = 0;
        if (button_value == 0){
            
            button_value = gpio_get(BUTTON_PIN);
            button_state = BUTTON_NOT_PRESSED;
        }
    else{
        possible = button_value;
        button_value = gpio_get(BUTTON_PIN);
        button_state = BUTTON_MAYBE_PRESSED;
    }
    break;

    case BUTTON_MAYBE_PRESSED:
    if (button_value == possible){
        button_value = gpio_get(BUTTON_PIN);
        button_control = 1;
        button_state = BUTTON_PRESSED;
    }
    else{
        button_value = gpio_get(BUTTON_PIN);
        button_state = BUTTON_NOT_PRESSED;
    }
    break;
    case BUTTON_PRESSED:
    if (button_value == possible){
        button_value = gpio_get(BUTTON_PIN);
        button_state = BUTTON_PRESSED;
        }
    else{
        button_value = gpio_get(BUTTON_PIN);
        button_state = BUTTON_MAYBE_NOT_PRESSED;
    }
    case BUTTON_MAYBE_NOT_PRESSED:
    if (button_value == possible) {
        button_value = gpio_get(BUTTON_PIN);
        button_state = BUTTON_PRESSED;
    }
    else {
        button_value = gpio_get(BUTTON_PIN);
        button_state = BUTTON_NOT_PRESSED;
    }
    break;

    default:
    button_value = -1;
    possible = -1;
    break;

    }
}

static int t = 0;
void run_sequence() {
    reference_angle = int2fix15(0);
    t = time_us_32();
    
    while(1){
        if (time_us_32() - t > 5000000){
            reference_angle = int2fix15(30);
        }
        else if (time_us_32() - t > 10000000)
        {
            reference_angle = int2fix15(-30);
        }
        else if (time_us_32() - t > 15000000)
        {
            reference_angle = int2fix15(0);
        }
    }
}
int k = 0;
float buffer_kd[5];
// Interrupt service routine
void on_pwm_wrap() {
    
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    // filter the acceleration reading values
    filteredax = filteredax + ((acceleration[0]-filteredax)>>2);
    filtereday = filtereday + ((acceleration[1]-filtereday)>>2);
    filteredaz = filteredaz + ((acceleration[2]-filteredaz)>>2);
    
    previous_a[0] = filteredax;
    previous_a[1] = filtereday;
    previous_a[2] = filteredaz;
    // Calculate angle

    // Accelerometer angle (degrees - 15.16 fixed point) 
    // Only ONE of the two lines below will be used, depending whether or not a small angle approximation is appropriate

    // NO SMALL ANGLE APPROXIMATION_
    accel_angle = multfix15(float2fix15(atan2(fix2float15(filteredaz), fix2float15(filtereday))), oneeightyoverpi);
    
    // Gyro angle delta (measurement times timestep) (15.16 fixed point)
    
    gyro_angle_delta = multfix15(gyro[0], float2fix15(0.001)) ;
    // Complementary angle (degrees - 15.16 fixed point)
    
    complementary_angle = multfix15(complementary_angle - gyro_angle_delta, float2fix15(0.99)) 
                        + multfix15(accel_angle, float2fix15(0.01));
    
    
    if (complementary_angle > int2fix15(180))
        complementary_angle = int2fix15(180);
    else if (complementary_angle < int2fix15(0))
        complementary_angle = int2fix15(0);

    
    
    
    prev_error = error;
    error = fix2float15(reference_angle-complementary_angle) ;
    // convert error to radius
    filter_error = prev_error + ((error - prev_error) / 4);
    
    ki_value = ki_value_set;
    curr_time = time_us_32();
    delta_time = (curr_time - prev_time) * (1e-6);
    i_term += (1.0 * error);
    if (i_term >= (3000.0 / ki_value_set))
        i_term = 3000.0 / ki_value_set;
    else if (i_term <= (-3000.0 / ki_value_set))
        i_term = -3000.0 / ki_value_set;
    pwm_control = ((kp_value * error)
                + (kd_value * (error - prev_error))
                + (ki_value * i_term));
    
    if (pwm_control < 0) {
            pwm_control = 0;
        }
    else if (pwm_control > 3500){
        pwm_control = 3500;
    }
    
    pwm_set_chan_level(slice_num, PWM_CHAN_A, (int)(pwm_control));
    
    prev_time = curr_time;
    
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
    static float OldRange = 180. ; // (+/- 180)
    static float NewRange = 150. ; // (looks nice on VGA)
    static float OldMin = 0. ;
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
    sprintf(screentext, "") ;
    setCursor(50, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "+5000") ;
    setCursor(50, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
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
    sprintf(screentext, "+180") ;
    setCursor(45, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
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

            
            drawVLine(xcoord, 0, 480, BLACK) ;

            // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(filteredax)*120.0)-OldMin)/OldRange)), WHITE) ;
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(filtereday)*120.0)-OldMin)/OldRange)), RED) ;
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(filteredaz)*120.0)-OldMin)/OldRange)), GREEN) ;
            
            // Draw the bottom plot

            // Low-pass filter for pwm signal only when displaying
            filter_pwm = prev_pwm_control + ((pwm_control - prev_pwm_control )>> 4);
            prev_pwm_control = filter_pwm;
            drawPixel(xcoord, 430 - (int)(150*((float)(((float)(filter_pwm))/5000.0))), GREEN);
            drawPixel(xcoord, 430 - (int)(150*((float)(((float)((kd_value * (error - prev_error))))/5000.0))), RED);
            drawPixel(xcoord, 430 - (int)(150*((float)(((float)((ki_value * i_term)))/5000.0))), WHITE);
            
            drawPixel(xcoord, 430 - (int)(150*((float)(((float)((kp_value * error)))/5000.0))), BLUE);
            // Draw top plot
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(complementary_angle))-OldMin)/OldRange)), WHITE) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(complementary_angle))-OldMin)/OldRange)), RED) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(complementary_angle))-OldMin)/OldRange)), GREEN) ;
            
            // display desired angle
            sprintf(text1, "desired angle: %f   ", fix2float15(reference_angle));
            setCursor(10, 10);
            setTextColor2(WHITE, BLACK);
            setTextSize(1);
            writeString(text1);

            // display desired angle
            sprintf(text1, "actual angle: %f   ", fix2float15(complementary_angle));
            setCursor(10, 20);
            setTextColor2(WHITE, BLACK);
            setTextSize(1);
            writeString(text1);

            // display kp
            sprintf(text1, "Kp: %f   ", kp_value);
            setCursor(10, 30);
            setTextColor2(WHITE, BLACK);
            setTextSize(1);
            writeString(text1);
            
            // display kd
            sprintf(text1, "Kd: %f   ", kd_value);
            setCursor(10, 40);
            setTextColor2(WHITE, BLACK);
            setTextSize(1);
            writeString(text1);

            //display ki
            sprintf(text1, "Ki: %f   ", ki_value);
            setCursor(10, 50);
            setTextColor2(WHITE, BLACK);
            setTextSize(1);
            writeString(text1);
            
            sprintf(text1, "p term: %f    ", kp_value * error);
            setCursor(200, 10);
            setTextColor2(WHITE, BLACK);
            setTextSize(1);
            writeString(text1);
            
            sprintf(text1, "d term: %f    ", (kd_value * (error - prev_error)));
            setCursor(200, 20);
            setTextColor2(WHITE, BLACK);
            setTextSize(1);
            writeString(text1);

            sprintf(text1, "i term: %f    ", (ki_value * i_term));
            setCursor(200, 30);
            setTextColor2(WHITE, BLACK);
            setTextSize(1);
            writeString(text1);

            sprintf(text1, "pwm: %d    ", (pwm_control));
            setCursor(200, 40);
            setTextColor2(WHITE, BLACK);
            setTextSize(1);
            writeString(text1);

            

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
    PT_YIELD_usec(40000);
    PT_END(pt);
}

// User input thread. User can change draw speed

static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char classifier ;
    static float test_in ;
    static char cmd;
    
    while(1) {
        sprintf(pt_serial_out_buffer, "what to change?: ");
        serial_write ;
        serial_read ;
        sscanf(pt_serial_in_buffer,"%c", &cmd) ;
        switch (cmd) {
            case 'a':
                sprintf(pt_serial_out_buffer, "desired angle in degree: ");
                serial_write ;
                // spawn a thread to do the non-blocking serial read
                serial_read ;
                // convert input string to number
                sscanf(pt_serial_in_buffer,"%f", &test_in) ;
                reference_angle = float2fix15(test_in);
                break;
            case 'i':
                //Type in ki value
                sprintf(pt_serial_out_buffer, "input Ki value: ");
                serial_write ;
                // spawn a thread to do the non-blocking serial read
                serial_read ;
                // convert input string to number
                sscanf(pt_serial_in_buffer,"%f", &test_in) ;
                ki_value_set = test_in;
                break;
            case 'p':
                // Type in kp value
                sprintf(pt_serial_out_buffer, "input Kp value: ");
                serial_write ;
                // spawn a thread to do the non-blocking serial read
                serial_read ;
                // convert input string to number
                sscanf(pt_serial_in_buffer,"%f", &test_in) ;
                kp_value = test_in;
                break;
            case 'd':
                // Type in kd value
                sprintf(pt_serial_out_buffer, "input Kd value: ");
                serial_write ;
                // spawn a thread to do the non-blocking serial read
                serial_read ;
                // convert input string to number
                sscanf(pt_serial_in_buffer,"%f", &test_in) ;
                kd_value = test_in;
                break;
            default:
                sprintf(pt_serial_out_buffer, "not correct command ");
                serial_write ;
                break;
        }

    }
    PT_END(pt) ;
}
static int time = 0;
static int flag = 0;
static int seq = 0;

static PT_THREAD (protothread_button(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);
    while (1) {
        
        button_pressing();
        // reset state, set number of balls to max and reset histogram
        if (button_control != button_control_prev) {
            // printf("button:%d,%d\n", button_control, button_control_prev);
            button_control_prev = button_control;
            if (button_control == 0) {
               flag = 1;
               seq = 0;
               time = time_us_32();
               reference_angle = int2fix15(90);
            }
            else if (button_control == 1) {
                reference_angle = divfix(int2fix15(0), oneeightyoverpi);
            }
        }
        // yield for necessary amount of time
        PT_YIELD_usec(40000);
    }
    PT_END(pt);
}

static PT_THREAD (protothread_run_sequence(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);
    while (1) {
        if (flag) {
            if (seq == 0 && time_us_32() - time >= 5000000) {
                seq ++;
                time = time_us_32();
                reference_angle = int2fix15(120);
            }
            else if (seq == 1 && time_us_32() - time >= 5000000) {
                seq ++;
                time = time_us_32();
                reference_angle = int2fix15(60);
            }
            else if (seq == 2 && time_us_32() - time >= 5000000) {
                seq ++;
                time = time_us_32();
                reference_angle = int2fix15(90);
            }
            else if (seq == 3) {
                flag = 0;
            }
        }
        // yield for necessary amount of time
        PT_YIELD_usec(5000);
    }
    PT_END(pt);
}


// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_add_thread(protothread_button);
    pt_add_thread(protothread_run_sequence);
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
    // initialize button gpio
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);

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
    // Invert channel A
    pwm_set_output_polarity(slice_num, 1, 0);
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
