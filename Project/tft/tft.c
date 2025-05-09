
/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green 
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */
#include <stdio.h> //The standard C library
#include <stdlib.h> //C stdlib
#include <string.h>
#include "pico/stdlib.h" //Standard library for Pico
#include <math.h> //The standard math library
#include "hardware/gpio.h" //The hardware GPIO library
#include "pico/time.h" //The pico time library
#include "hardware/irq.h" //The hardware interrupt library
#include "hardware/pwm.h" //The hardware PWM library
#include "hardware/pio.h" //The hardware PIO library
// #include "TFTMaster.h" //The TFT Master library
#include "hardware/dma.h"
#include "hardware/spi.h" //The hardware SPI library
#include "hardware/sync.h"
// Include protothreads
#include "pico/multicore.h"
#include "pt_cornell_rp2040_v1_3.h"
#include "vga16_graphics.h"
#include "hardware/adc.h"


#define size 2
#define FRAME_RATE 33000
#define WIDTH 640
#define HEIGHT 480
#define MAX_ITER 500
#define WIDTH_DISPLAY 528


volatile int alive_count = 0;
volatile int alive = 0;
int count;
static char text1[40];
int frequencies[4] = {261, 293, 329, 392};
uint8_t menu_choice = 0;

float x_min = -2.0, x_max = 1.0;
float y_min = -2.0, y_max = 1.0;

static unsigned char cell_array[WIDTH_DISPLAY/size][HEIGHT/size] ;
static char cell_array_next[WIDTH_DISPLAY/size][HEIGHT/size] ;
uint8_t start_init = 1; // flag to indicate if the initial state is set

// mouse control
#define PIN_LEFT 8
#define PIN_UP 9
#define PIN_DOWN 10
#define PIN_RIGHT 11
#define PIN_CONFIRM 12
#define BUTTON_PIN 14
#define ADC_PIN 26

#define default_curser_x 120
#define default_curser_y 160

int curser_x = default_curser_x;
int curser_y = default_curser_y;

// potentiometer
#define DEFAULT_SIZE 5
#define min_zoom_size 1
#define max_zoom_size 10
#define zoom_unit_size 11

int potentio_read;
int potentio_read_prev;
int zoom_size_prev;
int zoom_size_set = DEFAULT_SIZE;
int zoom_start_x;
int zoom_start_y;
int zoom_end_x;
int zoom_end_y;

typedef enum {
    RESET,
    BUTTON_NOT_PRESSED,
    BUTTON_MAYBE_PRESSED,
    BUTTON_PRESSED,
    BUTTON_MAYBE_NOT_PRESSED
} ButtonState;

static ButtonState button_state_switch = RESET;
static ButtonState button_state_left = RESET;
static ButtonState button_state_right = RESET;
static ButtonState button_state_up = RESET;
static ButtonState button_state_down = RESET;
static ButtonState button_state_confirm = RESET;
int button_value_switch = -1;
int button_value_up = -1;
int button_value_down = -1;
int button_value_left = -1;
int button_value_right = -1;
int button_value_confirm = -1;
static int possible_switch = -1;
static int possible_left = -1;
static int possible_right = -1;
static int possible_up = -1;
static int possible_down = -1;
static int possible_confirm = -1;
static int button_control = -1;
static int up_control = 0;
static int down_control = 0;
static int left_control = 0;
static int right_control = 0;
static int confirm_control = 0;
static int button_control_prev = -1;

float previous_left = 1, previous_right = 1, previous_up = 1, previous_down = 1;
float acc = 1.2;

float x[WIDTH] = {0};
float y[HEIGHT] = {0};
int count = 0;

//////////
// Beep
// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ 0

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))


//Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0  // 2^32 (a constant)
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)

// the DDS units - core 0
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs ;

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size] ;

// Values output to DAC
int DAC_output_0 ;
int DAC_output_1 ;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1) ;    // maximum amplitude
fix15 attack_inc ;                      // rate at which sound ramps up
fix15 decay_inc ;                       // rate at which sound ramps down
fix15 current_amplitude_0 = 0 ;         // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0 ;         // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME             250
#define DECAY_TIME              250
#define SUSTAIN_TIME            10000
#define BEEP_DURATION           6500
#define BEEP_REPEAT_INTERVAL    50000
#define TONE_DURATION           10000

// State machine variables
volatile unsigned int count_0 = 0 ;

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define LED      25
#define SPI_PORT spi0

//GPIO for timing the ISR
#define ISR_GPIO 2

static void alarm_irq(void) {
    int frequency;
    // printf("\n alarm: %d", STATE);
    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1) ;

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;
    
    if ( button_control == 0 || button_control == 1 ) {
        // DDS phase and sine table lookup
        frequency = (int)(sqrt(count)*10);
        

        phase_accum_main_0 += (int)((frequency*two32)/Fs) ;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0>>24])) + 2048 ;// Add 2048 to center it from range (-2048,2048) to (0,4096)

        // Ramp up amplitude (ATTACK_TIME = 250)
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
        }
        // Ramp down amplitude (DECAY_TIME = 250)
        else if (count_0 > BEEP_DURATION - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xfff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        count_0 += 1 ;
        
        if (count_0 == BEEP_DURATION) {
            count_0 = 0 ;
            current_amplitude_0 = 0 ;
            
        }
    }
    else if ( button_control == 2 ) {
        static int current_freq_idx = 0;

        // Set new frequency at start of beep
        frequency = frequencies[current_freq_idx];

        // DDS phase and sine table lookup
        phase_accum_main_0 += (int)((frequency * two32) / Fs);
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
                        sin_table[phase_accum_main_0 >> 24])) + 2048;

        // Ramp up
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 += attack_inc;
        }
        // Ramp down
        else if (count_0 > BEEP_DURATION - DECAY_TIME) {
            current_amplitude_0 -= decay_inc;
        }

        // Mask with DAC control bits and send
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xfff));
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);

        count_0 += 1;

        if (count_0 == BEEP_DURATION) {
            count_0 = 0;
            current_amplitude_0 = 0;

            // Move to next frequency in the list (loop around)
            current_freq_idx = (current_freq_idx + 1) % 4;
        }
    }
    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0) ;
}

void change_zoom_size(){
    if (button_control == 2 ){
        adc_select_input(0);
        potentio_read = adc_read();
        // printf("pot:%d\n", potentio_read);
        zoom_size_prev = zoom_size_set;
        if (potentio_read != potentio_read_prev) {
        zoom_size_set = (int)min_zoom_size + (int)((potentio_read-13)*(max_zoom_size-min_zoom_size)/(4095-13));
        zoom_size_set = zoom_size_prev + (int)((zoom_size_set - zoom_size_prev)>>1);
        potentio_read_prev = potentio_read;
        }

    }
}


void drawcell (int x, int y, int value) {
    for (int i = 0; i < size; i++){
        for (int j = 0; j < size; j++){
            drawPixel(size*x+i, size*y+j, value*WHITE);
        }
    }
    cell_array[x][y] = value;
}

int live_x[50] = {0};
int live_y[50] = {0};
int live_count = 0;

void draw_curser( ){
    if (button_control == 1 || button_control == 0) {
        if (live_x[live_count] != curser_x*size || live_y[live_count] != curser_y*size) {
            printf("move cursor\n");
            drawPixel(live_x[live_count]-1,  live_y[live_count]-1, BLACK);
            drawPixel(live_x[live_count]+1,  live_y[live_count]+1, BLACK);
            drawPixel(live_x[live_count]+1,  live_y[live_count]-1, BLACK);
            drawPixel(live_x[live_count]-1,  live_y[live_count]+1, BLACK);
            live_count++;
            live_x[live_count] = curser_x*size;
            live_y[live_count] = curser_y*size;
            if (live_count == 50) {
                live_count --;
            }
        }
        drawPixel(curser_x*size, curser_y*size, YELLOW);
        drawPixel(curser_x*size-1, curser_y*size-1, YELLOW);
        drawPixel(curser_x*size+1, curser_y*size+1, YELLOW);
        drawPixel(curser_x*size+1, curser_y*size-1, YELLOW);
        drawPixel(curser_x*size-1, curser_y*size+1, YELLOW);
    }
    // else if (button_control == 2){
    //     drawPixel(curser_x, curser_y, YELLOW);
    // }
}

void draw_zoomin(){
    if (button_control == 2){
        // if(potentio_read)
        int zoom_length = zoom_unit_size * zoom_size_set;
        // printf("length:%d\n", zoom_length);
        drawRect(curser_x, curser_y, zoom_length, zoom_length, MAGENTA );
        zoom_start_x = curser_x;
        zoom_start_y = curser_y;
        zoom_end_x = curser_x + zoom_length;
        zoom_end_y = curser_y + zoom_length;
        
    }
}

void menu() {
    // printf("%d\n", button_control);
    if (button_control == -1) {
        sprintf(text1, "1.Conway Game of Life - Pi");
        setCursor(30, 10);
        if (menu_choice == 0) {
            setTextColor2(BLACK, WHITE);
        } else {
            setTextColor2(WHITE, BLACK);
        }
        setTextSize(1);
        writeString(text1);
    
        sprintf(text1, "2.Conway Game of Life - Random");
        setCursor(30, 20);
        // setTextColor2(WHITE, BLACK);
        if (menu_choice == 1) {
            setTextColor2(BLACK, WHITE);
        } else {
            setTextColor2(WHITE, BLACK);
        }
        setTextSize(1);
        writeString(text1);

        sprintf(text1, "3.Mandelbrot Set");
        setCursor(30, 30);
        // setTextColor2(WHITE, BLACK);
        if (menu_choice == 2) {
            setTextColor2(BLACK, WHITE);
        } else {
            setTextColor2(WHITE, BLACK);
        }
        setTextSize(1);
        writeString(text1);

    }
}
const char* conway_title[] = { "Conway", "Game", "of", "Life" };
const char* mandelbrot_title[] = { "Mandel-","brot", "Set" };

void draw_title_area( const char* title_words[], int word_count ) {
    // Fill the rightmost 112-pixel-wide strip with white
    // fillRect(528,0,112,480,WHITE);

    // Display one word per line, left-aligned in the white strip
    for (int i = 0; i < word_count; i++) {
        sprintf(text1, "%s", title_words[i]);

        setCursor(530, 100 + i * 30); // vertical spacing, adjust as needed
        setTextColor2(BLACK, WHITE);
        setTextSize(2);
        writeString(text1);
    }
}

void draw_title(){
    if (button_control == 0 || button_control == 1){
        draw_title_area(conway_title, 4);
    }else if (button_control == 2){
        draw_title_area(mandelbrot_title, 3);
    }

}

void mandelbrot() {
    if(button_control == 2) {
        fillRect(0, 0, WIDTH_DISPLAY, HEIGHT, BLACK);
    
        float Zre, Zim, Cre, Cim ;
        float Zre_sq, Zim_sq ;
        int n, i, j ;
        for (i = 0; i < WIDTH_DISPLAY; ++i) {
            for (j = 0; j < HEIGHT; ++j) {
                Zre = Zim = Zre_sq = Zim_sq = 0;
                Cre = x[i] ;
                Cim = y[j] ;
                n = 0 ;
                while (Zre_sq + Zim_sq < 4 && n < MAX_ITER) {
                    Zim = 2 * Zre * Zim + Cim;
                    Zre = Zre_sq - Zim_sq + Cre;
                    Zre_sq = Zre * Zre;
                    Zim_sq = Zim * Zim;
                    n++;
                    // printf("n: %d\n", n);
                }
                if (n >= MAX_ITER) {drawPixel(i, j, BLACK) ;}
                else if (n >= (MAX_ITER>>1)) {drawPixel(i, j, WHITE); }
                else if (n >= (MAX_ITER>>2)) {drawPixel(i, j, YELLOW); }
                else if (n >= (MAX_ITER>>3)) {drawPixel(i, j, MED_GREEN); ;}
                else if (n >= (MAX_ITER>>4)) {drawPixel(i, j, RED); }
                else if (n >= (MAX_ITER>>5)) {drawPixel(i, j, DARK_ORANGE); }
                else if (n >= (MAX_ITER>>6)) {drawPixel(i, j, ORANGE); }
                else {drawPixel(i, j, PINK);}
                 
            }
        }
    }
}
// ====================================================================
// =                          button FSM                              =
// ====================================================================
void button_pressing_switch( )
{
    switch (button_state_switch)
    {

    case RESET:
        button_value_switch = gpio_get(BUTTON_PIN);
        button_state_switch = BUTTON_NOT_PRESSED;
    break;

    case BUTTON_NOT_PRESSED:
        if (button_value_switch == 0){
            button_value_switch = gpio_get(BUTTON_PIN);
            button_state_switch = BUTTON_NOT_PRESSED;
        }
    else{
        possible_switch = button_value_switch;
        button_value_switch = gpio_get(BUTTON_PIN);
        button_state_switch = BUTTON_MAYBE_PRESSED;
    }
    break;

    case BUTTON_MAYBE_PRESSED:
    if (button_value_switch == possible_switch){
        printf("switch button pressed\n");
        button_value_switch = gpio_get(BUTTON_PIN);
        // button_control += (button_control == 3)?-3:1; //remainder 1:,2:,3
        button_state_switch = BUTTON_PRESSED;
        if (button_control == -1) {
            menu_choice = (menu_choice + 1) % 3;
        }
        else {
            button_control = -1;
        }
    }
    else{
        button_value_switch = gpio_get(BUTTON_PIN);
        button_state_switch = BUTTON_NOT_PRESSED;
    }
    break;
    case BUTTON_PRESSED:
    if (button_value_switch == possible_switch){
        button_value_switch = gpio_get(BUTTON_PIN);
        button_state_switch = BUTTON_PRESSED;
        }
    else{
        button_value_switch = gpio_get(BUTTON_PIN);
        button_state_switch = BUTTON_MAYBE_NOT_PRESSED;
    }
    case BUTTON_MAYBE_NOT_PRESSED:
    if (button_value_switch == possible_switch) {
        button_value_switch = gpio_get(BUTTON_PIN);
        button_state_switch = BUTTON_PRESSED;
    }
    else {
        button_value_switch = gpio_get(BUTTON_PIN);
        button_state_switch = BUTTON_NOT_PRESSED;
    }
    break;

    default:
        button_value_switch = -1;
        possible_switch = -1;
    break;

    }
}

void button_pressing_left()
{
    switch (button_state_left)
    {
    case RESET:
        button_value_left = gpio_get(PIN_LEFT);
        button_state_left = BUTTON_NOT_PRESSED;
        break;

    case BUTTON_NOT_PRESSED:
        if (gpio_get(PIN_LEFT) == 0) {
            button_value_left = gpio_get(PIN_LEFT);
            button_state_left = BUTTON_NOT_PRESSED;
        }
        else {
            possible_left = button_value_left;
            button_value_left = gpio_get(PIN_LEFT);
            button_state_left = BUTTON_MAYBE_PRESSED;
        }
        break;

    case BUTTON_MAYBE_PRESSED:
        if (button_value_left == possible_left) {
            button_value_left = gpio_get(PIN_LEFT);
            printf("left button pressed\n");
            left_control = 1;
            // Optionally reset other controls
            right_control = 0;
            up_control = 0;
            down_control = 0;
            confirm_control = 0;
            
            curser_x -= previous_up;
            previous_up *= acc;
            previous_down = 1;
            previous_left = 1;
            previous_right = 1;

            button_state_left = BUTTON_PRESSED;
        }
        else {
            button_value_left = gpio_get(PIN_LEFT);
            button_state_left = BUTTON_NOT_PRESSED;
        }
        break;

    case BUTTON_PRESSED:
        if (button_value_left == possible_left) {
            button_value_left = gpio_get(PIN_LEFT);
            button_state_left = BUTTON_PRESSED;
        }
        else {
            button_value_left = gpio_get(PIN_LEFT);
            button_state_left = BUTTON_MAYBE_NOT_PRESSED;
        }
        break;

    case BUTTON_MAYBE_NOT_PRESSED:
        if (button_value_left == possible_left) {
            button_value_left = gpio_get(PIN_LEFT);
            button_state_left = BUTTON_PRESSED;
        }
        else {
            button_value_left = gpio_get(PIN_LEFT);
            button_state_left = BUTTON_NOT_PRESSED;
        }
        break;

    default:
        button_value_left = -1;
        possible_left = -1;
        break;
    }
}
void button_pressing_right()
{
    switch (button_state_right)
    {
    case RESET:
        button_value_right = gpio_get(PIN_RIGHT);
        button_state_right = BUTTON_NOT_PRESSED;
        break;

    case BUTTON_NOT_PRESSED:
        if (gpio_get(PIN_RIGHT) == 0) {
            button_value_right = gpio_get(PIN_RIGHT);
            button_state_right = BUTTON_NOT_PRESSED;
        }
        else {
            possible_right = button_value_right;
            button_value_right = gpio_get(PIN_RIGHT);
            button_state_right = BUTTON_MAYBE_PRESSED;
        }
        break;

    case BUTTON_MAYBE_PRESSED:
        if (button_value_right == possible_right) {
            button_value_right = gpio_get(PIN_RIGHT);
            printf("right button pressed\n");
            right_control = 1;
            // Optionally reset other controls
            left_control = 0;
            up_control = 0;
            down_control = 0;
            confirm_control = 0;

            curser_x += previous_right;
            previous_right *= acc;
            previous_down = 1;
            previous_up = 1;
            previous_left = 1;


            button_state_right = BUTTON_PRESSED;
        }
        else {
            button_value_right = gpio_get(PIN_RIGHT);
            button_state_right = BUTTON_NOT_PRESSED;
        }
        break;

    case BUTTON_PRESSED:
        if (button_value_right == possible_right) {
            button_value_right = gpio_get(PIN_RIGHT);
            button_state_right = BUTTON_PRESSED;
        }
        else {
            button_value_right = gpio_get(PIN_RIGHT);
            button_state_right = BUTTON_MAYBE_NOT_PRESSED;
        }
        break;

    case BUTTON_MAYBE_NOT_PRESSED:
        if (button_value_right == possible_right) {
            button_value_right = gpio_get(PIN_RIGHT);
            button_state_right = BUTTON_PRESSED;
        }
        else {
            button_value_right = gpio_get(PIN_RIGHT);
            button_state_right = BUTTON_NOT_PRESSED;
        }
        break;

    default:
        button_value_right = -1;
        possible_right = -1;
        break;
    }
}

void button_pressing_up()
{
    switch (button_state_up)
    {
    case RESET:
        button_value_up = gpio_get(PIN_UP);
        button_state_up = BUTTON_NOT_PRESSED;
        break;

    case BUTTON_NOT_PRESSED:
        if (gpio_get(PIN_UP) == 0) {
            button_value_up = gpio_get(PIN_UP);
            button_state_up = BUTTON_NOT_PRESSED;
        }
        else {
            possible_up = button_value_up;
            button_value_up = gpio_get(PIN_UP);
            button_state_up = BUTTON_MAYBE_PRESSED;
        }
        break;

    case BUTTON_MAYBE_PRESSED:
        if (button_value_up == possible_up) {
            button_value_up = gpio_get(PIN_UP);
            printf("up button pressed\n");
            up_control = 1;
            // Optionally reset other controls
            left_control = 0;
            right_control = 0;
            down_control = 0;
            confirm_control = 0;

            curser_y -= previous_up;
            previous_up *= acc;
            previous_down = 1;
            previous_left = 1;
            previous_right = 1;

            button_state_up = BUTTON_PRESSED;
            
        }
        else {
            button_value_up = gpio_get(PIN_UP);
            button_state_up = BUTTON_NOT_PRESSED;
        }
        break;

    case BUTTON_PRESSED:
        if (button_value_up == possible_up) {
            button_value_up = gpio_get(PIN_UP);
            button_state_up = BUTTON_PRESSED;
        }
        else {
            button_value_up = gpio_get(PIN_UP);
            button_state_up = BUTTON_MAYBE_NOT_PRESSED;
        }
        break;

    case BUTTON_MAYBE_NOT_PRESSED:
        if (button_value_up == possible_up) {
            button_value_up = gpio_get(PIN_UP);
            button_state_up = BUTTON_PRESSED;
        }
        else {
            button_value_up = gpio_get(PIN_UP);
            button_state_up = BUTTON_NOT_PRESSED;
        }
        break;

    default:
        button_value_up = -1;
        possible_up = -1;
        break;
    }
}
void button_pressing_down()
{
    switch (button_state_down)
    {
    case RESET:
        button_value_down = gpio_get(PIN_DOWN);
        button_state_down = BUTTON_NOT_PRESSED;
        break;

    case BUTTON_NOT_PRESSED:
        if (gpio_get(PIN_DOWN) == 0) {
            button_value_down = gpio_get(PIN_DOWN);
            button_state_down = BUTTON_NOT_PRESSED;
        }
        else {
            possible_down = button_value_down;
            button_value_down = gpio_get(PIN_DOWN);
            button_state_down = BUTTON_MAYBE_PRESSED;
        }
        break;

    case BUTTON_MAYBE_PRESSED:
        if (button_value_down == possible_down) {
            button_value_down = gpio_get(PIN_DOWN);
            printf("down button pressed\n");
            down_control = 1;
            // Optionally reset other controls
            left_control = 0;
            right_control = 0;
            up_control = 0;
            confirm_control = 0;

            curser_y += previous_down;
            previous_down *= acc;
            previous_left = 1;
            previous_up = 1;
            previous_right = 1;

            button_state_down = BUTTON_PRESSED;
        }
        else {
            button_value_down = gpio_get(PIN_DOWN);
            button_state_down = BUTTON_NOT_PRESSED;
        }
        break;

    case BUTTON_PRESSED:
        if (button_value_down == possible_down) {
            button_value_down = gpio_get(PIN_DOWN);
            button_state_down = BUTTON_PRESSED;
        }
        else {
            button_value_down = gpio_get(PIN_DOWN);
            button_state_down = BUTTON_MAYBE_NOT_PRESSED;
        }
        break;

    case BUTTON_MAYBE_NOT_PRESSED:
        if (button_value_down == possible_down) {
            button_value_down = gpio_get(PIN_DOWN);
            button_state_down = BUTTON_PRESSED;
        }
        else {
            button_value_down = gpio_get(PIN_DOWN);
            button_state_down = BUTTON_NOT_PRESSED;
        }
        break;

    default:
        button_value_down = -1;
        possible_down = -1;
        break;
    }
}

uint8_t change_man = 0;

void button_pressing_confirm()
{
    switch (button_state_confirm)
    {
    case RESET:
        button_value_confirm = gpio_get(PIN_CONFIRM);
        button_state_confirm = BUTTON_NOT_PRESSED;
        break;

    case BUTTON_NOT_PRESSED:
        if (gpio_get(PIN_CONFIRM) == 0) {
            button_value_confirm = gpio_get(PIN_CONFIRM);
            button_state_confirm = BUTTON_NOT_PRESSED;
        }
        else {
            possible_confirm = button_value_confirm;
            button_value_confirm = gpio_get(PIN_CONFIRM);
            button_state_confirm = BUTTON_MAYBE_PRESSED;
        }
        break;

    case BUTTON_MAYBE_PRESSED:
        if (button_value_confirm == possible_confirm) {
            button_value_confirm = gpio_get(PIN_CONFIRM);
            printf("confirm button pressed\n");
            confirm_control = 1;
            // Optionally reset other controls
            left_control = 0;
            right_control = 0;
            up_control = 0;
            down_control = 0;
            // printf("button_ctrl%d", button_control);
            if (button_control == -1) button_control = menu_choice;
            else if (button_control == 2) change_man = 1;
            else if (button_control == 0 || button_control == 1) {
                printf("live_cnt%d", live_count);
                for(int j = 0; j < live_count; j++) {
                    drawPixel(live_x[live_count]+1, live_y[live_count]+1, BLACK);
                    drawPixel(live_x[live_count]+1, live_y[live_count]-1, BLACK);
                    drawPixel(live_x[live_count]-1, live_y[live_count]+1, BLACK);
                    drawPixel(live_x[live_count], live_y[live_count], BLACK);
                    drawcell(live_x[j]/size, live_y[j] /size, 1);
                    live_x[j] = -1;
                    live_y[j] = -1;
                }   
                live_count = 0;
            }
           
            button_state_confirm = BUTTON_PRESSED;
        }
        else {
            button_value_confirm = gpio_get(PIN_CONFIRM);
            button_state_confirm = BUTTON_NOT_PRESSED;
        }
        break;

    case BUTTON_PRESSED:
        if (button_value_confirm == possible_confirm) {
            button_value_confirm = gpio_get(PIN_CONFIRM);
            button_state_confirm = BUTTON_PRESSED;
        }
        else {
            button_value_confirm = gpio_get(PIN_CONFIRM);
            button_state_confirm = BUTTON_MAYBE_NOT_PRESSED;
        }
        break;

    case BUTTON_MAYBE_NOT_PRESSED:
        if (button_value_confirm == possible_confirm) {
            button_value_confirm = gpio_get(PIN_CONFIRM);
            button_state_confirm = BUTTON_PRESSED;
        }
        else {
            button_value_confirm = gpio_get(PIN_CONFIRM);
            button_state_confirm = BUTTON_NOT_PRESSED;
        }
        break;

    default:
        button_value_confirm = -1;
        possible_confirm = -1;
        break;
    }
}




// ==============================================================================
//                       button fsm end
// ==============================================================================
void random_initial(){
    if ( button_control == 1 && start_init) {
        start_init = 0;
        for (int i = 0; i < WIDTH_DISPLAY/size; i++){
            for(int j = 0; j < HEIGHT/size; j++){
                srand(time_us_32()); // Use microsecond timer to seed
                int rand_bit = rand() % 2;
                drawcell(i, j, rand_bit);
                
            }
        }
    }
}

void pi_initial(){
    int x_offset[4] = {50, 120, 50, 120};    // starting x position
    int y_offset[4] = {70, 70, 150, 150};    // starting y position
    int height = 50;      // height of vertical legs
    int bar_width = 40;   // width of the top bar
    if (button_control == 0 && start_init) {
        start_init = 0;
    for (int i = 0; i < 4; i++){
        int left_x = x_offset[i];
        int right_x = x_offset[i] + bar_width - 1;
        
        // Top horizontal bar (solid)
        for (int x = 0; x < bar_width; x++) {
            drawcell(x_offset[i] + x, y_offset[i], 1);
        }

        // Left vertical leg
        for (int y = 1; y < height - 1; y++) {
            drawcell(left_x, y_offset[i] + y, 1);
        }
        // Bottom outward curve for left leg
        drawcell(left_x - 1, y_offset[i] + height - 2, 1);
        drawcell(left_x - 2, y_offset[i] + height - 1, 1);

        // Right vertical leg
        for (int y = 1; y < height - 1; y++) {
            drawcell(right_x, y_offset[i] + y, 1);
        }
        // Bottom outward curve for right leg
        drawcell(right_x + 1, y_offset[i] + height - 2, 1);
        drawcell(right_x + 2, y_offset[i] + height - 1, 1);
    }
    }
}



int is_alive(int x, int y) {
    int cnt = 0;
    if (x > 0 & y > 0 & x < (WIDTH_DISPLAY/size-1) & y < (HEIGHT/size-1)) {
        cnt = cell_array[x-1][y-1] + cell_array[x-1][y]
                + cell_array[x-1][y+1] + cell_array[x][y-1] + cell_array[x][y+1]
                + cell_array[x+1][y-1] + cell_array[x+1][y] + cell_array[x+1][y+1];
    }
    else if ( x == 0 && y == 0) {
        cnt = cell_array[x][y+1] + cell_array[x+1][y] + cell_array[x+1][y+1];
    }
    else if ( x == (WIDTH_DISPLAY/size-1) && y == (HEIGHT/size-1)) {
        cnt = cell_array[x-1][y-1] + cell_array[x-1][y] + cell_array[x][y-1];
    }
    else if ( x == 0 && y == (HEIGHT/size-1)) {
        cnt = cell_array[x][y-1] + cell_array[x+1][y-1] + cell_array[x+1][y];
    }
    else if ( x == (WIDTH_DISPLAY/size-1) && y == 0) {
        cnt = cell_array[x-1][y] + cell_array[x-1][y+1] + cell_array[x][y+1];
    }
    else if ( x == 0) {
        cnt = cell_array[x][y-1] + cell_array[x][y+1]
                + cell_array[x+1][y-1] + cell_array[x+1][y] + cell_array[x+1][y+1];
    }
    else if ( y == 0) {
        cnt = cell_array[x-1][y] + cell_array[x-1][y+1] 
            + cell_array[x][y+1] + cell_array[x+1][y] + cell_array[x+1][y+1];
    }
    else if (x == (WIDTH_DISPLAY/size-1)) {
        cnt = cell_array[x-1][y-1] + cell_array[x-1][y]
                + cell_array[x-1][y+1] + cell_array[x][y-1] + cell_array[x][y+1];
    }
    else if (y == (HEIGHT/size-1)) {
        cnt = cell_array[x-1][y-1] + cell_array[x-1][y]
                + cell_array[x][y-1] + cell_array[x+1][y-1] + cell_array[x+1][y];
    }
    if (cnt == 3 && cell_array[x][y] == 0) {
        cell_array_next[x][y] = 1;
    }
    else if (cnt > 3 || cnt < 2) {
        cell_array_next[x][y] = 0;
    }

    return cell_array_next[x][y];
}


void update_alive(){

    for (int i = 0; i< (WIDTH_DISPLAY/size); i++){
        for (int j = 0; j < (HEIGHT/size); j++){
            alive = is_alive (i,j);
            if ( alive )
                alive_count++;

        }
    }
    count = alive_count;
}

void update_cell(){
    for (int i = 0; i< (WIDTH_DISPLAY/size); i++){
        for (int j = 0; j < (HEIGHT/size); j++){
            if (cell_array[i][j] != cell_array_next[i][j]) {
                if (cell_array_next[i][j]){
                    drawcell(i, j, 1);
                }
                else{
                    drawcell(i, j, 0);
                }
            }
            
            cell_array[i][j] =  cell_array_next[i][j];
        }
    }
}
// core 0: update alive
// static PT_THREAD (protothread_update_alive(struct pt *pt))
// {
//     PT_BEGIN(pt);

//     static int begin_time ;
//     static int spare_time ;

//     first_generation();
//     while(1){
//         update_alive();
//         PT_YIELD_usec(1000) ;
//     }
//     PT_END(pt);
// } // alive thread

// core 1: animation
static PT_THREAD (protothread_anim(struct pt *pt))
{
    PT_BEGIN(pt);

    static int begin_time ;
    static int spare_time ;
    // first_generation();
    // random_initial();

    while(1){
        // button_value = gpio_get(BUTTON_PIN);
        // printf("button_control: %d, prev:%d\n", button_control, button_control_prev);
        if (button_control == -1) {
            menu();
            PT_YIELD_usec(5000);
            continue;
        }
        alive_count = 0;
        if (button_control != button_control_prev) {
            x_min = -2;
            x_max = 1;
            y_min = -1;
            y_max = 1;
            button_control_prev = button_control;
            fillRect(0, 0, WIDTH, HEIGHT, BLACK);
            memset(cell_array, 0, sizeof(cell_array));
            memset(cell_array_next, 0, sizeof(cell_array_next));
            start_init = 1;
            curser_x = default_curser_x;
            curser_y = default_curser_y;
            fillRect(528,0,112,480,WHITE);
            if(button_control == 2){
                for (int i = 0; i < WIDTH_DISPLAY; i++) {
                    x[i] = x_min + (x_max - x_min) * i / (WIDTH_DISPLAY - 1);
                }
                
                for (int i = 0; i < HEIGHT; i++) {
                    y[i] = y_min + (y_max - y_min) * i / (HEIGHT - 1);
                }
                mandelbrot();
            }
                
        }
        // printf("change:%d\n", change_man);
        if (change_man) {
        
            float x_c, y_c;
            float x1, x2 = 0;
            float y1, y2 = 0;
            x1 = x_min + zoom_start_x * (x_max - x_min) / WIDTH_DISPLAY;
            x2 = x_min + zoom_end_x * (x_max - x_min) / WIDTH_DISPLAY;
            y1 = y_min + zoom_start_y * (y_max - y_min) / WIDTH_DISPLAY;
            y2 = y_min + zoom_end_y * (y_max - y_min) / WIDTH_DISPLAY;
            x_c = (x1 + x2) / 2;
            y_c = (y1 + y2) / 2;
            x_min = x_c - (x2 - x1) / 2;
            x_max = x_c + (x2 - x1) / 2;
            y_min = y_c - (y2 - y1) / 2;
            y_max = y_c + (y2 - y1) / 2;
            for (int i = 0; i < WIDTH_DISPLAY; i++) {
                x[i] = x_min + (x_max - x_min) * i / (WIDTH_DISPLAY - 1);
            }
            
            for (int i = 0; i < HEIGHT; i++) {
                y[i] = y_min + (y_max - y_min) * i / (HEIGHT - 1);
            }

            mandelbrot();
            change_man = 0;
        }

        pi_initial();
        random_initial();
        update_alive();
        update_cell(); 
        draw_curser();
        draw_zoomin();
        draw_title();
        
        
        PT_YIELD_usec(1000) ;
    }
    PT_END(pt);
} // animation thread

static PT_THREAD (protothread_btn(struct pt *pt))
{
    PT_BEGIN(pt);

    static int begin_time ;
    static int spare_time ;
    // first_generation();
    // random_initial();

    while(1){
        button_pressing_switch();
        button_pressing_up();
        button_pressing_down();
        button_pressing_right();
        button_pressing_left();
        button_pressing_confirm();

        // printf("control: %d, left: %d, up: %d, down: %d, right: %d, confirm: %d\n", button_control, left_control, up_control, down_control, right_control, confirm_control);


        alarm_irq();
        // button_value = gpio_get(BUTTON_PIN)

        PT_YIELD_usec(1000) ;
    }
    PT_END(pt);
} // btn thread

static PT_THREAD (protothread_mouse(struct pt *pt))
{
    PT_BEGIN(pt);

    static int begin_time ;
    static int spare_time ;
    // first_generation();
    // random_initial();

    while(1){
        change_zoom_size();

        PT_YIELD_usec(1000) ;
    }
    PT_END(pt);
} 

void core1_main(){
  // Add animation thread
    pt_add_thread(protothread_anim);

    pt_schedule_start ;

}


int main(){
    
    stdio_init_all(); //Initialize all of the present standard stdio types that are linked into the binary
    adc_init();
    adc_gpio_init(ADC_PIN);
    // initialize button gpio
    gpio_init(BUTTON_PIN);
    gpio_init(PIN_UP);
    gpio_init(PIN_DOWN);
    gpio_init(PIN_LEFT);
    gpio_init(PIN_RIGHT);
    gpio_init(PIN_CONFIRM);

    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_set_dir(PIN_UP, GPIO_IN);
    gpio_set_dir(PIN_DOWN, GPIO_IN);
    gpio_set_dir(PIN_LEFT, GPIO_IN);
    gpio_set_dir(PIN_RIGHT, GPIO_IN);
    gpio_set_dir(PIN_CONFIRM, GPIO_IN);

    initVGA(); //Initialize the hardware for the TFT
    // tft_begin(); //Initialize the TFT
    fillRect(0, 0, WIDTH, HEIGHT, BLACK); //Fill the entire screen with black colour
    // initialize the array 
    
    // Beep
    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC) ;
    gpio_set_dir(LDAC, GPIO_OUT) ;
    gpio_put(LDAC, 0) ;

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO) ;
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0) ;
    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME)) ;
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME)) ;

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
        sin_table[ii] = float2fix15(2047*sin((float)ii*6.283/(float)sine_table_size));
    }



    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM) ;
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq) ;
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true) ;
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;
    //Beep end

      // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(&core1_main);

    // add threads
    pt_add_thread(protothread_btn);
    // pt_add_thread(protothread_update_alive);
    pt_add_thread(protothread_mouse);


    // start scheduler
    pt_schedule_start ;
   
}
