
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
#include "TFTMaster.h" //The TFT Master library
#include "hardware/dma.h"
#include "hardware/spi.h" //The hardware SPI library
#include "hardware/sync.h"
// Include protothreads
#include "pico/multicore.h"
#include "pt_cornell_rp2040_v1_3.h"

#define size 1
#define FRAME_RATE 33000
#define WIDTH 240
#define HEIGHT 320
#define MAX_ITER 2000


volatile int alive_count = 0;
volatile int alive = 0;
int count;
static char text1[40];
int frequencies[4] = {261, 293, 329, 392};
uint8_t menu_choice = 0;

static unsigned char cell_array[240/size][320/size] ;
static char cell_array_next[240/size][320/size] ;
uint8_t start_init = 1; // flag to indicate if the initial state is set

// mouse control
#define PIN_LEFT 8
#define PIN_UP 9
#define PIN_DOWN 10
#define PIN_RIGHT 11
#define PIN_CONFIRM 12
#define BUTTON_PIN 14

int curser_x = 120;
int curser_y = 160;

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
static int button_control = 0;
static int up_control = 0;
static int down_control = 0;
static int left_control = 0;
static int right_control = 0;
static int confirm_control = 0;
static int button_control_prev = 0;

int previous_left, previous_right, previous_up, previous_down = 0;

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
        // frequency = (int)(1000);
        

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



void menu() {
    if (button_control == -1) {

        
        tft_setTextSize(1);
        for (int i = 0; i < 3; i++) {
            
            tft_setCursor(10, 10 + i * 10);

            if (i == menu_choice) {
                // Highlight this line
                tft_setTextColor2(ILI9340_BLACK, ILI9340_WHITE);  // White background
                sprintf(text1, "→ %d. ", i+1);
            } else {
                tft_setTextColor2(ILI9340_WHITE, ILI9340_BLACK);  // Normal
                sprintf(text1, "  %d. ", i+1);
            }

            if (i == 0) strcat(text1, "Conway Game of Life - Pi");
            else if (i == 1) strcat(text1, "Conway Game of Life - Random");
            else if (i == 2) strcat(text1, "Mandelbrot Set");

            tft_writeString(text1);
        }
    }
}
void drawcell (int x, int y, int value) {
    for (int i = 0; i < size; i++){
        for (int j = 0; j < size; j++){
            tft_drawPixel(size*x+i, size*y+j, value*ILI9340_WHITE);
        }
    }
    cell_array[x][y] = value;
}

// void mouse_control(){
//     if (button_control == 1 || button_control == 2){
//         if (up_control == 1) {
//             curser_y -= 1;
//             if (cell_array[curser_x][curser_y] == 0){
//                 drawcell(curser_x, curser_y, 1);
//             }
//         }
//         if (down_control == 1) {
//             curser_y += 1;
//             if (cell_array[curser_x][curser_y] == 0){
//                 drawcell(curser_x, curser_y, 1);
//             }
//         }
//         if (left_control == 1) {
//             curser_x -= 1;
//             if (cell_array[curser_x][curser_y] == 0){
//                 drawcell(curser_x, curser_y, 1);
//             }
//         }
//         if (right_control == 1) {
//             curser_x += 1;
//             if (cell_array[curser_x][curser_y] == 0){
//                 drawcell(curser_x, curser_y, 1);
//             }
//         }

//     }

// }

void draw_curser( ){
    if (button_control == 1 || button_control == 0){
        tft_drawPixel(curser_x, curser_y, ILI9340_YELLOW);
        drawcell(curser_x,curser_y,1);
    }
    else if (button_control == 2){
        tft_drawPixel(curser_x, curser_y, ILI9340_YELLOW);
    }
}

void mandelbrot() {
    if(button_control == 2) {
        tft_fillScreen(ILI9340_BLACK);
    
        float Zre, Zim, Cre, Cim ;
        float Zre_sq, Zim_sq ;
        int n, i, j ;
        for (i = 0; i < WIDTH; ++i) {
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
                if (n >= MAX_ITER) tft_drawPixel(i, j, ILI9340_BLACK) ;
                else if (n >= (MAX_ITER>>1)) tft_drawPixel(i, j, ILI9340_WHITE) ;
                else if (n >= (MAX_ITER>>2)) tft_drawPixel(i, j, ILI9340_YELLOW) ;
                else if (n >= (MAX_ITER>>3)) tft_drawPixel(i, j, ILI9340_MAGENTA) ;
                else if (n >= (MAX_ITER>>4)) tft_drawPixel(i, j, ILI9340_RED) ;
                else if (n >= (MAX_ITER>>5)) tft_drawPixel(i, j, ILI9340_BLUE) ;
                else if (n >= (MAX_ITER>>6)) tft_drawPixel(i, j, ILI9340_MAGENTA) ;
                else tft_drawPixel(i, j, ILI9340_CYAN) ;
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
        button_control += (button_control == 3)?-3:1; //remainder 1:,2:,3
        button_state_switch = BUTTON_PRESSED;
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
            
            curser_x -= previous_left;
            previous_left *= 2;
            previous_down = 1;
            previous_up = 1;
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
            previous_right *= 2;
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
            previous_up *= 2;
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
            previous_down *= 2;
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
        for (int i = 0; i < 240/size; i++){
            for(int j = 0; j < 320/size; j++){
                srand(time_us_32()); // Use microsecond timer to seed
                int rand_bit = rand() % 2;
                drawcell(i, j, rand_bit);
                
            }
        }
    }
}

void pi_initial(){
    int x_offset[6] = {50, 120, 50, 120, 50, 120};    // starting x position
    int y_offset[6] = {70, 70, 150, 150, 230, 230};    // starting y position
    int height = 50;      // height of vertical legs
    int bar_width = 40;   // width of the top bar
    if (button_control == 0 && start_init) {
        start_init = 0;
    for (int i = 0; i < 6; i++){
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
    if (x > 0 & y > 0 & x < (240/size-1) & y < (320/size-1)) {
        cnt = cell_array[x-1][y-1] + cell_array[x-1][y]
                + cell_array[x-1][y+1] + cell_array[x][y-1] + cell_array[x][y+1]
                + cell_array[x+1][y-1] + cell_array[x+1][y] + cell_array[x+1][y+1];
    }
    else if ( x == 0 && y == 0) {
        cnt = cell_array[x][y+1] + cell_array[x+1][y] + cell_array[x+1][y+1];
    }
    else if ( x == (240/size-1) && y == (320/size-1)) {
        cnt = cell_array[x-1][y-1] + cell_array[x-1][y] + cell_array[x][y-1];
    }
    else if ( x == 0 && y == (320/size-1)) {
        cnt = cell_array[x][y-1] + cell_array[x+1][y-1] + cell_array[x+1][y];
    }
    else if ( x == (240/size-1) && y == 0) {
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
    else if (x == (240/size-1)) {
        cnt = cell_array[x-1][y-1] + cell_array[x-1][y]
                + cell_array[x-1][y+1] + cell_array[x][y-1] + cell_array[x][y+1];
    }
    else if (y == (320/size-1)) {
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

    for (int i = 0; i< (240/size); i++){
        for (int j = 0; j < (320/size); j++){
            alive = is_alive (i,j);
            if ( alive )
                alive_count++;

        }
    }
    count = alive_count;
}

void update_cell(){
    for (int i = 0; i< (240/size); i++){
        for (int j = 0; j < (320/size); j++){
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
        alive_count = 0;
        if (button_control != button_control_prev) {
            button_control_prev = button_control;
            tft_fillScreen(ILI9340_BLACK);
            memset(cell_array, 0, sizeof(cell_array));
            memset(cell_array_next, 0, sizeof(cell_array_next));
            start_init = 1;
            if(button_control == 2)
                mandelbrot();
        }

        pi_initial();
        random_initial();
        update_alive();
        update_cell(); 
        draw_curser();
        
        
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
        menu();

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
        // mouse_control();
        // printf("x: %d, y: %d", curser_x, curser_y);
        left_control = 0;
        right_control = 0;
        up_control = 0;
        down_control = 0;
        confirm_control = 0;

        

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

    tft_init_hw(); //Initialize the hardware for the TFT
    tft_begin(); //Initialize the TFT
    tft_fillScreen(ILI9340_BLACK); //Fill the entire screen with black colour
    // initialize the array 
    float x_min = -2.0, x_max = 1.0;
    float y_min = -2.0, y_max = 1.0;
    for (int i = 0; i < WIDTH; i++) {
        x[i] = x_min + (x_max - x_min) * i / (WIDTH - 1);
    }
    
    for (int i = 0; i < HEIGHT; i++) {
        y[i] = y_min + (y_max - y_min) * i / (HEIGHT - 1);
    }
    
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
    // pt_add_thread(protothread_mouse);


    // start scheduler
    pt_schedule_start ;
   
}
