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
#define BUTTON_PIN 14
#define WIDTH 240
#define HEIGHT 320
#define MAX_ITER 2000

static unsigned char cell_array[240/size][320/size] ;
static char cell_array_next[240/size][320/size] ;
uint8_t start_init = 1; // flag to indicate if the initial state is set
static char text1[40];

typedef enum {
    RESET,
    BUTTON_NOT_PRESSED,
    BUTTON_MAYBE_PRESSED,
    BUTTON_PRESSED,
    BUTTON_MAYBE_NOT_PRESSED
} ButtonState;

static ButtonState button_state = RESET;
int button_value = -1;
static int possible = -1;
static int button_control = -1;
static int button_control_prev = -1;
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
volatile unsigned int STATE_0 = 0 ;
volatile unsigned int count_0 = 0 ;

volatile int alive_count = 0;
volatile int alive = 0;
int count;
int frequencies[4] = {261, 293, 329, 392};

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value


// Menu control
uint8_t menu_choice = 0;
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

// void menu() {
//     // tft_fillScreen(ILI9340_BLACK);
//     tft_setTextColor(ILI9340_WHITE);
//     tft_setTextSize(1);

//     sprintf(text1, "1. Conway Game of Life - Pi");
//     tft_setCursor(10, 10);
//     tft_setTextColor2(ILI9340_WHITE, ILI9340_BLACK);
//     tft_writeString(text1);

//     sprintf(text1, "2. Conway Game of Life - Random");
//     tft_setCursor(10, 20);
//     tft_writeString(text1);

//     sprintf(text1, "3. Mandelbrot Set");
//     tft_setCursor(10, 30);
//     tft_writeString(text1);
// }
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

void button_pressing()
{
    switch (button_state)
    {

    case RESET:
        button_value = gpio_get(BUTTON_PIN);
        button_state = BUTTON_NOT_PRESSED;
    break;

    case BUTTON_NOT_PRESSED:
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
        menu_choice = (menu_choice + 1) % 3; // Cycle through menu choices
        button_control += (button_control == 3)?-3:1; //remainder 1:,2:,3:
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

void drawcell (int x, int y, int value) {
    for (int i = 0; i < size; i++){
        for (int j = 0; j < size; j++){
            tft_drawPixel(size*x+i, size*y+j, value*ILI9340_WHITE);
        }
    }
    cell_array[x][y] = value;
}
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

void draw_gosper_glider_gun(int x, int y) {
    int gun[][2] = {
        {0,4},{1,4},{0,5},{1,5},
        {10,4},{10,5},{10,6},{11,3},{11,7},{12,2},{12,8},
        {13,2},{13,8},{14,5},{15,3},{15,7},{16,4},{16,5},{16,6},
        {17,5},
        {20,2},{20,3},{20,4},{21,2},{21,3},{21,4},
        {22,1},{22,5},{24,0},{24,1},{24,5},{24,6},
        {34,2},{34,3},{35,2},{35,3}
    };
    for (int i = 0; i < sizeof(gun)/sizeof(gun[0]); i++) {
        drawcell(x + gun[i][0], y + gun[i][1], 1);
    }
}

// R-pentomino (chaotic methuselah pattern)
void draw_r_pentomino(int x, int y) {
    int pts[][2] = {
        {1,0},{2,0},{0,1},{1,1},{1,2}
    };
    for (int i = 0; i < 5; i++) {
        drawcell(x + pts[i][0], y + pts[i][1], 1);
    }
}

// Diehard (chaotic, dies after 130 generations)
void draw_diehard(int x, int y) {
    int pts[][2] = {
        {7,0},
        {1,1},{2,1},
        {2,2},{6,2},{7,2},{8,2}
    };
    for (int i = 0; i < 7; i++) {
        drawcell(x + pts[i][0], y + pts[i][1], 1);
    }
}

// Simple oscillator: Blinker (3-line)
void draw_blinker(int x, int y) {
    for (int i = 0; i < 3; i++) drawcell(x + i, y, 1);
}

// Still life: Block
void draw_block(int x, int y) {
    drawcell(x, y, 1); drawcell(x+1, y, 1);
    drawcell(x, y+1, 1); drawcell(x+1, y+1, 1);
}

// Random scatter
void draw_random_sprinkle(int width, int height, int probability) {
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            if (rand() % probability == 0) {
                drawcell(x, y, 1);
            }
        }
    }
}

void first_generation() {
    draw_gosper_glider_gun(5, 5);             // left top
    draw_r_pentomino(100, 50);                // mid
    draw_diehard(200, 60);                    // right

    draw_blinker(60, 30);
    draw_blinker(150, 80);
    draw_block(220, 30);
    draw_block(50, 100);

    draw_random_sprinkle(32, 24, 15);         //
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
        // printf("button_control: %d\n", button_control);
        pi_initial();
        random_initial();
        update_alive();
        update_cell(); 
        
        
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
        button_pressing();
        alarm_irq();
        menu();

        PT_YIELD_usec(1000) ;
    }
    PT_END(pt);
} // animation thread



void core1_main(){
  // Add animation thread
    pt_add_thread(protothread_anim);


    pt_schedule_start ;

}


int main(){
    
    stdio_init_all(); //Initialize all of the present standard stdio types that are linked into the binary
    
    // initialize button gpio
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);

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

    // start scheduler
    pt_schedule_start ;

}
