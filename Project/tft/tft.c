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
// Include protothreads

#include "pico/multicore.h"
#include "pt_cornell_rp2040_v1_3.h"
#define size 1
#define FRAME_RATE 33000
#define BUTTON_PIN 14

static unsigned char cell_array[240/size][320/size] ;
static char cell_array_next[240/size][320/size] ;
uint8_t start_init = 1; // flag to indicate if the initial state is set

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
static int button_control = 0;
static int button_control_prev = 0;

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

void is_alive(int x, int y) {
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
    
    return;
}


void update_alive(){
    for (int i = 0; i< (240/size); i++){
        for (int j = 0; j < (320/size); j++){
            is_alive (i,j);
        }
    }
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
        
        if (button_control != button_control_prev) {
            button_control_prev = button_control;
            tft_fillScreen(ILI9340_BLACK);
            memset(cell_array, 0, sizeof(cell_array));
            memset(cell_array_next, 0, sizeof(cell_array_next));
            start_init = 1;
        }
        printf("button_control: %d\n", button_control);
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
        // button_value = gpio_get(BUTTON_PIN)

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

      // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(&core1_main);

    // add threads
    pt_add_thread(protothread_btn);
    // pt_add_thread(protothread_update_alive);

    // start scheduler
    pt_schedule_start ;
    // first_generation();
    // while(true){
    //     update_alive();
    //     // sleep_ms(500);
    //     update_cell();
    //     // sleep_ms(500);
    // }

//     while(1){ //Infinite while loop
//         unsigned long begin_time = (unsigned long)(get_absolute_time() / 1000); //Get the start time
//         switch(count){ //Based on the current count, switch different colours
//             case 0: col = ILI9340_BLUE;
//                     break;
//             case 1: col = ILI9340_RED;
//                     break;
//             case 2: col = ILI9340_GREEN;
//                     break;
//             case 3: col = ILI9340_CYAN;
//                     break;
//             case 4: col = ILI9340_MAGENTA;
//                     break;
//             case 5: col = ILI9340_YELLOW;
//                     break;
//             case 6: col = ILI9340_WHITE;
//                     break;
//         }
//         for(i = 0; i < ILI9340_TFTWIDTH / 4; i++){
//             for(j = 0; j < ILI9340_TFTHEIGHT / 4; j++){
//                 //tft_drawRect(i << 2, j << 2, 4, 4, col); //Simply drawing a rectangle takes 222 ms
//                 tft_fillRect(i << 2, j << 2, 4, 4, col); //Filling the entire rectangle surprisingly takes 110 ms
//             }
//         }
//         count = (count + 1) % 7; //Increment the count and keep it between 0-6
//         unsigned char exTime = ((unsigned long)(get_absolute_time() / 1000) - begin_time); //Calculate the amount of time taken
//         printf("%u\n", count); //Print the time out
//     }
}
