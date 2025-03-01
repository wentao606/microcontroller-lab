
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

// Include the VGA grahics library
#include "vga16_graphics.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/adc.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_3.h"
//
#include "hardware/dma.h"
#include "hardware/spi.h"

// Number of samples per period in sine table
#define sine_table_size 256

// Sine table
int raw_sin[sine_table_size] ;
int row_width_table[18];

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size] ;

// Pointer to the address of the DAC data table
unsigned short * address_pointer_dma = &DAC_data[0] ;

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

//SPI configurations
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0
#define ADC_PIN 26

// Button gpio 
#define BUTTON_PIN 3

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size ;


//DMA

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))
#define sqrtfix(a) float2fix15(sqrt(fix2float15(a)))
// Wall detection
#define hitBottom(b) (b>int2fix15(400))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))

// uS per frame
#define FRAME_RATE 33000

// the color of the boid
char color = WHITE ;

// row number
static int ball_num = 0;
static int ball_num_set = 0;
static int row_num = 16;
int data_chan;
int last_peg = -1;
uint8_t prev_ball_num = 0;

static uint32_t start_time;
static uint32_t boot_time;

static int fallen_ball_num;
static char text1[40];
static char text2[40];
static char text3[40];
static char text4[40];
int bottom_ball[17] = {0};
static int height1 = 0;
static int height2 = 0;
static int height3 = 0;
static int height4 = 0;
static int height5 = 0;
static int height6 = 0;
static int height7 = 0;
static int height8 = 0;
static int height9 = 0;
static int height10 = 0;
static int height11 = 0;
static int height12 = 0;
static int height13 = 0;
static int height14 = 0;
static int height15 = 0;
static int height16 = 0;
static int height17 = 0;


int potentio_read;
int potentio_read_prev;

// bounciness
static fix15 bounciness = float2fix15(0.5);
static fix15 min_bounciness = float2fix15(0.1);
static fix15 max_bounciness = float2fix15(0.9);

// gravity
static fix15 gravity= float2fix15(0.75);

// Boid on core 0
fix15 boid0_x ;
fix15 boid0_y ;
fix15 boid0_vx ;
fix15 boid0_vy ;

// Boid on core 1
fix15 boid1_x ;
fix15 boid1_y ;
fix15 boid1_vx ;
fix15 boid1_vy ;

// Ball 
fix15 ball_x;
fix15 ball_y;
fix15 ball_vx;
fix15 ball_vy;

// Compute x and y distances between ball and peg
fix15 dx;
fix15 dy;
fix15 distance;
fix15 normal_x;
fix15 normal_y;
fix15 intermediate_term;

// Number of pegs, Number of balls

#define peg_num 1
// #define ball_num 15
#define max_ball_num 251
#define min_ball_num 1



// Peg separations
#define vertical_seperation 19
#define horizontal_seperation 38


typedef struct {
    int x;
    int y;
} PegCoordinates;
PegCoordinates peg_coordinate[136];

typedef struct {
  fix15 x;
  fix15 y;
  fix15 vx;
  fix15 vy;
} BoidCoordinates;
BoidCoordinates ball_coordinate[max_ball_num];

typedef enum {
    RESET,
    BUTTON_NOT_PRESSED,
    BUTTON_MAYBE_PRESSED,
    BUTTON_PRESSED,
    BUTTON_MAYBE_NOT_PRESSED
} ButtonState;

static ButtonState button_state = RESET;
static int button_value = -1;
static int button_control = -1;
static int possible = -1;
// Create a boid
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction)
{
  float rand_vx = (float)rand()/RAND_MAX*0.1 ;
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(30) ;
  // Choose left or right
  if (direction) *vx =float2fix15(rand_vx) ;
  else *vx = float2fix15(-rand_vx) ;
  // Moving down
  *vy = int2fix15(1) ;
}

// Draw the boundaries
void drawArena() {
  drawVLine(100, 100, 280, BLUE) ;
  drawVLine(540, 100, 280, WHITE) ;
  drawHLine(100, 100, 440, WHITE) ;
  drawHLine(100, 380, 440, WHITE) ;
}


void ballPegCollision(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  uint8_t hit = 0;
  for (int j = 0; j < 136; j++)
  {
    dx = (fix15)((* x) - int2fix15(peg_coordinate[j].x));
    dy = (fix15)((* y) - int2fix15(peg_coordinate[j].y));

    if ((dx < int2fix15(10) && dx > int2fix15(-10)) && (dy > int2fix15(-10) && dy < int2fix15(10) )) {
    
      hit = 1;
      dma_start_channel_mask(1u << data_chan);
      distance = sqrtfix(multfix15(dx, dx)+multfix15(dy, dy));//fix15 sqrt and fix15 multiplication
      
      
      normal_x =  divfix(dx, distance);//fix15 division
      normal_y =  divfix(dy, distance);//fix15 division

      intermediate_term = multfix15(int2fix15(-2), multfix15(normal_x, *vx) + multfix15(normal_y, *vy));
      // intermediate_term = (fix15)(int2fix15(-2) * (normal_x * (* vx) + normal_y * (* vy)));

      if (intermediate_term > int2fix15(0))
      {
        *x = int2fix15(peg_coordinate[j].x) + multfix15(normal_x, distance+int2fix15(1));
        *y = int2fix15(peg_coordinate[j].y) + multfix15(normal_y, distance+int2fix15(1));
        

        *vx = *vx + multfix15(normal_x, intermediate_term);
        *vy = *vy + multfix15(normal_y, intermediate_term);

        if (last_peg != j) {
          
          *vx = multfix15(bounciness, *vx);
          *vy = multfix15(bounciness, *vy);
          last_peg = j;
        }
        
      }
      *vy = gravity +  *vy;
      // Use ball's updated velocity to update its position
      *x = *x + *vx;
      *y = *y + *vy;
      break;
    }

    // if (hit)
      // break;
  }
  if hitBottom(*y){
    fallen_ball_num ++;
    for (int i = 0; i < (row_num + 1); ++i) {
      if ((*x <= int2fix15(row_width_table[i + 1])) && (*x >= int2fix15(row_width_table[i]))) {
        bottom_ball[i] ++;
      }
    }
    

    int rand_direc = rand() % 2 ;
    float rand_vx = (float)rand()/RAND_MAX*0.1 ;

    *x = int2fix15(320) ;
    *y = int2fix15(30) ;
    if (rand_direc) *vx =float2fix15(rand_vx) ;
    else *vx = float2fix15(-rand_vx) ;

    // Moving down
    *vy = int2fix15(1) ;
    
    
  }
  // Apply gravity
  * vy = gravity +  * vy;
  // Use ball's updated velocity to update its position
  * x = * x + * vx;
  * y = * y + * vy;
}

void drawBoard(){
  int num = 0;
  for(int i = 0; i < row_num; i++){
    for (int j = 0; j < i + 1; j++){
      fillCircle(320-i*horizontal_seperation/2+j*horizontal_seperation, 100+i*vertical_seperation, 6, GREEN);
      peg_coordinate[num].x = 320 - i*horizontal_seperation/2 + j*horizontal_seperation;
      peg_coordinate[num].y = 100 + i*vertical_seperation;
      num = num + 1;
    }
  }
}

void display(uint32_t boot_time){
  sprintf(text1, "Number of fallen Balls: %d", fallen_ball_num);
  sprintf(text2, "Number of Balls: %d   ", ball_num_set);
  int boot_time_s = boot_time / 1000000;
  sprintf(text3, "Time: %d", boot_time_s);
  sprintf(text4, "Bounciness: %f", fix2float15(bounciness));

  setCursor(10, 10);
  // fillRect(10, 10, 200, 100, BLACK);
  setTextColor2(WHITE, BLACK);
  setTextSize(1);
  writeString(text1);
  setCursor(10, 20);
  // fillRect(10, 20, 200, 100, BLACK);
  writeString(text2);
  setCursor(10, 30);
  // fillRect(10, 30, 200, 100, BLACK);
  writeString(text3);
  setCursor(10, 40);
  writeString(text4);

}

// int sum = 0;
// int num_sum = 0;
// uint16_t size = 10;
// void change_ball_num(){
  
//   adc_select_input(0);
//   potentio_read = adc_read();
//   sum += potentio_read;
//   num_sum ++;

//   if (potentio_read != potentio_read_prev && num_sum == size){
//     int k = (float)sum / (float)size * 1.0;
//     ball_num_set = (int)min_ball_num + (int)((k-13)*(max_ball_num-min_ball_num)/(4095-13));
//     ball_num = ball_num_set;
//     printf("%d\n", potentio_read);
//   }

//   if (num_sum == size) {
//     sum = 0;
//     num_sum = 0;
//   }

// }
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
    button_control += (button_control == 2)?-2:1; //remainder 1:,2:,3:
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
      // button_control += 1; //remainder 1:,2:,3:
      button_state = BUTTON_PRESSED;
    }
  else{
    button_value = gpio_get(BUTTON_PIN);
    button_state = BUTTON_MAYBE_NOT_PRESSED;
  }
case BUTTON_MAYBE_NOT_PRESSED:
  if (button_value == possible){
    button_value = gpio_get(BUTTON_PIN);
    button_state = BUTTON_PRESSED;
  }
  else{
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

int ball_num_prev = 0;
void change_ball_num(){
  if (button_control == 1){
    adc_select_input(0);
    potentio_read = adc_read();
    ball_num_prev = ball_num_set;
    if (potentio_read != potentio_read_prev){
      ball_num_set = (int)min_ball_num + (int)((potentio_read-13)*(max_ball_num-min_ball_num)/(4095-13));
      ball_num_set = ball_num_prev + (int)((ball_num_set - ball_num_prev)>>4);
      ball_num = ball_num_set;
    }
  }
}

int ball_bounciness_prev = 0;
void change_ball_bounciness(){
  if (button_control == 2){
    adc_select_input(0);
    potentio_read = int2fix15(adc_read());
    ball_bounciness_prev = bounciness;
    if (potentio_read != potentio_read_prev){
      potentio_read_prev = potentio_read;
      bounciness = min_bounciness + divfix(multfix15((potentio_read-int2fix15(13)), (max_bounciness-min_bounciness)),int2fix15(4095-13));
      // ball_num_set = ball_num_prev + (int)((ball_num_set - ball_num_prev)>>4);
      // ball_num = ball_num_set;
      printf("bounciness:%f", fix2float15(bounciness));
    }
  }
}


// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input ;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write ;
      while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input a number in the range 1-15: ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input) ;
        // update boid color
        if ((user_input > 0) && (user_input < 16)) {
          color = (char)user_input ;
        }
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;

    // prev_ball_num = ball_num_set;
    // change_ball_num();
    // Spawn a boid
    for (int i = 0; i < ball_num_set; i++){
      int rand_direc = rand() % 2 ;
      spawnBoid(&ball_coordinate[i].x, &ball_coordinate[i].y, 
        &ball_coordinate[i].vx, &ball_coordinate[i].vy, rand_direc);
    }
    
    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;      
      //
      drawBoard();
      //
      prev_ball_num = ball_num_set;
      change_ball_num();
      change_ball_bounciness();
      if (prev_ball_num < ball_num_set) {
        for (int i = prev_ball_num; i < ball_num_set; ++i) {
          int rand_direc = rand() % 2 ;
          spawnBoid(&ball_coordinate[i].x, &ball_coordinate[i].y, &ball_coordinate[i].vx,
            &ball_coordinate[i].vy, rand_direc);
        }
      }
      else if ( prev_ball_num > ball_num_set) {
        for (int i = ball_num_set; i < prev_ball_num; ++i) {
          fillCircle(fix2int15(ball_coordinate[i].x), fix2int15(ball_coordinate[i].y), 4, BLACK);
        }
      }
      // erase boid
      for (int i = 0; i < max_ball_num; i++){
        if (i < ball_num) {
          fillCircle(fix2int15(ball_coordinate[i].x), fix2int15(ball_coordinate[i].y), 4, BLACK);
          ballPegCollision(&ball_coordinate[i].x, &ball_coordinate[i].y, &ball_coordinate[i].vx, &ball_coordinate[i].vy);
          fillCircle(fix2int15(ball_coordinate[i].x), fix2int15(ball_coordinate[i].y), 4, color);
        }
        // else {
        //   fillCircle(fix2int15(ball_coordinate[i].x), fix2int15(ball_coordinate[i].y), 4, BLACK);
        // }
        
      }
      display(time_us_32() - start_time);
      
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

int prev_height[17];
// Animation on core 1
static PT_THREAD (protothread_anim1(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ; 
      for (int i = 0; i < (row_num + 1); ++i) {
        // (1-fix2int15(divfix(int2fix15(bottom_ball[i]), int2fix15(fallen_ball_num))))*70
        //  (short)480 - (short)multfix15(divfix(int2fix15(bottom_ball[i]), int2fix15(fallen_ball_num)), 50),
        
        int height = (int)((float)(bottom_ball[i]) / fallen_ball_num * 250);
        height = height > 50? 50:height;
        int diff = 0;
        if (prev_height[i] > height) {
          diff = prev_height[i] - height;
          fillRect(row_width_table[i], 
            (short)480 - prev_height[i],
            38, diff, BLACK);
        }
        fillRect(row_width_table[i], 
          (short)480 - height,
          36, 50, YELLOW);
        prev_height[i] = height;
      }
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread
static PT_THREAD (protothread_anim2(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);
    button_pressing();
    PT_YIELD_usec(30000);
  PT_END(pt);
}
// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){
  // Add animation thread
  pt_add_thread(protothread_anim1);
  // Start the scheduler
  pt_schedule_start ;

}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  // initialize stio
  stdio_init_all() ;
  start_time = time_us_32();
  // initiate adc hardware and 
  adc_init();
  adc_gpio_init(ADC_PIN);

  // initialize button gpio
  gpio_init(BUTTON_PIN);
  gpio_set_dir(BUTTON_PIN, GPIO_IN);


  // Initialize SPI channel (channel, baud rate set to 20MHz)
  spi_init(SPI_PORT, 20000000) ;

  // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
  spi_set_format(SPI_PORT, 16, 0, 0, 0);

  // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

  // Build sine table and DAC data table
  int i ;
  for (i=0; i<(sine_table_size); i++){
      raw_sin[i] = (int)(2047 * sin((float)i*6.283/(float)sine_table_size) + 2047); //12 bit
      DAC_data[i] = DAC_config_chan_A | (raw_sin[i] & 0x0fff) ;
  }

  // Select DMA channels
  data_chan = dma_claim_unused_channel(true);
  int ctrl_chan = dma_claim_unused_channel(true);

   // Setup the control channel
   dma_channel_config c = dma_channel_get_default_config(ctrl_chan);   // default configs
   channel_config_set_transfer_data_size(&c, DMA_SIZE_32);             // 32-bit txfers
   channel_config_set_read_increment(&c, false);                       // no read incrementing
   channel_config_set_write_increment(&c, false);                      // no write incrementing
   //channel_config_set_chain_to(&c, data_chan);                         // chain to data channel

  dma_channel_configure(
       ctrl_chan,                          // Channel to be configured
       &c,                                 // The configuration we just created
       &dma_hw->ch[data_chan].read_addr,   // Write address (data channel read address)
       &address_pointer_dma,                   // Read address (POINTER TO AN ADDRESS)
       1,                                  // Number of transfers
       false                               // Don't start immediately
  );

  // Setup the data channel
  dma_channel_config c2 = dma_channel_get_default_config(data_chan);  // Default configs
  channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);            // 16-bit txfers
  channel_config_set_read_increment(&c2, true);                       // yes read incrementing
  channel_config_set_write_increment(&c2, false);                     // no write incrementing
  // (X/Y)*sys_clk, where X is the first 16 bytes and Y is the second
  // sys_clk is 125 MHz unless changed in code. Configured to ~44 kHz
  dma_timer_set_fraction(0, 0x0017, 0xffff) ;
  // 0x3b means timer0 (see SDK manual)
  channel_config_set_dreq(&c2, 0x3b);                                 // DREQ paced by timer 0
  channel_config_set_chain_to(&c2, ctrl_chan);                        // Chain to control channel

  dma_channel_configure(
      data_chan,                  // Channel to be configured
      &c2,                        // The configuration we just created
      &spi_get_hw(SPI_PORT)->dr,  // write address (SPI data register)
      DAC_data,                   // The initial read address
      sine_table_size,            // Number of transfers
      false                       // Don't start immediately.
  );

  dma_start_channel_mask(1u << ctrl_chan) ;
  for (uint16_t i = 0; i < row_num + 2; ++i) {
    row_width_table[i] = i * 38;
  }
  // initialize VGA
  initVGA() ;
  
  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);
  pt_add_thread(protothread_anim2);

  // start scheduler
  pt_schedule_start ;
  
} 