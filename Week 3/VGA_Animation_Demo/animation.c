
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
// Include protothreads
#include "pt_cornell_rp2040_v1_3.h"

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

// Wall detection
#define hitBottom(b) (b>int2fix15(380))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))

// uS per frame
#define FRAME_RATE 33000

// the color of the boid
char color = WHITE ;

// row number
static int row_num = 6;

// bounciness
static fix15 bounciness = 0.5;

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

// Number of pegs, Number of balls

#define peg_num 1
#define ball_num 1

// Peg separations
#define vertical_seperation 19
#define horizontal_seperation 38


typedef struct {
    int x;
    int y;
} Coordinates;
Coordinates peg_coordinate[16];
Coordinates ball_coordinate[10];
// Create a boid
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction)
{
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(30) ;
  // Choose left or right
  if (direction) *vx = int2fix15(3) ;
  else *vx = int2fix15(-3) ;
  // Moving down
  *vy = int2fix15(0) ;
}

// Draw the boundaries
void drawArena() {
  drawVLine(100, 100, 280, BLUE) ;
  drawVLine(540, 100, 280, WHITE) ;
  drawHLine(100, 100, 440, WHITE) ;
  drawHLine(100, 380, 440, WHITE) ;
}

// Detect wallstrikes, update velocity and position
void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // Reverse direction if we've hit a wall
  if (hitTop(*y)) {
    *vy = (-*vy) ;
    *y  = (*y + int2fix15(5)) ;
  }
  if (hitBottom(*y)) {
    *vy = (-*vy) ;
    *y  = (*y - int2fix15(5)) ;
  } 
  if (hitRight(*x)) {
    *vx = (-*vx) ;
    *x  = (*x - int2fix15(5)) ;
  }
  if (hitLeft(*x)) {
    *vx = (-*vx) ;
    *x  = (*x + int2fix15(5)) ;
  } 
  
  // Update position using velocity
  *x = *x + *vx ;
  *y = *y + *vy ;
}

void ballPegCollision(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  
  // Compute x and y distances between ball and peg
  fix15 dx;
  fix15 dy;
  fix15 distance;
  fix15 normal_x;
  fix15 normal_y;
  fix15 intermediate_term;

  for (int j = 0; j < 16; j++)
  {
    
    dx = (fix15)((* x) - int2fix15(peg_coordinate[j].x));
    dy = (fix15)((* y) - int2fix15(peg_coordinate[j].y));

    if (dx < 10 || dy < 10){
      distance = (fix15)(sqrt(dx * dx + dy * dy));
      
      normal_x =  (fix15)(dx / distance);
      normal_y =  (fix15)(dy / distance);

      intermediate_term = (fix15)(int2fix15(-2) * (normal_x * (* vx) + normal_y * (* vy)));

      if (intermediate_term > 0)
      {
        (* x)= (fix15)(peg_coordinate[j].x + (normal_x * (distance+1)));
        (* y) = (fix15)(peg_coordinate[j].y + (normal_y * (distance+1)));

        * vx = (fix15)(* vx + (normal_x * intermediate_term));
        * vy = (fix15)(* vy + (normal_y * intermediate_term));
        
        * vx = (fix15)(bounciness * (* vx));
        * vy = (fix15)(bounciness * (* vy));
        
      }
    }

    
  }
  fillCircle(100, 100, 5, RED);
  (* vy) = (fix15)(* vy + float2fix15(0.75));
  (* x) = (fix15)(* x + * vx);
  (* y) = (fix15)((* y) + * vy);
  printf("vy:%d, x:%d, y:%d", *vy, *x, *y);

}



void drawBoard(){
  
  for(int i = 0; i < row_num; i++){
    for (int j = 0; j < i + 1; j++){
      fillCircle(320-i*horizontal_seperation/2+j*horizontal_seperation, 100+i*vertical_seperation, 6, WHITE);
      peg_coordinate[i].x = 320 - i*horizontal_seperation/2 + j*horizontal_seperation;
      peg_coordinate[i].y = 100 + i*vertical_seperation;
    }
  }
}

// void drawBoard(){
//   for (int i = 0; i < peg_num; i++)
//   {
//     fillCircle( 320, 30, 6, WHITE);
//   }
//   for (int i = 0; i < ball_num; i++)
//   {
//     fillCircle( 320, 10, 4, GREEN);
//   }
// }

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

    // Spawn a boid
    spawnBoid(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy, 0);

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;      
      //
      drawBoard();
      //
      // erase boid
      fillCircle(fix2int15(boid0_x), fix2int15(boid0_y), 4, BLACK);
      // update boid's position and velocity
      ballPegCollision(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy);
      // draw the boid at its new position
      fillCircle(fix2int15(boid0_x), fix2int15(boid0_y), 4, color); 
      
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread


// Animation on core 1
static PT_THREAD (protothread_anim1(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;

    // Spawn a boid
    spawnBoid(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy, 1);

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ; 
      
      // erase boid
      drawRect(fix2int15(boid1_x), fix2int15(boid1_y), 2, 2, BLACK);
      // update boid's position and velocity
      wallsAndEdges(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy) ;
      // draw the boid at its new position
      drawRect(fix2int15(boid1_x), fix2int15(boid1_y), 2, 2, color); 

      //

      //


      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

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

  // initialize VGA
  initVGA() ;

  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 
