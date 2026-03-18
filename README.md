# ELE313RoboticsLab
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "leds.h"
#include "spi_comm.h"
#include "sensors/proximity.h"
#include "motors.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static inline int imax(int a, int b){ return (a>b)?a:b; }

int main(void) {

halInit();
chSysInit();
mpu_init();

// Proximity
messagebus_init(&bus, &bus_lock, &bus_condvar);
proximity_start(0);
calibrate_ir();

//LED
clear_leds();
spi_comm_start();

//Motors
motors_init();

// /* Infinite loop. */ // const int TRIGGER=220; // const int RELEASE=500; // bool blocked=false;

// --- thresholds  ---
   const int FRONT_TH = 450;   // head-on detection threshold
   const int SIDE_TH  = 500;   // side obstacle detection threshold
   const int DELTA    = 50;    // small margin to choose left vs right

   while (1)
   {
       // FRONT sensors
       int rf = get_calibrated_prox(0); // front-right
       int lf = get_calibrated_prox(7); // front-left

       // SIDE sensors
       int rside = imax(get_calibrated_prox(1), get_calibrated_prox(2)); // right flank
       int lside = imax(get_calibrated_prox(5), get_calibrated_prox(6)); // left flank

       int front = imax(rf, lf); // overall front signal


       // 1) Head-on obstacle
       if (front > FRONT_TH) {

    	   if ((lf > rf - DELTA) && (lf < rf + DELTA)) {

    	           // Reverse a bit to create turning space
    	           left_motor_set_speed(-700);
    	           right_motor_set_speed(-700);
    	           chThdSleepMilliseconds(120);

    	           // Then turn (choose one direction → here: turn left)
    	           left_motor_set_speed(-675);
    	           right_motor_set_speed(675);
    	           chThdSleepMilliseconds(150);

    	           continue;}

           if (lf > rf + DELTA) {
               // more obstacle on left-front → pivot RIGHT
               left_motor_set_speed(675);
               right_motor_set_speed(-675);
           } else {
               // more obstacle on right-front → pivot LEFT
               left_motor_set_speed(-675);
               right_motor_set_speed(675);
           }
       }
       // 2) Side obstacles
       else if (rside > SIDE_TH && lside <= SIDE_TH) {
           // obstacle on right → turn LEFT
           left_motor_set_speed(-675);
           right_motor_set_speed(675);
       }
       else if (lside > SIDE_TH && rside <= SIDE_TH) {
           // obstacle on left, turn RIGHT
           left_motor_set_speed(675);
           right_motor_set_speed(-675);
       }
       // 3) Clear path, go straight
       else {
           left_motor_set_speed(1000);
           right_motor_set_speed(1000);
       }
       set_rgb_led (LED8, 10, 0, 10);
       set_rgb_led (LED2, 10, 0, 10);
       set_rgb_led (LED4, 10, 0, 10);
       set_rgb_led (LED6, 10, 0, 10);
       set_front_led (1);
       set_body_led(2);
       chThdSleepMilliseconds(50); // loop delay
   }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}
