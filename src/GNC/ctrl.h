#ifndef CTRL_H
#define CTRL_H

#include <Arduino.h>
#include <Servo.h>

// Global variables
extern float Kp;
extern float Ki;
extern float Kd;

extern unsigned long dt_now, dt_prev;
extern double dt;

extern Servo mtr1, mtr2, mtr3, mtr4;
extern int mtr1_speed, mtr2_speed, mtr3_speed, mtr4_speed;

// Function declarations
void GNC_init_ctrl();
void GNC_loop_ctrl();
int T2180(int T);

#endif
