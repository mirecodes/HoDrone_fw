#include <Arduino.h>
#include "GNC/ctrl.h"
#include "config.h"

#include <Servo.h>

float Kp = 1.0;
float Ki = 0.0;
float Kd = 1.0;

unsigned long dt_now = 0, dt_prev = 0;

double dt = 0;

Servo mtr1, mtr2, mtr3, mtr4;

int mtr1_speed = 0;
int mtr2_speed = 0;
int mtr3_speed = 0;
int mtr4_speed = 0;

int T2180(int T){
    return (int)(((float)T-1000.0)/1000.0 * 180.0);
}

void GNC_init_ctrl() {
    mtr1.attach(MTR_1, 1000, 2000);
    mtr2.attach(MTR_2, 1000, 2000);
    mtr3.attach(MTR_3, 1000, 2000);
    mtr4.attach(MTR_4, 1000, 2000);
    
    mtr1.write(0);
    mtr2.write(0);
    mtr3.write(0);
    mtr4.write(0);
}

void GNC_loop_ctrl() {

}

    // // Controller start
    // r_err = r_tar - r_cur;
    // p_err = p_tar - p_cur;
    // if(mode == 1){ y_err = 0; }
    // else{ y_err = y_tar - y_cur; }

    // dt_now = micros();
    // dt = (dt_now - dt_prev) / 1000000.0;
    // dt_prev = dt_now;

    // Balr  = Kp * r_err;       Balp  = Kp * p_err;       Baly  = Kp * y_err;
    // Balr += Kd * -r_rate;     Balp += Kd * -p_rate;     Baly += Kd * -y_rate;
    // Balr += Ki * r_err * dt;  Balp += Ki * p_err * dt;  Baly += Ki * y_err * dt;
    
    // if (armed){
    //     mtr1_speed = constrain(T2180(T) + Balp + Balr + Baly, 0, 180);
    //     mtr2_speed = constrain(T2180(T) + Balp - Balr - Baly, 0, 180);
    //     mtr3_speed = constrain(T2180(T) - Balp + Balr - Baly, 0, 180);
    //     mtr4_speed = constrain(T2180(T) - Balp - Balr + Baly, 0, 180);
    // }
    // else{
    //     mtr1_speed = 0;  mtr2_speed = 0;  mtr3_speed = 0;  mtr4_speed = 0;
    // }

    // // Motor update
    // mtr1.write(mtr1_speed);  mtr2.write(mtr2_speed);  mtr3.write(mtr3_speed);  mtr4.write(mtr4_speed);
