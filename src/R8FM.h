#include <Arduino.h>
#include "config.h"

#include "PPMReader.h"


PPMReader ppm(PPM_INT, PPM_CH);

unsigned A  = 0, E  = 0, T  = 0, R  = 0;
unsigned U1 = 0, U2 = 0, U3 = 0, U4 = 0;

unsigned conn  = 0;
unsigned armed = 0;
unsigned mode  = 0;

unsigned r_tar_rx = 0;
unsigned p_tar_rx = 0;
unsigned y_tar_rx = 0;

void update_ppm() {
    A  = ppm.latestValidChannelValue(1, 0);
    E  = ppm.latestValidChannelValue(2, 0);
    T  = ppm.latestValidChannelValue(3, 0);
    R  = ppm.latestValidChannelValue(4, 0);
    U1 = ppm.latestValidChannelValue(5, 0);
    U2 = ppm.latestValidChannelValue(6, 0);
    U3 = ppm.latestValidChannelValue(7, 0);
    U4 = ppm.latestValidChannelValue(8, 0);
    
    // Connection Check
    if (T == 0) {
        conn = 0;
    }
    else {
        conn = 1;
    }

    // Mapping
    if (conn == 1) {
        r_tar_rx = map(A, 1000, 2000, -20, 20);
        p_tar_rx = map(E, 1000, 2000, -20, 20);
        y_tar_rx = map(R, 1000, 2000, -20, 20);
        
        // TODO : arming
    }
    else {
        armed = 0;
    }
}
