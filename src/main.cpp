
#include <Arduino.h>
#include "config.h"

#include "R8FM.h"

#include "GNC/nav.h"
#include "GNC/ctrl.h"

unsigned long prev_ms = 0;





void setup() {
    Serial.begin(115200);

    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);
    pinMode(LED_4, OUTPUT);
    pinMode(LED_5, OUTPUT);
    pinMode(BUZ,   OUTPUT);
    pinMode(MNT,   INPUT);

    GNC_init_nav();
}

void loop() {
    update_ppm();    // ppm update

    GNC_loop_nav();
    
    uint32_t curr_ms = millis();

    if (curr_ms - prev_ms >= 10) {
        prev_ms = curr_ms;
        
        Serial.print(r_cur);
        Serial.print("\t");
        Serial.print(p_cur);
        Serial.print("\t");
        Serial.print(y_cur);
        Serial.print("\t");
        // Serial.println();
        
        Serial.print(sats);
        Serial.print("\t");
        Serial.print(lat, 6);
        Serial.print("\t");
        Serial.print(lng, 6);
        Serial.print("\t");
        Serial.print(alt_GPS);
        Serial.print("\t");
        Serial.println();
    }
}
