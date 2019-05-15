#include <Arduino.h>
#include <Wire.h>
#include "svm30.h"

    uint16_t i = 0;
    int16_t err;
    uint16_t tvoc_ppb, co2_eq_ppm;
    uint32_t iaq_baseline;
    int32_t temperature, humidity;

void sensirion_i2c_init() {
    Wire.begin();
}

int8_t sensirion_i2c_read(uint8_t address, uint8_t *data, uint16_t count) {
    uint8_t readData[count];
    uint8_t rxByteCount = 0;

    // 2 bytes RH, 1 CRC, 2 bytes T, 1 CRC
    Wire.requestFrom(address, count);

    while (Wire.available()) {  // wait till all arrive
        readData[rxByteCount++] = Wire.read();
        if (rxByteCount >= count)
            break;
    }

    memcpy(data, readData, count);

    return 0;
}

int8_t sensirion_i2c_write(uint8_t address, const uint8_t *data,
                           uint16_t count) {
    Wire.beginTransmission(address);
    Wire.write(data, count);
    Wire.endTransmission();

    return 0;
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_sleep_usec(uint32_t useconds) {
    delay((useconds / 1000) + 1);
}

void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
}

void loop() {

       while (svm_probe() != STATUS_OK) {
       Serial.println("SVM30 module probing failed");
    }
    Serial.println("SVM30 module probing successful"); 

    /* Consider the two cases (A) and (B):
     * (A) If no baseline is available or the most recent baseline is more than
     *     one week old, it must discarded. A new baseline is found with
     *     sgp_iaq_init() */
    err = sgp30_iaq_init();
    /* (B) If a recent baseline is available, set it after sgp_iaq_init() for
     * faster start-up */
    /* IMPLEMENT: retrieve iaq_baseline from presistent storage;
     * err = sgp_set_iaq_baseline(iaq_baseline);
     */

    /* Run periodic IAQ measurements at defined intervals */
    while (1) {
        err = svm_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm,
                                            &temperature, &humidity);
        if (err == STATUS_OK) {
            Serial.println("tVOC  Concentration:" + (String)tvoc_ppb);
            Serial.println("CO2eq Concentration:" + (String)co2_eq_ppm);
            double temp = temperature / 1000.0f;
            double humi = humidity / 1000.0f;
            Serial.println("Temperature:" + (String)temp);
            Serial.println("Humidity:" + (String)humi);
        } else {
            //Serial.println("error reading sensor\n");
        }

        delay(2000);
        /* Persist the current baseline every hour */
        if (++i % 3600 == 3599) {
            err = sgp30_get_iaq_baseline(&iaq_baseline);
            if (err == STATUS_OK) {
                /* IMPLEMENT: store baseline to presistent storage */
            }
        }
    }
}
