/*
 * Copyright (C) 2015  TELECOM Nancy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <SPI.h>

#include "Mpu9250Spi.hpp"


/**
 * @brief Prefix serial data
 */
#define SERIAL_HEADER 0xA5

/**
 * @brief MPU9250 SPI slave select pin connection
 */
int _slaveSelectMpu = 10;
/**
 * @brief Altimeter SPI slave select pin connection
 */
int _slaveSelectAlt = 9;
/**
 * @brief MPU9250 interrupt pin connection
 */
int _mpuInt = 8;

/**
 * @brief MPU9250 with SPI support
 */
mpu9250::Mpu9250Spi mpu(_slaveSelectMpu, _mpuInt);

namespace sketch {

    /**
    * @brief MPU9250 data as either uint8_t array or 3d vectors
    *
    *
    */
    union {
        /**
        * @brief MPU9250 data in 3D Vectors
        *
        *
        */
        struct {
            struct mpu9250::Int16Vect3D accel; /**< 3D accelerometer vector (arbitrary unit) */
            struct mpu9250::Int16Vect3D gyro; /**< 3D gyroscope vector (arbitrary unit) */
            struct mpu9250::Int16Vect3D mag; /**< 3D magnetometer vector (arbitrary unit) */
            int16_t temp; /**< MPU9250 temperature (arbitrary unit) */
        };
        uint8_t value[20]; /**< Structure as array of byte */
    } data;
}


void setup() {
    // Altimeter no used
    pinMode(_slaveSelectAlt, OUTPUT);
    digitalWrite(_slaveSelectAlt, HIGH);

    // Serial at 1Mbauds
    Serial.begin(1000000);
    // Init MPU2950
    mpu.init();
    // Set accelerometer low pass filter to 41Hz
    mpu.setAccelDLPF(true, mpu9250::DLPF_ACCEL_41HZ);
    // Set accelerometer full scale to +- 4G to avoid overflow
    mpu.setAccelFullScale(mpu9250::ACCEL_FULL_SCALE_4G);
    // Set gyroscope low pass filter to 41Hz
    mpu.setGyroDLPF(mpu9250::FCHOICE_DLPF_ENABLE, mpu9250::DLPF_GYRO_41HZ);
    // Set gyroscope full scale to +- 250 degrees/s
    mpu.setGyroFullScale(mpu9250::GYRO_FULL_SCALE_250DPS);
    // We want a sample at 100 Hz (1kHz / (1 + 9))
    mpu.setSampleRateDivider(0x09);
    // Enable interrupt only for data ready
    mpu.setInterrupt(false, false,true, true);
    // Interrupt pin goes LOW only when a read occurs
    mpu.setInterruptPin(false,false, false, true);
    // Send config to MPU9250
    mpu.uploadConfig();
}


void loop() {
    // Check if data ready
    if(digitalRead(_mpuInt) == HIGH) {
        // Get MPU9250 data
        mpu.getSensors(&sketch::data.accel, &sketch::data.gyro, &sketch::data.mag, &sketch::data.temp) ;
        // Write it to serial
        writeSerial();
    }
}

/**
 * @brief write the data structure to the serial port
 *
 *
 */
void writeSerial() {

    uint8_t sum = 0; // checksum initialisation

    // Compute checksum
    // XOR between each byte
    for(uint8_t i = 0; i< 20; i++) {
        sum ^= sketch::data.value[i];
    }

    // Send serial prefix
    Serial.write(SERIAL_HEADER);
    // Send data
    Serial.write(sketch::data.value, 20);
    // Sen checksum
    Serial.write(sum);

}
