// Example usage for Adafruit_FRAM_I2C library by KTOWN (Adafruit Industries).

#include "Adafruit_FRAM_I2C.h"

// Initialize objects from the lib
Adafruit_FRAM_I2C adafruit_FRAM_I2C;

void setup() {
    // Call functions on initialized library objects that require hardware
    adafruit_FRAM_I2C.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    adafruit_FRAM_I2C.process();
}
