// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "BleKeyboard.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
MPU6050 accelgyro;
int16_t ax1, ay1, az1, gx1, gy1, gz1;
int16_t ax2, ay2, az2, gx2, gy2, gz2;

// BLE keyboard
BleKeyboard bleKeyboard;

const int16_t threshold_length = 7000;
#define BUTTON_PIN 36
#define SCROLL_SENSITIVITY 10
#define GESTURE_TIME_MS 300
#define BLE_DELAY_MS 100

void setup()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // initialize serial communication
    Serial.begin(9600);
    sleep(3);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify mpu6050 connection
    Serial.println("Testing device connections...");
    if (accelgyro.testConnection())
    {
        Serial.println("MPU6050 connection successful");
    }
    else
    {
        Serial.println("MPU6050 connection failed");
        while (1)
            ;
    }

    // bluetooth keyboard
    Serial.println("Starting BLE work!");
    bleKeyboard.begin();

    // configure button pin for input
    pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop()
{
    if (bleKeyboard.isConnected())
    {
        // read button state
        int buttonState = digitalRead(BUTTON_PIN);
        if (buttonState == HIGH)
        {
            // read raw accel/gyro measurements from device
            accelgyro.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
            delay(GESTURE_TIME_MS);
            accelgyro.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);

            // send arrow direction
            if (ax2 - ax1 >= threshold_length)
            {
                Serial.println("Left");
                bleKeyboard.write(KEY_LEFT_ARROW);
                delay(BLE_DELAY_MS);
            }
            else if (ax2 - ax1 <= -threshold_length)
            {
                Serial.println("Right");
                bleKeyboard.write(KEY_RIGHT_ARROW);
                delay(BLE_DELAY_MS);
            }
            else if (az2 - az1 >= threshold_length)
            {
                Serial.println("Up");
                for (int i = 0; i < SCROLL_SENSITIVITY; i++)
                {
                    bleKeyboard.write(KEY_UP_ARROW);
                }
                delay(BLE_DELAY_MS);
            }
            else if (az2 - az1 <= -threshold_length)
            {
                Serial.println("Down");
                for (int i = 0; i < SCROLL_SENSITIVITY; i++)
                {
                    bleKeyboard.write(KEY_DOWN_ARROW);
                }
                delay(BLE_DELAY_MS);
            }
        }
    }
}