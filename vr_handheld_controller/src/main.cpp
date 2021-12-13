/*
Ryan Dalby- CS 6360 Virtual Reality Final Project 

.ino for vr_handheld_controller

This code is the code behind the handheld VR controller that broadcasts an IMU service with accelerometer, gyroscope, and magnetometer data.
*/

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

#define BUTTON_PIN 3

#define ARDUINO_FLOAT_LENGTH_BYTES 4
#define ACC_ELEMENTS 3
#define GYR_ELEMENTS 3
#define MAG_ELEMENTS 3
#define BLE_LOCAL_NAME_IMU "VRHANDHELDCONTROLLER"
#define BLE_UUID_IMU_SERVICE "12307e98-a67e-45e2-b8f0-1445fa6e2cd8"
#define BLE_UUID_ACC "cf89ef50-4bdb-423a-a822-88aaf7f96c5a"
#define BLE_UUID_GYR "1552ab31-afbb-418c-bd23-1f4527751390"
#define BLE_UUID_MAG "55584dd1-b773-44aa-a38d-ed6d9c94a851"
#define BLE_UUID_BUTTON "017fa2e8-b256-47bf-953b-a494b75fb9bf"

typedef unsigned char uint8_t;

float acc[ACC_ELEMENTS] = {0.0,0.0,0.0}; // x,y,z g = 9.80665 m/s^2      
uint8_t *acc_buffer = (uint8_t *) acc;
float gyr[GYR_ELEMENTS] = {0.0,0.0,0.0}; // x,y,z deg/s
uint8_t *gyr_buffer = (uint8_t *) gyr;
float mag[MAG_ELEMENTS] = {0.0,0.0,0.0};  // x,y,z muT (micro-Teslas)
uint8_t *mag_buffer = (uint8_t *) mag;
bool button_pressed = false;

BLEService IMUService(BLE_UUID_IMU_SERVICE); 
BLECharacteristic IMUCharacteristicAcc(BLE_UUID_ACC, BLENotify, ARDUINO_FLOAT_LENGTH_BYTES*3, true); 
BLECharacteristic IMUCharacteristicGyr(BLE_UUID_GYR, BLENotify, ARDUINO_FLOAT_LENGTH_BYTES*3, true); 
BLECharacteristic IMUCharacteristicMag(BLE_UUID_MAG, BLENotify, ARDUINO_FLOAT_LENGTH_BYTES*3, true); 
BLEBoolCharacteristic ButtonPressedCharacteristic(BLE_UUID_BUTTON, BLENotify);

void setup() {
    delay(5000); // Delay so setup Serial output can be observed
    Serial.begin(9600);

    // Enable button pin for input
    pinMode(BUTTON_PIN, INPUT);

    // BLE
    if (!BLE.begin()){
        Serial.println("Starting BLE module failed!");
        while (1);
    }
    Serial.println("***Started BLE module***");

    BLE.setLocalName(BLE_LOCAL_NAME_IMU);
    BLE.setAdvertisedService(IMUService);
    Serial.println("Set advertised local name and service UUID");

    IMUService.addCharacteristic(IMUCharacteristicAcc);
    IMUService.addCharacteristic(IMUCharacteristicGyr);
    IMUService.addCharacteristic(IMUCharacteristicMag);
    IMUService.addCharacteristic(ButtonPressedCharacteristic);
    Serial.println("Added characteristics to service");

    BLE.addService(IMUService);
    Serial.println("Added service");

    IMUCharacteristicAcc.writeValue(acc_buffer, ARDUINO_FLOAT_LENGTH_BYTES*3);
    IMUCharacteristicGyr.writeValue(gyr_buffer, ARDUINO_FLOAT_LENGTH_BYTES*3);
    IMUCharacteristicMag.writeValue(mag_buffer, ARDUINO_FLOAT_LENGTH_BYTES*3);
    ButtonPressedCharacteristic.writeValue(button_pressed);
    Serial.println("Wrote initial characteristic values");

    BLE.advertise();  
    Serial.println("Started BLE advertising");
    Serial.println("***BLE initialization complete***");
    Serial.println();
    Serial.println();


    // IMU
    if (!IMU.begin()){
        Serial.println("Starting IMU module failed!");
        while (1);
    }
    Serial.println("***IMU module started***");
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println("Hz");

    Serial.print("Gyroscope sample rate = ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println("Hz");

    Serial.print("Magnetometer sample rate = ");
    Serial.print(IMU.magneticFieldSampleRate());
    Serial.println("Hz");
    Serial.println("***IMU initialization complete***");
    Serial.println();
    Serial.println();
}

void loop() {
    BLEDevice central = BLE.central();
    Serial.println("Discovering central device");

    if(central){
        Serial.println("Connected to central device");
        Serial.print("MAC address: ");
        Serial.println(central.address());

        while (central.connected()) {
            if (IMU.accelerationAvailable()) {
                // Acc range [-4,+4]g -/+0.122 mg
                IMU.readAcceleration(acc[0], acc[1], acc[2]); // g = 9.80665 m/s^2
                IMUCharacteristicAcc.writeValue(acc_buffer, ARDUINO_FLOAT_LENGTH_BYTES*ACC_ELEMENTS);
            }
            if (IMU.gyroscopeAvailable()) {
                // Gyr range [-2000, +2000] dps +/-70 mdps
                IMU.readGyroscope(gyr[0], gyr[1], gyr[2]); // deg/s
                IMUCharacteristicGyr.writeValue(gyr_buffer, ARDUINO_FLOAT_LENGTH_BYTES*ACC_ELEMENTS);
            }
            if (IMU.magneticFieldAvailable()) {
                // Mag range  [-400, +400] uT +/-0.014 uT
                IMU.readMagneticField(mag[0], mag[1], mag[2]); // muT (micro-Teslas)
                IMUCharacteristicMag.writeValue(mag_buffer, ARDUINO_FLOAT_LENGTH_BYTES*ACC_ELEMENTS);
            }
            // Send button state
            button_pressed = (digitalRead(BUTTON_PIN) == HIGH); 
            ButtonPressedCharacteristic.writeValue(button_pressed);
        }

        Serial.println("Disconnected to central device");

    }

}