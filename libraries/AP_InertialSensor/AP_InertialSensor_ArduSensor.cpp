#include "AP_InertialSensor_ArduSensor.h"

extern const AP_HAL::HAL& hal;


ArduSensor::ArduSensor(uint8_t address) : _dev(address), _address(address), ax(0), ay(0), az(0), gx(0), gy(0), gz(0) {}


bool ArduSensor::init() {
    // Begin I2C communication
    if (!hal.i2c->begin()) {
        hal.console->println("I2C initialization failed.");
        return false;
    }

    // Check sensor ID to ensure proper connection
    uint8_t chip_id;
    if (!read_register(0x00, chip_id) || chip_id != 0x24) {  // 0x24 is the expected chip ID for BMI270
        hal.console->println("BMI270 not found.");
        return false;
    }

    // Configure the sensor settings (simple configuration for accelerometer and gyro)
    hal.i2c->write_register(_address, 0x7C, 0x04);  // Power up accelerometer
    hal.i2c->write_register(_address, 0x7D, 0x04);  // Power up gyroscope
    hal.i2c->write_register(_address, 0x40, 0xA8);  // Set accelerometer config
    hal.i2c->write_register(_address, 0x42, 0xA9);  // Set gyroscope config

    hal.console->println("BMI270 initialized successfully.");
    return true;
}

// Read accelerometer and gyroscope data from the sensor
bool ArduSensor::read() {
    uint8_t buffer[12];  // Buffer to hold accelerometer and gyroscope data
    if (!read_registers(0x0C, buffer, 12)) {  // Reading from the accelerometer and gyroscope data registers
        hal.console->println("Failed to read BMI270 data.");
        return false;
    }

    // Convert raw data to signed integers
    ax = (int16_t)((buffer[1] << 8) | buffer[0]) * 0.00098f;  // Scaling factor for BMI270
    ay = (int16_t)((buffer[3] << 8) | buffer[2]) * 0.00098f;
    az = (int16_t)((buffer[5] << 8) | buffer[4]) * 0.00098f;

    gx = (int16_t)((buffer[7] << 8) | buffer[6]) * 0.00106f;   // Scaling factor for BMI270
    gy = (int16_t)((buffer[9] << 8) | buffer[8]) * 0.00106f;
    gz = (int16_t)((buffer[11] << 8) | buffer[10]) * 0.00106f;

    return true;
}

// Helper function to read a single register
bool ArduSensor::read_register(uint8_t reg, uint8_t &value) {
    return _dev.read_register(reg, value);
}

// Helper function to read multiple registers
bool ArduSensor::read_registers(uint8_t reg, uint8_t *buf, uint8_t length) {
    return _dev.read_registers(reg, buf, length);
}