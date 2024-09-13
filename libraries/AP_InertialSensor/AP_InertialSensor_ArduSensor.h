#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/I2CDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"



class AP_InertialSensor_ArduSensor : public AP_InertialSensor_Backend {
    public:
        ArduSensor(uint8_t address = 0x68);

        bool init();
        bool read();

        float get_ax() const { return ax; }
        float get_ay() const { return ay; }
        float get_az() const { return az; }
        float get_gx() const { return gx; }
        float get_gy() const { return gy; }
        float get_gz() const { return gz; }

    private:
        AP_HAL::Device _dev;
        uint8_t _address;
        
        float ax, ay, az, gx, gy, gz;

        bool read_register(uint8_t reg, uint8_t &value);
        bool read_registers(uint8_t reg, uint8_t *buf, uint8_t length);
}
