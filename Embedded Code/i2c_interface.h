#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <Wire.h>
#include <ens220.h>

using namespace ScioSense;

class I2cInterface : public virtual Utils::IoInterface<ENS220::RegisterAddress, ENS220::Result>
{
public:
    I2cInterface();
public:
    void begin(TwoWire& twoWire = Wire, uint8_t address = 0x20);
public:
    virtual ENS220::Result read(const ENS220::RegisterAddress& address, uint8_t* data, const size_t& size);
    virtual ENS220::Result write(const ENS220::RegisterAddress& address, uint8_t* data, const size_t& size);
private:
    uint8_t slaveAddress;
    TwoWire* wire;
};

I2cInterface::I2cInterface()
{
    wire                = nullptr;
    slaveAddress        = 0x20;
}

void I2cInterface::begin(TwoWire& twoWire, uint8_t address)
{
    wire         = &twoWire;
    slaveAddress = address;
}

ENS220::Result I2cInterface::read(const ENS220::RegisterAddress& address, uint8_t* data, const size_t& size)
{
    ENS220::Result result = ENS220::Result::IOError;

    if (size == 0)
    {
        return result;
    }

    wire->beginTransmission(slaveAddress);
    wire->write((uint8_t)address);

    if (wire->endTransmission() == 0) // 0 == success
    {
        // the FIFO buffer of ENS220 is 3*32 bytes in size and should be read in multiples of 3.
        // the Wire.h library defines the read BUFFER_LENGTH to be 32 max.
        // to avoid Wire issues and to ensure that the FIFO is read efficiently,
        // data is requested in chunks of <= 30
        constexpr uint8_t MAX_CHUNK_SIZE = 30;

        size_t len = 0;
        while (len < size)
        {
            size_t bytesToRequest = ( MAX_CHUNK_SIZE < (size - len) ? MAX_CHUNK_SIZE : (size - len) );
            wire->requestFrom(slaveAddress, bytesToRequest);
            size_t n = wire->readBytes(data + len, bytesToRequest);
            len += n;

            if (n == 0)
            {
                break;
            }
        }

        if (len == size)
        {
            result = ENS220::Result::Ok;
        }
    }

    return result;
}

ENS220::Result I2cInterface::write(const ENS220::RegisterAddress& address, uint8_t* data, const size_t& size)
{
    ENS220::Result result = ENS220::Result::IOError;

    wire->beginTransmission(slaveAddress);
    wire->write((uint8_t)address);
    wire->write(data, size);
    if (wire->endTransmission() == 0) // 0 == success
    {
        result = ENS220::Result::Ok;
    }

    return result;
}

#endif //I2C_INTERFACE_H
