#include "esp32.h"
#include <Wire.h>

ESP32::ESP32()
{

}

void ESP32::init()
{

}

uint32_t ESP32::ip()
{
    uint8_t buffer[4];
    i2cReadBytes(ESP32_COMMAND_IP, buffer, 4);
    return (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | (buffer[3] << 0);
}

void ESP32::set_wifi_ssid(uint8_t *buf, uint8_t buflen)
{
    i2cWriteBytes(ESP32_COMMAND_WIFI_SSID, buf, buflen);
}

void ESP32::set_wifi_password(uint8_t *buf, uint8_t buflen)
{
    i2cWriteBytes(ESP32_COMMAND_WIFI_PASSWORD, buf, buflen);
}

void ESP32::wifi_reconnect()
{
	i2cWriteBytes(ESP32_COMMAND_WIFI_RECONNECT, nullptr, 0);
}

// Read a specified number of bytes over I2C at a given subAddress
int16_t ESP32::i2cReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	Wire.beginTransmission((uint8_t)ESP32_I2C_ADDRESS);
	Wire.write(subAddress);
	Wire.endTransmission(true);

	Wire.requestFrom((uint8_t)ESP32_I2C_ADDRESS, count);

	for (int i=0; i<count; i++)
	{
		dest[i] = Wire.read();
	}

	return true;
}

// Write a specified number of bytes over I2C to a given subAddress
uint16_t ESP32::i2cWriteBytes(uint8_t subAddress, uint8_t * src, uint8_t count)
{
	Wire.beginTransmission((uint8_t)ESP32_I2C_ADDRESS);
	Wire.write(subAddress);
	for (int i=0; i<count; i++)
	{
		Wire.write(src[i]);
	}
	Wire.endTransmission(true);

	return true;
}