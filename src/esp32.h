#ifndef FOCSTIM_ESP32_H
#define FOCSTIM_ESP32_H
#ifdef BOARD_FOCSTIM_V4

#include <cstdint>

#define ESP32_I2C_ADDRESS   0x72

#define ESP32_COMMAND_FWVERSION     	0x01
#define ESP32_COMMAND_IP            	0x02
#define ESP32_COMMAND_WIFI_SSID     	0x03
#define ESP32_COMMAND_WIFI_PASSWORD 	0x04
#define ESP32_COMMAND_WIFI_RECONNECT 	0x05


class ESP32 {
public:
    ESP32();

    void init();

    // TODO: get version
	// TODO: error checking
    uint32_t ip();

	// TODO: error checking
    void set_wifi_ssid(uint8_t *buf, uint8_t buflen);
    void set_wifi_password(uint8_t *buf, uint8_t buflen);
	void wifi_reconnect();


private:
    /**
	    Read a specified number of bytes over I2C at a given subAddress

		@param subAddress is the 8-bit address of the data to be read
		       dest is the data buffer to be written to
			   count is the number of bytes to be read
		@return true on success
	*/
	int16_t i2cReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);

	/**
	    Write a specified number of bytes over I2C to a given subAddress

		@param subAddress is the 8-bit address of the data to be written to
		       src is the data buffer to be written
			   count is the number of bytes to be written
		@return true on success
	*/
	uint16_t i2cWriteBytes(uint8_t subAddress, uint8_t * src, uint8_t count);
};

#endif
#endif