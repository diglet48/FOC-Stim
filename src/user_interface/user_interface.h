#ifndef FOCSTIM_USER_INTERFACE_H
#define FOCSTIM_USER_INTERFACE_H
#ifdef BOARD_FOCSTIM_V4


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


class UserInterface {
public:
	enum UIState {
		DetectBattery,
		Idle,
		Connected,
		Playing,
		Error,
	};

	UserInterface();

	// TODO: detect display
	void init();

	// TODO: smarter refresh
	void refresh();

	void setState(UIState state) {
		this->state = state;
	}
	void setPowerLevel(float power) {
		this->power = power;
	}
	void setBatteryPresent(bool is_present) {
		this->battery_is_present = is_present;
	}
	void setBatterySoc(float soc) {
		this->battery_soc = soc;
	}
	void setIP(uint32_t ip) {
		this->ip = ip;
	}

	void hexdump();

private:
	Adafruit_SSD1306 display;
	UIState state;

	float power;
	bool battery_is_present;
	float battery_soc;
	uint32_t ip;
};

#endif
#endif