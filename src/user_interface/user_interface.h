#ifndef FOCSTIM_USER_INTERFACE_H
#define FOCSTIM_USER_INTERFACE_H
#ifdef BOARD_FOCSTIM_V4


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cstring>


class UserInterface {
public:
	enum UIState {
		InitBSP,
		InitBattery,
		InitSelfTest,
		Idle,
		Connected,
		Playing,
		Error,
		SelfTestError,
	};

	UserInterface();

	// TODO: detect display
	void init();

	// repaint onto internal memory buffer (~600µs)
	void repaint();

	// full display update (~5200µs)
	void full_update();

	// update display in chunks (~260us), updates 1/32 of the display.
	void partial_update();

	void setState(UIState state) {
		dirty |= state != this->state;
		this->state = state;
	}
	void setPowerLevel(float power) {
		dirty |= power != this->power;
		this->power = power;
	}
	void setBatteryPresent(bool is_present) {
		dirty |= is_present != this->battery_is_present;
		this->battery_is_present = is_present;
	}
	void setBatterySoc(float soc) {
		dirty |= soc != this->battery_soc;
		this->battery_soc = soc;
	}
	void setIP(uint32_t ip) {
		dirty |= ip != this->ip;
		this->ip = ip;
	}

	void setSelfTestErrorString(const char* error_string) {
		strncpy(this->error_string, error_string, sizeof(this->error_string) - 1);
	}

	void hexdump();

private:
	Adafruit_SSD1306 display;
	UIState state;

	float power;
	bool battery_is_present;
	float battery_soc;
	uint32_t ip;

	bool dirty;
	int partial_update_index;

	char error_string[256];
};

#endif
#endif