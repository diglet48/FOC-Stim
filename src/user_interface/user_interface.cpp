#ifdef BOARD_FOCSTIM_V4

#include "user_interface.h"


#include "user_interface/images/battery.xbm"
#include "user_interface/images/battery_empty.xbm"
#include "user_interface/images/battery_empty2.xbm"

#include "stim_clock.h"
#include "bsp/bsp.h"


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET -1		// Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32


UserInterface::UserInterface()
    : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1'000'000ULL, 100'000ULL)
    , state(UIState::DetectBattery)
{
}

void UserInterface::init()
{
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
	if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
	{
		Serial.println(F("SSD1306 allocation failed"));
		for (;;)
			; // Don't proceed, loop forever
	}

    // Show initial display buffer contents on the screen --
	// the library initializes this with an Adafruit splash screen.
    display.display();

    // Clear the buffer
	// display.clearDisplay();
}

void UserInterface::refresh()
{
    // clear display
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // ###############
    // ## display power level
    switch (state) {
        case UIState::DetectBattery:
        case UIState::Error:
            break;

        case UIState::Idle:
        case UIState::Connected:
        case UIState::Playing:
            display.setTextSize(1);
            display.setCursor(0, -2);
            display.print("power:");

            int a = int(power);
            display.setTextSize(3);
            display.setCursor(0, 11);
            display.printf("%02i", a);

            int b = int(power * 10) % 10;
            if (a < 100) {
                display.setTextSize(1);
                display.setCursor(12*3 - 2, 25);
                display.printf(".%1i", b);
            }
        break;
    }

    // ###############
    // ## display battery charge level
    switch (state) {
        case UIState::DetectBattery:
        case UIState::Idle:
        case UIState::Connected:
        case UIState::Playing:
        case UIState::Error:

        int x0 = 104;
        int y0 = 0;

        if (battery_is_present) {
            int soc = min(100, max(int(battery_soc * 100), 0));
            if (soc < 10) {
                display.setCursor(x0+3+6, y0+2);
            } else if (soc < 100) {
                display.setCursor(x0+3+3, y0+2);
            } else {
                display.setCursor(x0+3, y0+2);
            }
            display.drawXBitmap(x0, 0, battery_bits, battery_width, battery_height, SSD1306_WHITE);
            display.setTextSize(1);
            display.printf("%i", soc);
        } else {
            // display.drawXBitmap(x0, 0, battery_empty_bits, battery_empty_width, battery_empty_height, SSD1306_WHITE);
            display.drawXBitmap(x0, 0, battery_empty2_bits, battery_empty2_width, battery_empty2_height, SSD1306_WHITE);
        }
    }

    // ###############
    // ## status msg
    display.setTextSize(1);
    switch (state) {
        case UIState::DetectBattery:
            display.setCursor(128 - 6*10, 31-7);
            display.print("detect bat");
            break;
        case UIState::Idle:
            if (ip) {
                display.setCursor(128 - 6*11, 31-7-8);
                display.printf("ip: %3u.%3u\n", (ip >> 24) & 0xFF, (ip >> 16) & 0xFF);
                display.setCursor(128 - 6*11, 31-7);
                display.printf("   .%3u.%3u", (ip >> 8) & 0xFF, (ip >> 0) & 0xFF);
            } else {
                display.setCursor(128 - 6*10, 31-7);
                display.print("   no wifi");
            }
            break;
        case UIState::Connected:
            display.setCursor(128 - 6*10, 31-7);
            display.print(" connected");
            break;
        case UIState::Playing:
            display.setCursor(128 - 6*10, 31-7);
            display.print("   playing");
            break;
        case UIState::Error:
            display.setCursor(128 - 6*10, 31-7);
            display.print("     error");
            break;
    }

    display.display();
}

void UserInterface::hexdump()
{
    Serial.printf("dumping display data\r\n");
    const uint8_t* buf = display.getBuffer();
    Serial.print("hex_data = [\r\n");
    for (int row = 0; row < 32; row++) {
        Serial.print("    [");
        for (int col = 0; col < (128/8); col++) {
            uint8_t u = 0;
            for (int pixel = 0; pixel < 8; pixel++) {
                u = u << 1;
                u |= display.getPixel(col * 8 + pixel, row);
            }


            if (col != 0) {
                Serial.printf(", ");
            }
            // Serial.printf("0x%x", buf[row * SCREEN_HEIGHT + col]);
            Serial.printf("0x%x", u);
        }
        Serial.print("],");
        Serial.println();
    }
    Serial.print("]\r\n");
}


#endif