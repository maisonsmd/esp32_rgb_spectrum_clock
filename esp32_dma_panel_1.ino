#define PIXEL_COLOR_DEPTH_BITS	8
#define MATRIX_WIDTH			128
#define MATRIX_HEIGHT			64

#define SCHEDULER_SOURCE current_ms

#include "Schedule.h"

#include <ESP32-RGB64x32MatrixPanel-I2S-DMA.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>

#if 1

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>

#include <driver/i2s.h>
#define I2S_WS 22
#define I2S_SD 32
#define I2S_SCK 33
#define I2S_PORT I2S_NUM_0
#include "arduinoFFT.h"

// ----------------------------

RGB64x32MatrixPanel_I2S_DMA panel(true);

const char* ssid = "KHANH TY T1B";
const char* password = "11112222";
//const char* ssid = "MS";
//const char* password = "11110101";

// weather stuff
float temperature;
String weather;
const char * url = "http://api.openweathermap.org/data/2.5/weather?q=Danang,VN&appid=183d1985ae57374a885437e5e342999c";
HTTPClient http;
struct tm timeinfo;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7 * 3600;
const int   daylightOffset_sec = 0;

// FFT stuff
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
const uint16_t samples = 1024; //This value MUST ALWAYS be a power of 2
const uint16_t samplingFrequency = 44100;
float vReal[samples];
float vImag[samples];
float log_table[128]; // double screen height

enum class SpectrumDisplayMode : uint8_t {
	vertical_bar,
	mirror
};

SpectrumDisplayMode spectrum_display_mode;

constexpr uint16_t indexes[128] = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,85,87,89,91,93,95,97,99,102,105,108,111,114,117,121,125,129,133,138,143,148,154,160,166,173,180,187,195,203,212,222,232,243,254,266,279,293,307,322,338,355,373,393,414 };

float band_magnitude[128] = {};

const float max_in_vol = 10000;
const float max_out_vol = 63;

constexpr int16_t n_columns = 128;
constexpr int16_t column_width = 128 / n_columns;
uint16_t peaks[n_columns];
uint32_t peaks_last_ms[n_columns];
uint32_t peaks_last_update = 0;

int16_t sample_count = 0;

uint32_t sound_start = 0;
uint32_t last_sound_detected = 0;
bool sound_detected = false;
bool is_showing_fft = false;
bool last_is_showing_fft = false;

// animation stuff
enum class ClockAnimation {
	collapsing,
	expanding,
	none
};

ClockAnimation clock_animation = ClockAnimation::none, last_clock_animation = ClockAnimation::none;

float anim_frame_big_x = 8, anim_frame_big_y = 27, anim_frame_small_x = 96, anim_frame_small_y = 2, anim_frame_big_w = 112, anim_frame_small_w = 6 * 5, anim_frame_big_h = 35, anim_frame_small_h = 7;
float percent = 0;
float anim_frame_x, anim_frame_y, anim_frame_w, anim_frame_h;

const uint16_t primary_color = panel.color565(150, 150, 150);
const uint16_t secondary_color = panel.color565(120, 110, 120);
bool dot = false;

void i2s_install() {
	const i2s_config_t i2s_config = {
	  i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),	// mode
	  44100,										// sample_rate
	  i2s_bits_per_sample_t(16),					// bits_per_sample
	  I2S_CHANNEL_FMT_ONLY_LEFT,					// channel_format
	  i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),	// communication_format
	  0,											// intr_alloc_flags, default interrupt priority
	  8,											// dma_buf_count
	  128,											// dma_buf_len
	  true											// use_apll
	};

	i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {
	const i2s_pin_config_t pin_config = {
	  I2S_SCK,	// bck_io_num
	  I2S_WS,	// ws_io_num
	  -1,		// data_out_num
	  I2S_SD	// data_in_num
	};

	i2s_set_pin(I2S_PORT, &pin_config);
}

void fetch_weather() {
	http.begin(url);

	// Send HTTP GET request
	int httpResponseCode = http.GET();
	if (httpResponseCode > 0) {
		Serial.print("HTTP Response code: ");
		Serial.println(httpResponseCode);

		DynamicJsonDocument doc(1024);
		deserializeJson(doc, http.getString());
		JsonObject obj = doc.as<JsonObject>();

		temperature = obj["main"]["feels_like"].as<float>() - 273;
		weather = obj["weather"][0]["main"].as<String>();
	}
	else {
		Serial.print("Error code: ");
		Serial.println(httpResponseCode);
	}
	// Free resources
	http.end();
};

// return true if calculation is completed
bool fft_calc() {
	auto calc_band_magnitude = [&](float * raw_mag) -> float {
		for (int16_t i = 0; i < 128; i++) {
			auto i_start = i; if (i_start > 1) i_start -= 1; if (i_start > 1) i_start -= 1;
			auto i_end = i; if (i_end < 127) i_end += 1; if (i_end < 127) i_end += 1;

			uint16_t band_index = indexes[i_start];
			uint16_t next_index = indexes[i_end];
			float average_mag = 0;
			//float band_max = 0;
			for (uint16_t j = band_index; j < next_index; ++j) {
				average_mag += raw_mag[j];
				//if (raw_mag[j] > band_max) band_max = raw_mag[j];
			}
			average_mag /= next_index - band_index;

			auto mag = map(average_mag, 0, max_in_vol, 0, max_out_vol);
			mag = constrain(mag, 0, 127);
			band_magnitude[i] = mag * log_table[mag];

			//band_magnitude[i] = map(band_max, 0, max_in_vol, 0, max_out_vol);
			//band_magnitude[i] = constrain(band_magnitude[i], 0, max_out_vol);
		}
	};

	int16_t sample = 0;
	if (sample_count < 1024) {
		int bytes = i2s_pop_sample(I2S_PORT, (char*)&sample, 10);
		//bytes = i2s_pop_sample(I2S_PORT, (char*)&sample, portMAX_DELAY);
		if (bytes > 0) {
			vReal[sample_count] = sample;
			vImag[sample_count++] = 0;
		}

		return false;
	}

	sample_count = 0;

	float freq = 0;
	float volume = 0;

	FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
	FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
	FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
	FFT.MajorPeak(vReal, samples, samplingFrequency, &freq, &volume);

	calc_band_magnitude(vReal);

	const uint32_t current_ms = millis();
	// make it less sensitive when clock is showing
	if (volume > (is_showing_fft ? 15000 : 22000)) {
		last_sound_detected = current_ms;
		if (!sound_detected) {
			Serial.println("sound start");
			sound_start = current_ms;
		}
		sound_detected = true;
	}

	if (sound_detected
		&& current_ms > sound_start + 7000
		&& current_ms < last_sound_detected + 3000) {
		is_showing_fft = true;
	}

	if (sound_detected
		&& current_ms > last_sound_detected + 15000) {
		sound_detected = false;
		Serial.println("sound ended");
		is_showing_fft = false;
	}

	/*Serial.print(band_magnitude[0]);
	Serial.print(' ');
	Serial.println(band_magnitude[1]);*/

	//if (!is_showing_fft) return;

	/*uint8_t fail = 0;
	for (int16_t x = 0; x < 32; ++x) {
		float y;
		for (int i = 0; i < 4; ++i) {
			y += band_magnitude[x * 4 + i];
		}
		y /= 4;

		if (y > 40) fail++;
	}*/

	//if (fail > 16) return false;

	return true;
}

void fft_draw() {
	const uint32_t current_ms = millis();
	for (int16_t x = 0; x < n_columns; ++x) {
		float y = 0;
		for (int i = 0; i < column_width; ++i) {
			y += band_magnitude[x * column_width + i];
		}
		y /= column_width;
		y *= 0.7 + 2.0 * (float)x / n_columns;

		if (y > peaks[x]) {
			peaks[x] = y;
			peaks_last_ms[x] = current_ms;
		}

		const auto drawFireLine = [](int16_t x, int16_t mag, uint16_t color) {
			// x2 not used,
			//panel.drawLine(x1, y1, x2, y2, panel.color565(map(y2, 0, 64, 250, 0), map(y2, 0, 64, 0, 250), 0));
			for (auto y = 63; y >= 63 - mag; --y) {
				if (y < 0) break;

				auto r = map(y, 63, 0, 150, 255);
				r = constrain(r, 0, 255);
				auto g = map(y, 20, 63, 0, 220);
				g = constrain(g, 0, 255);
				auto b = map(x, 0, 128, 0, 150);
				b = constrain(b, 0, 255);
				panel.drawPixel(x, y, panel.color565(r, g, b));
			}
		};

		const auto drawFireLineMirror = [](int16_t x, int16_t mag, uint16_t color) {
			mag /= 2;
			// x2 not used,
			//panel.drawLine(x1, y1, x2, y2, panel.color565(map(y2, 0, 64, 250, 0), map(y2, 0, 64, 0, 250), 0));
			for (auto y = 0; y <= mag; ++y) {
				auto g = map(y, 0, 32, 170, 0);
				g = constrain(g, 0, 255);
				auto r = map(y, 5, 32, 150, 255);
				r = constrain(r, 0, 255);
				auto b = map(x, 0, 128, 0, 150);
				b = constrain(b, 0, 255);

				panel.drawPixel(x, 32 + y, panel.color565(r, g, b));
				panel.drawPixel(x, 32 - y, panel.color565(r, g, b));
			}
		};

		y = constrain(y, 0, 64);
		//panel.drawRect(x * 4, 63 - y, 3, y, panel.Color888(50, 0, 50));

		auto space = column_width == 1 ? 0 : 1;
		switch (spectrum_display_mode) {
		case SpectrumDisplayMode::mirror:
			drawFireLineMirror(x * column_width + space, y, 0);
			//panel.fillRect(x * column_width, 63 - peaks[x] - 1, column_width, 2, panel.color565(120, 120, 80));
			break;
		case SpectrumDisplayMode::vertical_bar:
			drawFireLine(x * column_width + space, y, 0);
			panel.fillRect(x * column_width, 63 - peaks[x] - 1, column_width, 2, panel.color565(120, 120, 80));

			if (current_ms > peaks_last_update + 2) {
				peaks_last_update = current_ms;
				for (int16_t x = 0; x < n_columns; ++x) {
					if (current_ms > peaks_last_ms[x] + 350)
						if (peaks[x] > 0) peaks[x] -= 1;
				}
			}
			break;
		}
	}
}

void setup() {
	Serial.begin(115200);

	panel.setPanelBrightness(20);
	panel.setMinRefreshRate(60);

	panel.begin();
	panel.fillScreen(panel.color565(0, 0, 0));

	panel.println("Connecting to");
	panel.println();
	panel.println(ssid);
	panel.println();

	WiFi.begin(ssid, password);
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		panel.print(".");
	}
	panel.clearScreen();
	panel.setCursor(0, 0);
	panel.print("connected\n\nfetching time...");

	//init and get the time
	configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
	fetch_weather();

	i2s_install();
	i2s_setpin();
	i2s_start(I2S_PORT);

	// pre-calc log_table for speed
	for (int16_t i = 0; i < 128; ++i) {
		float x = (float)i / 64;
		x = constrain(x, 0, 1.01);
		log_table[i] = log10(120 * (1 - x * 0.97))*0.7;
		Serial.print(log_table[i]);
		Serial.print(',');
	}
}

void loop() {
	const uint32_t current_ms = millis();

	const uint8_t hour = timeinfo.tm_hour;
	const uint8_t minute = timeinfo.tm_min;

	if (is_showing_fft != last_is_showing_fft) {
		last_is_showing_fft = is_showing_fft;

		if (is_showing_fft) {
			clock_animation = ClockAnimation::collapsing;
			spectrum_display_mode = spectrum_display_mode == SpectrumDisplayMode::mirror
				? SpectrumDisplayMode::vertical_bar
				: SpectrumDisplayMode::mirror;
			panel.setMinRefreshRate(200);
		}
		else {
			clock_animation = ClockAnimation::expanding;
			panel.setMinRefreshRate(60);
		}
	}

	DO_EVERY(10 * 60000) {
		fetch_weather();
	}

	DO_EVERY(5000) {
		if (!getLocalTime(&timeinfo))
			Serial.println("Failed to obtain time");

		switch (hour) {
		case 0 ... 5:
		case 23:
			panel.setPanelBrightness(2);
			break;
		case 6:
		case 19 ... 22:
			panel.setPanelBrightness(10);
			break;
		case 7 ... 18:
			panel.setPanelBrightness(20);
			break;
		default:
			break;
		}
	}

	if (Serial.available()) {
		is_showing_fft = !is_showing_fft;
		Serial.println(is_showing_fft ? "true" : "false");
		delay(5);
		while (Serial.available()) Serial.read();
	}

	bool fft_updatable = fft_calc();

	DO_EVERY(500) {
		dot = !dot;

		if (!is_showing_fft && clock_animation == ClockAnimation::none) {
			panel.flipDMABuffer();
			panel.clearScreen();

			panel.setTextColor(secondary_color);
			panel.setFont(nullptr);
			panel.setCursor(8, 2);
			char buf[64];

			strftime(buf, 64, "%A, ", &timeinfo);
			String str = String(buf);
			strftime(buf, 64, "%B", &timeinfo);
			str += String(buf).substring(0, min(4, (int)strlen(buf)));
			strftime(buf, 64, " %d", &timeinfo);
			str += String(buf);
			panel.print(str);
			//panel.print(&timeinfo, "%A, %B %d");

			panel.setCursor(8, 13);
			panel.print(temperature, 1);
			panel.drawCircle(34, 14, 1, secondary_color);
			panel.print(" C, ");
			panel.print(weather);

			panel.setTextColor(primary_color);
			panel.setFont(&FreeSansBold24pt7b);
			panel.setCursor(5, 60);

			panel.printf("%2d", hour);
			if (dot) panel.setTextColor(primary_color);
			else panel.setTextColor(0);
			panel.print(":");
			panel.setTextColor(primary_color);
			panel.printf("%02d", minute);

			panel.showDMABuffer();
		}
	}
	if (clock_animation != last_clock_animation) {
		last_clock_animation = clock_animation;
		percent = 0;

		if (clock_animation == ClockAnimation::collapsing) {
			panel.clearScreen();
			anim_frame_x = map(percent, 0, 100, anim_frame_big_x, anim_frame_small_x);
			anim_frame_y = map(percent, 0, 100, anim_frame_big_y, anim_frame_small_y);
			anim_frame_w = map(percent, 0, 100, anim_frame_big_w, anim_frame_small_w);
			anim_frame_h = map(percent, 0, 100, anim_frame_big_h, anim_frame_small_h);
		}
		if (clock_animation == ClockAnimation::expanding) {
			panel.clearScreen();
			anim_frame_x = map(percent, 0, 100, anim_frame_small_x, anim_frame_big_x);
			anim_frame_y = map(percent, 0, 100, anim_frame_small_y, anim_frame_big_y);
			anim_frame_w = map(percent, 0, 100, anim_frame_small_w, anim_frame_big_w);
			anim_frame_h = map(percent, 0, 100, anim_frame_small_h, anim_frame_big_h);
		}
	}

	if (is_showing_fft && clock_animation == ClockAnimation::none && fft_updatable) {
		panel.flipDMABuffer();
		panel.clearScreen();
		fft_draw();

		panel.setFont(nullptr);
		panel.setCursor(96, 2);
		panel.printf("%2d", hour);
		panel.setTextColor(dot ? secondary_color : panel.color565(0, 0, 0));
		panel.print(":");
		panel.setTextColor(secondary_color);
		panel.printf("%02d", minute);

		panel.showDMABuffer();
	}

	if (clock_animation != ClockAnimation::none && fft_updatable) {
		percent += 15;
		percent = constrain(percent, 0, 100);

		panel.flipDMABuffer();
		panel.clearScreen();
		//panel.drawRect(anim_frame_x, anim_frame_y, anim_frame_w, anim_frame_h, panel.color565(0, 0, 0));
		if (clock_animation == ClockAnimation::collapsing) {
			anim_frame_x = map(percent, 0, 100, anim_frame_big_x, anim_frame_small_x);
			anim_frame_y = map(percent, 0, 100, anim_frame_big_y, anim_frame_small_y);
			anim_frame_w = map(percent, 0, 100, anim_frame_big_w, anim_frame_small_w);
			anim_frame_h = map(percent, 0, 100, anim_frame_big_h, anim_frame_small_h);
		}
		if (clock_animation == ClockAnimation::expanding) {
			anim_frame_x = map(percent, 0, 100, anim_frame_small_x, anim_frame_big_x);
			anim_frame_y = map(percent, 0, 100, anim_frame_small_y, anim_frame_big_y);
			anim_frame_w = map(percent, 0, 100, anim_frame_small_w, anim_frame_big_w);
			anim_frame_h = map(percent, 0, 100, anim_frame_small_h, anim_frame_big_h);
		}
		panel.drawRect(anim_frame_x, anim_frame_y, anim_frame_w, anim_frame_h, primary_color);

		// draw spectrum while collapsing
		if (clock_animation == ClockAnimation::collapsing)
			fft_draw();

		if (percent == 100) {
			if (clock_animation == ClockAnimation::expanding) {
				panel.clearScreen();
				// fast draw clock
				panel.setTextColor(primary_color);
				panel.setFont(&FreeSansBold24pt7b);
				panel.setCursor(5, 60);

				panel.printf("%2d", hour);
				if (dot) panel.setTextColor(primary_color);
				else panel.setTextColor(0);
				panel.print(":");
				panel.setTextColor(primary_color);
				panel.printf("%02d", minute);
			}
			clock_animation = ClockAnimation::none;
		}
		panel.showDMABuffer();
	}
}

#else

RGB64x32MatrixPanel_I2S_DMA panel(true);
bool sw = true;
bool dot = true;

void setup() {
	Serial.begin(115200);
	Serial.setTimeout(5);

	panel.setPanelBrightness(20);
	panel.setMinRefreshRate(60);

	panel.begin();
	panel.fillScreen(panel.color565(0, 0, 0));
	panel.setTextWrap(false);
}

const uint16_t primary_color = panel.color565(150, 150, 150);
const uint16_t secondary_color = panel.color565(120, 110, 120);

enum class ClockAnimation {
	collapsing,
	expanding,
	none
};

ClockAnimation clock_animation = ClockAnimation::none, last_clock_animation = ClockAnimation::none;
bool anim_show_clock = true;
bool anim_show_spectrum = true;

float anim_frame_big_x = 8, anim_frame_big_y = 27, anim_frame_small_x = 96, anim_frame_small_y = 2, anim_frame_big_w = 112, anim_frame_small_w = 6 * 5, anim_frame_big_h = 35, anim_frame_small_h = 7;
float percent = 0;
float anim_frame_x, anim_frame_y, anim_frame_w, anim_frame_h;

void loop() {
	auto hh = millis() / 3600000;
	auto mm = (millis() % 3600000) / 60000;
	panel.flipDMABuffer();
	panel.clearScreen();
	if (Serial.available()) {
		sw = !sw;
		Serial.println(sw ? "true" : "false");
		delay(5);
		while (Serial.available()) Serial.read();

		if (!sw) clock_animation = ClockAnimation::collapsing;
		else clock_animation = ClockAnimation::expanding;
	}

	static long last_dot = 0;
	if (millis() > last_dot + 500) {
		last_dot = millis();
		dot = !dot;
	}
	if (sw) {
		panel.setTextSize(1);

		panel.setFont(nullptr);
		panel.setCursor(8, 2);
		panel.print("Wed, sept 09");

		panel.setCursor(8, 13);
		panel.print(25.1, 1);
		panel.drawCircle(34, 14, 1, secondary_color);
		panel.print(" C, ");
		panel.print("clouds");

		panel.setTextColor(primary_color);
		panel.setFont(&FreeSansBold24pt7b);
		panel.setCursor(5, 60);

		panel.printf("%2d", hh);
		if (dot) panel.setTextColor(primary_color);
		else panel.setTextColor(0);
		panel.print(":");
		panel.setTextColor(primary_color);
		panel.printf("%02d", mm);
	}
	else {
		panel.setFont(nullptr);
		panel.setCursor(96, 2);
		panel.printf("%2d", hh);
		panel.setTextColor(dot ? secondary_color : panel.color565(0, 0, 0));
		panel.print(":");
		panel.setTextColor(secondary_color);
		panel.printf("%02d", mm);
	}

	if (clock_animation != last_clock_animation) {
		last_clock_animation = clock_animation;
		percent = 0;

		if (clock_animation == ClockAnimation::collapsing) {
			anim_show_clock = false;
			anim_show_spectrum = true;

			anim_frame_x = map(percent, 0, 100, anim_frame_big_x, anim_frame_small_x);
			anim_frame_y = map(percent, 0, 100, anim_frame_big_y, anim_frame_small_y);
			anim_frame_w = map(percent, 0, 100, anim_frame_big_w, anim_frame_small_w);
			anim_frame_h = map(percent, 0, 100, anim_frame_big_h, anim_frame_small_h);
		}
		if (clock_animation == ClockAnimation::expanding) {
			anim_show_clock = false;
			anim_show_spectrum = false;

			anim_frame_x = map(percent, 0, 100, anim_frame_small_x, anim_frame_big_x);
			anim_frame_y = map(percent, 0, 100, anim_frame_small_y, anim_frame_big_y);
			anim_frame_w = map(percent, 0, 100, anim_frame_small_w, anim_frame_big_w);
			anim_frame_h = map(percent, 0, 100, anim_frame_small_h, anim_frame_big_h);
		}
	}
	if (clock_animation != ClockAnimation::none) {
		percent += 5;
		percent = constrain(percent, 0, 100);

		panel.drawRect(anim_frame_x, anim_frame_y, anim_frame_w, anim_frame_h, panel.color565(0, 0, 0));
		if (clock_animation == ClockAnimation::collapsing) {
			anim_frame_x = map(percent, 0, 100, anim_frame_big_x, anim_frame_small_x);
			anim_frame_y = map(percent, 0, 100, anim_frame_big_y, anim_frame_small_y);
			anim_frame_w = map(percent, 0, 100, anim_frame_big_w, anim_frame_small_w);
			anim_frame_h = map(percent, 0, 100, anim_frame_big_h, anim_frame_small_h);
		}
		if (clock_animation == ClockAnimation::expanding) {
			anim_frame_x = map(percent, 0, 100, anim_frame_small_x, anim_frame_big_x);
			anim_frame_y = map(percent, 0, 100, anim_frame_small_y, anim_frame_big_y);
			anim_frame_w = map(percent, 0, 100, anim_frame_small_w, anim_frame_big_w);
			anim_frame_h = map(percent, 0, 100, anim_frame_small_h, anim_frame_big_h);
		}
		panel.drawRect(anim_frame_x, anim_frame_y, anim_frame_w, anim_frame_h, secondary_color);

		if (percent == 100) {
			if (clock_animation == ClockAnimation::collapsing) {
				anim_show_clock = false;
				anim_show_spectrum = true;
			}
			if (clock_animation == ClockAnimation::expanding) {
				anim_show_clock = true;
				anim_show_spectrum = false;
			}
			clock_animation = ClockAnimation::none;
		}
	}

	panel.showDMABuffer();
}

#endif