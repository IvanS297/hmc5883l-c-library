#include <pigpiod_if2.h>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <cmath>
#include "hmc5883.hpp"

char* addr_str = NULL;
char* port_str = NULL;
int pi = pigpio_start(addr_str, port_str);

hmc5883::hmc5883(uint8_t bus = 1, uint8_t address = 0x1E, int gauss = 1.3, int pi = 0) {
	_pi = pi;
	_bus = bus;
	_address = address;
	_handle = i2c_open(_pi, _bus, _address, 0);
	std::cout << "I2c handle " << _handle << std::endl;
	i2c_write_byte_data(_pi, _handle, HMC5883L_CONFIGURATION_A, 0x70);
	i2c_write_byte_data(_pi, _handle, HMC5883L_CONFIGURATION_B, _reg << 5);
	i2c_write_byte_data(_pi, _handle, HMC5883L_MODE, 0x00); // continuous measurement mode
}

hmc5883::~hmc5883() {
	i2c_write_byte_data(_pi, _handle, HMC5883L_MODE, 0x03); // idle mode
	i2c_close(_pi, _handle);
}

inline int hmc5883::read_byte(uint8_t adr) const {
	return i2c_read_byte_data(_pi, _handle, adr);
}

int hmc5883::read_word(uint8_t adr) {
	int high = i2c_read_byte_data(_pi, _handle, adr);
	int low = i2c_read_byte_data(_pi, _handle, adr+1);
	int val = (high << 8) + low;
	return val;
}

int hmc5883::read_word_2c(uint8_t adr) {
	int val = read_word(adr);
	if (val >= 0x8000) val = -((65535 - val) + 1);
	if (val == -4096) throw "Incorrect Value -4096";
	return val;
}

inline void hmc5883::write_byte(uint8_t adr, int byte) const {
	i2c_write_byte_data(_pi, _handle, adr, byte);
}

float* hmc5883::get_raw() {
	float* raw = new float[3]{};
	raw[0] = static_cast<float>(read_word_2c(HMC5883L_OUTX_MBS));
	raw[1] = static_cast<float>(read_word_2c(HMC5883L_OUTY_MBS));
	raw[2] = static_cast<float>(read_word_2c(HMC5883L_OUTZ_MBS));
	return raw;
}

float* hmc5883::get_calibrated() {
	float* data = get_raw();
	float* uncalibrated_values = new float[3]{};
	float* calibrated_values = new float[3]{};
	for (uint8_t i = 0; i < 3; ++i) uncalibrated_values[i] = data[i] - bias[i];
	for (uint8_t i = 0; i < 3; ++i) {
		for (uint8_t j = 0; j < 3; ++j) {
			calibrated_values[i] += calibration_matrix[i][j] * uncalibrated_values[j];
		}
	}
	delete[] data;
	delete[] uncalibrated_values;
	return calibrated_values;
}

float* hmc5883::get_axes() {
	float* data = get_raw();
	for (uint8_t i = 0; i < 3; ++i) data[i] *= _scale;
	return data;
}

float hmc5883::heading(double ax = 0, double ay = 0) {
	float* data = get_calibrated();
	ax = ax * M_PI / 180;
	ay = ay * M_PI / 180;

	double mx[3][3] = {  {1.0, 0.0, 0.0},
                		{0.0, std::cos(ax), -std::sin(ax)},
                		{0.0, std::sin(ax), std::cos(ax)}};

	double my[3][3] = {  {std::cos(ay), 0.0, std::sin(ay)},
				{0.0, 1.0, 0.0},
				{-std::sin(ay), 0.0, std::cos(ay)}};

	float values[3]{};
	memcpy(values, data, sizeof(values));
	float* DATA = new float[3]{};
	for (uint8_t i = 0; i < 3; ++i) {
		for (uint8_t j = 0; j < 3; ++j) {
			DATA[i] += mx[i][j] * values[j];
		}
	}
	memcpy(values, DATA, sizeof(values));
	for (uint8_t i = 0; i < 3; ++i) DATA[i] = 0;
	for (uint8_t i = 0; i < 3; ++i) {
                for (uint8_t j = 0; j < 3; ++j) {
                        DATA[i] += my[i][j] * values[j];
                }
        }
	double radians = -std::atan2(DATA[1], DATA[0]);

	delete[] DATA;
	delete[] data;
	return radians * 180 / M_PI;

}

void hmc5883::calibration()
{
	std::ofstream f("HMC5883L_calibr.txt");
	if (f.is_open()) {
		std::cout << "###############################################" << std::endl;
		std::cout << "Please twirl the HMC5883L around a minute..." << std::endl;
		std::cout << "The experimental data will stored to HMC5883L_calibr.txt file" << std::endl;
		std::cout << "Please use this file to calculate calibration matrix by Magneto software." << std::endl;
		std::cout << "###############################################" << std::endl;
		float* data = new float[3];
		for (uint16_t num = 0; num < 1000; ++num) {
			data = get_raw();
			f << data[0] << '\t' << data[1] << '\t' << data[2] << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(7));
		}
		delete[] data;
		f.close();
		std::cout << "Check the HMC5883L_calibr.txt file" << std::endl;
	}
	else std::cerr << "Unable to open file." << std::endl;
}

hmc5883 compass(1, 0x1e, 1.3, pi);

int main() {
	for (int i = 0; i < 100; ++i) {
		std::cout << compass.heading() << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(70));
	}
	compass.~hmc5883();
	pigpio_stop(pi);
	return 0;
}
