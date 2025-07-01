#pragma once

class hmc5883
{
	public:
		hmc5883(uint8_t bus, uint8_t address, int gauss, int pi);
		~hmc5883();
                const int HMC5883L_DEFAULT_ADDRESS =      0x1E;
		int read_byte(uint8_t adr) const;
		int read_word(uint8_t adr);
		int read_word_2c(uint8_t adr);
		void write_byte(uint8_t adr, int byte) const;
		float* get_raw();
		float* get_calibrated();
		float* get_axes();
		float heading(double ax, double ay);
		void calibration(void);

	protected:
		float bias[3] = {30.797967, -93.492117, -78.505766};
		float calibration_matrix[3][3] =
			{{1.152886, 0.024759, 0.017571},
                        {0.024759, 1.132549, -0.030021},
                        {0.017571, -0.030021, 1.232850}};

	private:
	        const int HMC5883L_CONFIGURATION_A =      0;
        	const int HMC5883L_CONFIGURATION_B =      1;
        	const int HMC5883L_MODE =                 2;
        	const int HMC5883L_OUTX_MBS =             3;
        	const int HMC5883L_OUTX_LSB =             4;
        	const int HMC5883L_OUTZ_MBS =             5;
        	const int HMC5883L_OUTZ_LSB =             6;
        	const int HMC5883L_OUTY_MBS =             7;
        	const int HMC5883L_OUTY_LSB =             9;
        	const int HMC5883L_STATUS =               10;
        	const int HMC5883L_IDENTIFICATION_A =     11;
        	const int HMC5883L_IDENTIFICATION_B =     12;
        	//const int HMC5883L_IDENTIFICATION_B =     13;

		int _handle;
		int _pi;
		int _bus = 0;
		int _address = 0;
		int _reg = 1;
		int _scale = 0.92;
};
