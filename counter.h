/*counter.h - Include file for SN74LV8154 IC driver
  Copyright (c) 2020 Justin Holland.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
 */

#ifndef COUNTER_H_
#define COUNTER_H_

#include "Arduino.h"

class ShiftRegIC;

class CounterIC {
	public:
		const char* mode;
		ShiftRegIC* serial_output;
		uint8_t GAL_pin;
		uint8_t GAU_pin;
		uint8_t GBL_pin;
		uint8_t GBU_pin;
		uint8_t CCLR_pin;
		uint8_t RCLK_pin;
		uint8_t RCOA_pin;
		uint8_t CLKBEN_pin;
		uint8_t Y0_pin;
		uint8_t Y1_pin;
		uint8_t Y2_pin;
		uint8_t Y3_pin;
		uint8_t Y4_pin;
		uint8_t Y5_pin;
		uint8_t Y6_pin;
		uint8_t Y7_pin;
		uint8_t a_trig_pin;
		uint8_t b_trig_pin;
		uint32_t a_freq;
		uint32_t b_freq;


		//Initialization argument:
		//mode = "single" for single 32-bit counter configuration
		//mode = "dual" for dual 16-bit counters configuration
		CounterIC(const char* m) : mode(m)
		{
			GAL_pin = 255;
			GAU_pin = 255;
			GBL_pin = 255;
			GBU_pin = 255;
			CCLR_pin = 255;
			RCLK_pin = 255;
			RCOA_pin = 255;
			CLKBEN_pin = 255;
			Y0_pin = 255;
			Y1_pin = 255;
			Y2_pin = 255;
			Y3_pin = 255;
			Y4_pin = 255;
			Y5_pin = 255;
			Y6_pin = 255;
			Y7_pin = 255;
			a_trig_pin = 255;
			b_trig_pin = 255;
			a_freq = 0;
			b_freq = 0;
			serial_output = NULL;
			_clkBenable = false;
			_updateRunning = false;
			_single = false;
			_overflow = false;
			_toggle = false;
		    _shift = false;
			_clear = false;
			_testA = false;
			_testB = false;
			_resetTimers = false;
			_resetTimer1 = false;
			_resetTimer2 = false;
			_resetTimer3 = false;
			t0_1 = 0;
			t0_2 = 0;
			t0_3 = 0;
			tf_1 = 0;
			tf_2 = 0;
			tf_3 = 0;
			testA_delay = 0;
			testB_delay = 0;
		}

		CounterIC(const char* m, ShiftRegIC* s) : mode(m), serial_output(s)
		{
			GAL_pin = 255;
			GAU_pin = 255;
			GBL_pin = 255;
			GBU_pin = 255;
			CCLR_pin = 255;
			RCLK_pin = 255;
			RCOA_pin = 255;
			CLKBEN_pin = 255;
			Y0_pin = 255;
			Y1_pin = 255;
			Y2_pin = 255;
			Y3_pin = 255;
			Y4_pin = 255;
			Y5_pin = 255;
			Y6_pin = 255;
			Y7_pin = 255;
			a_trig_pin = 255;
			b_trig_pin = 255;
			a_freq = 0;
			b_freq = 0;
			_clkBenable = false;
			_updateRunning = false;
			_single = false;
			_overflow = false;
			_toggle = false;
			_shift = false;
			_clear = false;
			_testA = false;
			_testB = false;
			_resetTimers = false;
			_resetTimer1 = false;
			_resetTimer2 = false;
			_resetTimer3 = false;
			t0_1 = 0;
			t0_2 = 0;
			t0_3 = 0;
			tf_1 = 0;
			tf_2 = 0;
			tf_3 = 0;
			testA_delay = 0;
			testB_delay = 0;
		}

		//Pin setting functions
		void set_data_pins(uint8_t pins[8]);
		void set_gate_pins(uint8_t gau, uint8_t gal);
		void set_gate_pins(uint8_t gau, uint8_t gal, uint8_t gbu, uint8_t gbl);
		void set_clear_pin(uint8_t cclr);
		void set_regclock_pin(uint8_t rclk);
		void set_rcoa_pin(uint8_t rcoa);
		void set_clkben_pin(uint8_t clkben);
		void set_test_pins(uint8_t a);
		void set_test_pins(uint8_t a, uint8_t b);

		//Configuration functions
		void set_testA_freq(uint32_t fa);
		void set_testB_freq(uint32_t fb);
		void set_serial_conn(ShiftRegIC* s);

		//Initialization function
		void init();

		//Update function
		void update();

		//Counter functions
		uint32_t readCounter(const char* ab);
		uint32_t readCounter_32bit();
		void clearCounters();
		void toggleCounterB();
		bool enabledCounterB();
		bool overFlow();


	private:
		bool _clkBenable;
		bool _updateRunning;
		bool _single;
		bool _overflow;
		bool _toggle;
		bool _shift;
		bool _clear;
		bool _testA;
		bool _testB;
		bool _resetTimers;
		bool _resetTimer1;
		bool _resetTimer2;
		bool _resetTimer3;
		uint32_t t0_1;
		uint32_t t0_2;
		uint32_t t0_3;
		uint32_t tf_1;
		uint32_t tf_2;
		uint32_t tf_3;
		uint32_t testA_delay;
		uint32_t testB_delay;
		uint32_t readDataPins();

};




#endif /* COUNTER_H_ */
