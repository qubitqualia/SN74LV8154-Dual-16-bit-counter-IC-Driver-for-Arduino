/*counter.cpp - Source file for SN74LV8154 IC driver
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

#include "Arduino.h"
#include "counter.h"
#include "shiftreg.h"
#include <math.h>

void CounterIC::set_data_pins(uint8_t pins[8]) {
/*Data pins on SN74LV8154
 *Only need to call this function when using the SN74LV8154
 *parallel output pins
 *  Y0: Pin 19	[Data output bit 0 (LSB)]
 *  Y1: Pin 18  [Data output bit 1]
 *  Y2: Pin 17  [Data output bit 2]
 *  Y3: Pin 16  [Data output bit 3]
 *  Y4: Pin 15  [Data output bit 4]
 *  Y5: Pin 14  [Data output bit 5]
 *  Y6: Pin 13  [Data output bit 6]
 *  Y7: Pin 12	[Data output bit 7]
 */

	Y0_pin = pins[0];
	Y1_pin = pins[1];
	Y2_pin = pins[2];
	Y3_pin = pins[3];
	Y4_pin = pins[4];
	Y5_pin = pins[5];
	Y6_pin = pins[6];
	Y7_pin = pins[7];
}

void CounterIC::set_gate_pins(uint8_t gau, uint8_t gal) {
/*Gate A pins on SN74LV8154
 *  GAL: Pin 3  [Gate A lower byte; active-low puts lower byte of stored counter A on Y bus]
 *  GAU: Pin 4  [Gate A upper byte; active-low puts upper byte of stored counter A on Y bus]
 */
	GAU_pin = gau;
	GAL_pin = gal;
}

void CounterIC::set_gate_pins(uint8_t gau, uint8_t gal, uint8_t gbu, uint8_t gbl) {
/*Gate A and B pins on SN74LV8154
 *  GAL: Pin 3  [Gate A lower byte; active-low puts lower byte of stored counter A on Y bus]
 *  GAU: Pin 4  [Gate A upper byte; active-low puts upper byte of stored counter A on Y bus]
 *  GBL: Pin 5  [Gate B lower byte; active-low puts lower byte of stored counter B on Y bus]
 *  GBU: Pin 6  [Gate B upper byte; active-low puts upper byte of stored counter B on Y bus]
 */
	GAU_pin = gau;
	GAL_pin = gal;
	GBU_pin = gbu;
	GBL_pin = gbl;
}

void CounterIC::set_clear_pin(uint8_t cclr) {
/*CCLR pin on SN74LV8154
 *  CCLR: Pin 11  [Clock clear, async active-low clear for both counters]
 */
	CCLR_pin = cclr;
}

void CounterIC::set_regclock_pin(uint8_t rclk) {
/*RCLK pin on SN74LV8154
 *  RCLK: Pin 7  [Register clock, rising edge stores counters into internal storage register]
 */
	RCLK_pin = rclk;
}

void CounterIC::set_clkben_pin(uint8_t clkben) {
/*Clock B enable pin on SN74LV8154
 *  CLKBEN: Pin 9  [Clock B enable; active-low allows clocking for counter B; connect to RCOA for 32-bit counter]
 */
	CLKBEN_pin = clkben;
}

void CounterIC::set_test_pins(uint8_t a) {
/*Test pin for Counter A on SN74LV8154
 *Set this pin to a digital output pin on the Arduino to perform controlled testing
 *of the counter; Connect to CLKA (Pin 1 on SN74LV8154)
 */
	a_trig_pin = a;
}

void CounterIC::set_test_pins(uint8_t a, uint8_t b) {
/*Test pins for Counters A and B on SN74LV8154
 *Set these pins to two digital output pins on the Arduino to perform controlled testin
 *of counters A and B; Connect to CLKA (Pin 1 on SN74LV8154) and CLKB (Pin 2)
 */
	a_trig_pin = a;
	b_trig_pin = b;
}

void CounterIC::set_testA_freq(uint32_t fa) {
/*Set frequency in Hz for testing Counter A
 *
 */
	a_freq = fa;
}

void CounterIC::set_testB_freq(uint32_t fb) {
/*Set frequency in Hz for testing Counter B
 *
 */
	b_freq = fb;
}

void CounterIC::set_serial_conn(ShiftRegIC* s) {
/*Provide a pointer to a ShiftRegIC object that will allow data reads from
 *the SN74LV8154 to be performed via a single serial wire connection to the Arduino
 */
	serial_output = s;
}

void CounterIC::init() {
/*This function must be called just prior to entering the loop
 *
 */

	//Single 32-bit counter mode requires specific pin configuration
	if (strcmp(mode, "single") == 0) {
		_single = true;
		//Single counter mode requires CLKBEN to be connected to RCOA
		if (CLKBEN_pin != 255 || RCOA_pin != 255) {
			Serial.println("fatal error: From CounterIC::init() -- cannot define CLKBEN or RCOA pins for single counter mode.");
			while(1);
		}
	}
	else {
		_single = false;
	}

	//Initialize Gate pins
	if (GAL_pin == 255 || GAU_pin == 255 || GBL_pin == 255 || GBU_pin == 255) {
		Serial.println("fatal error: From CounterIC::init() -- all gate pins must be defined!");
		while(1);
	}
	else {
		pinMode(GAL_pin, OUTPUT);
		pinMode(GAU_pin, OUTPUT);
		pinMode(GBL_pin, OUTPUT);
		pinMode(GBU_pin, OUTPUT);
		digitalWrite(GAL_pin, HIGH);
		digitalWrite(GAU_pin, HIGH);
		digitalWrite(GBL_pin, HIGH);
		digitalWrite(GBU_pin, HIGH);
	}

	//Initialize Register Clock
	if (RCLK_pin != 255) {
		pinMode(RCLK_pin, OUTPUT);
		digitalWrite(RCLK_pin, LOW);
	}
	else {
		Serial.println("fatal error: From CounterIC::init() -- register clock pin must be defined!");
	}

	//Initialize overflow status pin
	if (RCOA_pin != 255) {
		_overflow = true;
		pinMode(RCOA_pin, INPUT_PULLUP);
	}
	else {
		_overflow = false;
	}

	//Initialize Clock B enable pin
	if (CLKBEN_pin != 255) {
		_clkBenable = true;
		_toggle = true;
		pinMode(CLKBEN_pin, OUTPUT);
		digitalWrite(CLKBEN_pin, LOW);
	}
	else {
		_clkBenable = true;
		_toggle = false;
	}

	//Initialize Shift Register connection, if present
	if (serial_output != NULL) {
		_shift = true;
		if (Y0_pin != 255) {
			Serial.print("warning: From CounterIC::init() -- pins defined for parallel data output will not be used since shift register object has been specified.");
		}
	}
	else {
		_shift = false;
		if (Y0_pin == 255) {
			Serial.println("fatal error: From CounterIC::init() -- data pins (Y0-Y7) must be defined!");
			while(1);
		}
	}

	//Initialize CCLR pin
	if (CCLR_pin != 255) {
		_clear = true;
		pinMode(CCLR_pin, OUTPUT);
		digitalWrite(CCLR_pin, HIGH);
	}
	else {
		_clear = false;
	}

	//Initialize test pins
	if (a_trig_pin != 255) {
		_testA = true;
		pinMode(a_trig_pin, OUTPUT);
		digitalWrite(a_trig_pin, LOW);
		testA_delay = (uint32_t) round((1.0/a_freq)*1E6);
		_resetTimers = true;
	}
	else {
		_testA = false;
	}
	if (b_trig_pin != 255) {
		_testB = true;
		pinMode(b_trig_pin, OUTPUT);
		digitalWrite(b_trig_pin, LOW);
		testB_delay = (uint32_t) round((1.0/b_freq)*1E6);
		_resetTimers = true;
	}
	else {
		_testB = false;
	}
}

void CounterIC::update() {
/*This function must be called in the main loop if testing of Counters
 *A/B is being performed.  It can also be used to monitor for overflow condition
 *on Counter A.  Alternatively, overFlow() function can be called for this purpose
 */
	_updateRunning = true;
	tf_2 = micros();
	tf_3 = micros();


	//Handle timers for TestA and TestB signals
	if (_testA || _testB) {
		if (_resetTimers) {
			t0_2 = micros();
			t0_3 = micros();
			tf_2 = micros();
			tf_3 = micros();
			_resetTimers = false;
		}
	}

	if (_testA) {
		if (_resetTimer2) {
			t0_2 = micros();
			tf_2 = micros();
			_resetTimer2 = false;
		}

		if (tf_2 - t0_2 >= testA_delay/2) {
			digitalWrite(a_trig_pin, !digitalRead(a_trig_pin));
			_resetTimer2 = true;
		}
	}

	if (_testB) {
		if (_resetTimer3) {
			t0_3 = micros();
			tf_3 = micros();
			_resetTimer3 = false;
		}

		if (tf_3 - t0_3 >= testB_delay/2) {
			digitalWrite(b_trig_pin, !digitalRead(b_trig_pin));
			_resetTimer3 = true;
		}
	}

	//Handle Overflow status
	if (_overflow) {
		if (digitalRead(RCOA_pin) == LOW) {
			Serial.println("Counter A is full!");
		}
	}
}

uint32_t CounterIC::readCounter(const char* ab) {
/*Read the value stored on the internal register of the SN74LV8154
 *Argument: "A" for counter A
 *			"B" for counter B
 *If a shift register object has been attached to this counter object
 *it will automatically be used to retrieve the counter values.  Otherwise,
 *this function will attempt to read the parallel data pins from the
 *SN74LV8154 by default.
 */
	uint32_t data_out = 0x00;

	if (_testA || _testB) {
		if (!_updateRunning) {
			Serial.println("fatal error: From CounterIC::readCounter() -- CounterIC::Update() must be called in loop");
			while(1);
		}
	}

	digitalWrite(RCLK_pin, HIGH);
	delayMicroseconds(2);
	digitalWrite(RCLK_pin, LOW);

	//Read A counter
	if (strcmp(ab, "A") == 0 || strcmp(ab, "a") == 0) {
		digitalWrite(GAU_pin, LOW);
		delayMicroseconds(2);
		if (_shift) {data_out = serial_output->readByte(true);}
		else {data_out = readDataPins();}
		digitalWrite(GAU_pin, HIGH);

		digitalWrite(GAL_pin, LOW);
		delayMicroseconds(2);
		if (_shift) {data_out = (data_out << 8) | serial_output->readByte(true);}
		else {data_out = (data_out << 8) | readDataPins();}
		digitalWrite(GAL_pin, HIGH);
	}

	//Read B counter
	if (strcmp(ab, "B") == 0 || strcmp(ab, "b") == 0) {
		if (_clkBenable) {
			digitalWrite(GBU_pin, LOW);
			delayMicroseconds(2);
			if (_shift) {data_out = serial_output->readByte(true);}
			else {data_out = readDataPins();}
			digitalWrite(GBU_pin, HIGH);

			digitalWrite(GBL_pin, LOW);
			delayMicroseconds(2);
			if (_shift) {data_out = (data_out << 8) | serial_output->readByte(true);}
			else {data_out = (data_out << 8) | readDataPins();}
			digitalWrite(GBL_pin, HIGH);
		}
		else {
			Serial.println("warning: From CounterIC::readCounter() -- counter B is not enabled, cannot read.");
		}
	}

	return data_out;
}

uint32_t CounterIC::readCounter_32bit() {
/*Read the 32-bit value stored on the internal register of the SN74LV8154
 * This function can only be called when the IC is configured as a single 32-bit counter
 * by either connecting the CLKBEN pin to the RCOA pin or toggling Counter B on using this
 * library when an overflow occurs on Counter A.
 */
	uint32_t ret = 0xFFFF;
	if (_single) {
		uint32_t high_byte = readCounter("A");
		uint32_t low_byte = readCounter("B");
		ret = (high_byte << 16) | low_byte;
	}
	else {
		Serial.println("error: From CounterIC::readCounter_32bit() -- this function cannot be called for dual 16-bit counters configuration");
	}

	return ret;
}

void CounterIC::toggleCounterB() {
/*Toggles the state of the CLKBEN pin on the SN74LV8154
 *
 */
	if (_toggle) {
		digitalWrite(CLKBEN_pin, !digitalRead(CLKBEN_pin));
		if (digitalRead(CLKBEN_pin) == HIGH) {_clkBenable = false;}
		else {_clkBenable = true;}
	}
	else {
		Serial.println("error: From CounterIC::toggleCounterB() -- cannot toggle counter B since CLKBEN pin was not defined.");
	}
}

bool CounterIC::overFlow() {
/*Returns the status of the overflow pin RCOA
 *
 */
	bool ret = false;

	if (_overflow) {
		if (digitalRead(RCOA_pin) == HIGH) {
			ret = false;
		}
		else {
			ret = true;
		}
	}
	else {
		Serial.println("error: From CounterIC::overFlow() -- this function cannot be called because RCOA pin was not defined");
	}

	return ret;
}

bool CounterIC::enabledCounterB() {
/*Returns ON/OFF status of Counter B
 *
 */
	bool ret = false;

	if (_toggle) {
		if (digitalRead(CLKBEN_pin) == HIGH) {
			ret = false;
		}
		else {
			ret = true;
		}
	}
	else {
		Serial.println("error: From CounterIC::enabledCounterB() -- this function cannot be called because CLKBEN pin was not undefined");
	}
	return ret;
}

void CounterIC::clearCounters() {
/*This function clears the values stored in the internal register in the SN74LV8154
 *for both counters A and B.
 */
	digitalWrite(CCLR_pin, LOW);
	delayMicroseconds(2);
	digitalWrite(CCLR_pin, HIGH);
}

uint32_t CounterIC::readDataPins() {
	uint32_t val = 0x00;
	uint32_t data_out = 0x00;

	val = digitalRead(Y7_pin);
	data_out = (data_out << 1) | val;
	val = digitalRead(Y6_pin);
	data_out = (data_out << 1) | val;
	val = digitalRead(Y5_pin);
	data_out = (data_out << 1) | val;
	val = digitalRead(Y4_pin);
	data_out = (data_out << 1) | val;
	val = digitalRead(Y3_pin);
	data_out = (data_out << 1) | val;
	val = digitalRead(Y2_pin);
	data_out = (data_out << 1) | val;
	val = digitalRead(Y1_pin);
	data_out = (data_out << 1) | val;
	val = digitalRead(Y0_pin);
	data_out = (data_out << 1) | val;

	return data_out;
}

