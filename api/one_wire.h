/*
 * pico-pi-one-wire Library, derived from the mbed DS1820 Library, for the
 * Dallas (Maxim) 1-Wire Devices
 * Copyright (c) 2010, Michael Hagberg <Michael@RedBoxCode.com>
 *
 * This version uses a single instance to talk to multiple one wire devices.
 * During configuration the devices will be listed and the addresses
 * then stored within the system  they are associated with.
 *
 * Then previously stored addresses are used to query devices.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef PICO_PI_ONEWIRE_H
#define PICO_PI_ONEWIRE_H

#ifdef MOCK_PICO_PI

#include "../test/pico_pi_mocks.h"

#else

#include "hardware/gpio.h"
#include "pico/time.h"

#endif

#define FAMILY_CODE address.rom[0]
#define FAMILY_CODE_DS18S20 0x10 //9bit temp
#define FAMILY_CODE_DS18B20 0x28 //9-12bit temp also known as MAX31820
#define FAMILY_CODE_DS1822 0x22  //9-12bit temp
#define FAMILY_CODE_MAX31826 0x3B//12bit temp + 1k EEPROM
#define FAMILY_CODE_DS2404 0x04  //RTC
#define FAMILY_CODE_DS2417 0x27  //RTC
#define FAMILY_CODE_DS2740 0x36  //Current measurement
#define FAMILY_CODE_DS2502 0x09  //1k EEPROM

static const int ReadScratchPadCommand = 0xAA;
static const int WriteScratchPadCommand = 0x0F;
static const int CopyScratchPadCommand = 0x55;
static const int ReadMemoryCommand = 0xF0;

static const int MatchROMCommand = 0x55;
static const int ReadROMCommand = 0x33;
static const int SearchROMCommand = 0xF0;
static const int SkipROMCommand = 0xCC;

static const int ROMSize = 8;
struct rom_address_t {
	uint8_t rom[ROMSize];
};

/**
 * OneWire with DS1820 Dallas 1-Wire Temperature Probe
 *
 * Example:
 * @code
 * #include "one_wire.h"
 *
 * One_wire one_wire(15); //GP15 - Pin 20 on Pi Pico
 *
 * int main() {
 *     one_wire.init();
 *     rom_address_t address{};
 *     while (true) {
 *         one_wire.single_device_read_rom(address);
 *         one_wire.convert_temperature(address, true, true);
 *         printf("It is %3.1foC\n", one_wire.temperature(address));
 *         sleep_ms(1000);
 *     }
 * }
 * @endcode
 */
class One_wire {
public:
	uint8_t ram[10];
	uint8_t ta1;
	uint8_t ta2;
	uint8_t e_s;
	/** Create a one wire bus object connected to the specified pins
	 *
	 * The bus might either by regular powered or parasite powered. If it is parasite
	 * powered and power_pin is set, that pin will be used to switch an external mosfet
	 * connecting data to Vdd. If it is parasite powered and the pin is not set, the
	 * regular data pin is used to supply extra power when required. This will be
	 * sufficient as long as the number of devices is limited.
	 *
	 * @param data_pin pin for the data bus
	 */
	One_wire(uint data_pin)
		: _data_pin(data_pin) {
	}

	~One_wire();

	/**
	 * Initialise and determine if any devices are using parasitic power
	 */
	void init();

	/**
	 * Finds all one wire devices and returns the count
	 *
	 * @return - number of devices found
	 */
	int find_and_count_devices_on_bus();

	/**
	 * Get address of devices previously found
	 *
	 * @param index the index into found devices
	 * @return the address of
	 */
	static rom_address_t &get_address(int index);

	/**
	 * Assuming a single device is attached, do a Read ROM
	 *
	 * @param rom_address the address will be filled into this parameter
	 */
	void single_device_read_rom(rom_address_t &rom_address);

	/**
	 * Static utility method for easy conversion from previously stored addresses
	 *
	 * @param hex_address the address as a human readable hex string
	 * @return the rom address
	 */
	static rom_address_t address_from_hex(const char *hex_address);

	int read_scratch_pad(rom_address_t &address);

	int write_scratch_pad(rom_address_t &address, uint16_t data_address);

	int read_memory(rom_address_t &address, uint16_t data_address, uint8_t* data, int len);

	int copy_scratch_pad(rom_address_t &address);

private:
	uint _data_pin;

	int _last_discrepancy;	// search state
	bool _last_device;  // search state

	static uint8_t crc_byte(uint8_t crc, uint8_t byte);

	static void bit_write(uint8_t &value, int bit, bool set);

	[[nodiscard]] bool reset_check_for_device() const;

	void match_rom(rom_address_t &address);

	void skip_rom();

	void onewire_bit_out(bool bit_data) const;

	void onewire_byte_out(uint8_t data);

	[[nodiscard]] bool onewire_bit_in() const;

	uint8_t onewire_byte_in();

	static bool rom_checksum_error(uint8_t *address);

	bool ram_checksum_error();

	bool search_rom_find_next();
};


#endif// PICO_PI_ONEWIRE_H