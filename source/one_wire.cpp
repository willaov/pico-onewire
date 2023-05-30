#include "../api/one_wire.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#ifdef MOCK_PICO_PI

#include "../test/pico_pi_mocks.h"

#else

#include "hardware/gpio.h"

#endif

std::vector<rom_address_t> found_addresses;

void One_wire::init() {
	gpio_init(_data_pin);
	gpio_set_drive_strength(_data_pin, GPIO_DRIVE_STRENGTH_2MA);
}

One_wire::~One_wire() {
	found_addresses.clear();
}

bool One_wire::reset_check_for_device() const {
	// This will return false if no devices are present on the data bus
	bool presence = false;
	gpio_init(_data_pin);
	gpio_set_dir(_data_pin, GPIO_OUT);
	gpio_put(_data_pin, false); // bring low for 480us
	sleep_us(480);
	gpio_set_dir(_data_pin, GPIO_IN); // let the data line float high
	sleep_us(70); // wait 70us
	if (!gpio_get(_data_pin)) {
		// see if any devices are pulling the data line low
		presence = true;
	}
	sleep_us(410);
	return presence;
}

void One_wire::onewire_bit_out(bool bit_data) const {
	gpio_set_dir(_data_pin, GPIO_OUT);
	gpio_put(_data_pin, false);
	sleep_us(3);// (spec 1-15us)
	if (bit_data) {
		gpio_put(_data_pin, true);
		sleep_us(55);
	} else {
		sleep_us(60);// (spec 60-120us)
		gpio_put(_data_pin, true);
		sleep_us(5);// allow bus to float high before next bit_out
	}
}

void One_wire::onewire_byte_out(uint8_t data) {
	int n;
	for (n = 0; n < 8; n++) {
		onewire_bit_out((bool) (data & 0x01));
		data = data >> 1;// now the next bit is in the least sig bit position.
	}
}

bool One_wire::onewire_bit_in() const {
	bool answer;
	gpio_set_dir(_data_pin, GPIO_OUT);
	gpio_put(_data_pin, false);
	sleep_us(3);// (spec 1-15us)
	gpio_set_dir(_data_pin, GPIO_IN);
	sleep_us(3);// (spec read within 15us)
	answer = gpio_get(_data_pin);
	sleep_us(45);
	return answer;
}

uint8_t One_wire::onewire_byte_in() {
	uint8_t answer = 0x00;
	int i;
	for (i = 0; i < 8; i++) {
		answer = answer >> 1;// shift over to make room for the next bit
		if (onewire_bit_in())
			answer = (uint8_t) (answer | 0x80);// if the data port is high, make this bit a 1
	}
	return answer;
}

int One_wire::find_and_count_devices_on_bus() {
	found_addresses.clear();
	_last_discrepancy = 0;	// start search from begining
	_last_device = 0;
	while (search_rom_find_next()) {
	}
	return (int) found_addresses.size();
}

rom_address_t One_wire::address_from_hex(const char *hex_address) {
	rom_address_t address = rom_address_t();
	for (uint8_t i = 0; i < ROMSize; i++) {
		char buffer[3];
		strncpy(buffer, &hex_address[i * 2], 2);
		buffer[2] = '\0';
		address.rom[i] = (uint8_t) strtol(buffer, nullptr, 16);
	}
	return address;
}

rom_address_t &One_wire::get_address(int index) {
	return found_addresses[index];
}

void One_wire::bit_write(uint8_t &value, int bit, bool set) {
	if (bit <= 7 && bit >= 0) {
		if (set) {
			value |= (1 << bit);
		} else {
			value &= ~(1 << bit);
		}
	}
}

void One_wire::single_device_read_rom(rom_address_t &rom_address) {
	if (!reset_check_for_device()) {
		return;
	} else {
		onewire_byte_out(ReadROMCommand);
		for (int bit_index = 0; bit_index < 64; bit_index++) {
			bool bit = onewire_bit_in();
			bit_write((uint8_t &) rom_address.rom[bit_index / 8], (bit_index % 8), bit);
		}
	}
}

bool One_wire::search_rom_find_next() {
	uint8_t search_ROM[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	int discrepancy_marker, rom_bit_index;
	bool bitA, bitB;
	uint8_t byte_counter, bit_mask;

	if (!reset_check_for_device()) {
		// printf("Failed to reset one wire bus\n");
		return false;
	} else {
		if (_last_device) {
			return false;	// all devices found
		}
		rom_bit_index = 1;
		discrepancy_marker = 0;
		onewire_byte_out(SearchROMCommand);
		byte_counter = 0;
		bit_mask = 0x01;
		while (rom_bit_index <= 64) {
			bitA = onewire_bit_in();
			bitB = onewire_bit_in();
			if (bitA & bitB) {
				discrepancy_marker = 0;// data read error, this should never happen
				rom_bit_index = 0xFF;
				printf("Data read error - no devices on bus?\r\n");
			} else {
				if (bitA | bitB) {
					// Set ROM bit to Bit_A
					if (bitA) {
						search_ROM[byte_counter] =
								search_ROM[byte_counter] | bit_mask;// Set ROM bit to one
					} else {
						search_ROM[byte_counter] =
								search_ROM[byte_counter] & ~bit_mask;// Set ROM bit to zero
					}
				} else {
					// both bits A and B are low, so there are two or more devices present
					if (rom_bit_index == _last_discrepancy) {
						search_ROM[byte_counter] =
								search_ROM[byte_counter] | bit_mask;// Set ROM bit to one
					} else {
						if (rom_bit_index > _last_discrepancy) {
							search_ROM[byte_counter] =
									search_ROM[byte_counter] & ~bit_mask;// Set ROM bit to zero
							discrepancy_marker = rom_bit_index;
						} else {
							if ((search_ROM[byte_counter] & bit_mask) == 0x00)
								discrepancy_marker = rom_bit_index;
						}
					}
				}
				onewire_bit_out(search_ROM[byte_counter] & bit_mask);
				rom_bit_index++;
				if (bit_mask & 0x80) {
					byte_counter++;
					bit_mask = 0x01;
				} else {
					bit_mask = bit_mask << 1;
				}
			}
		}
		_last_discrepancy = discrepancy_marker;
		if (rom_bit_index != 0xFF) {
			#ifdef _LIST_ROMS
			printf ("Found %02x%02x%02x%02x%02x%02x%02x%02x\n",
				search_ROM[0], search_ROM[1], search_ROM[2], search_ROM[3],
				search_ROM[4], search_ROM[5], search_ROM[6], search_ROM[7]
				);
			#endif

			if (rom_checksum_error(search_ROM)) {// Check the CRC
				printf("failed crc\r\n");
				return false;
			}
			rom_address_t address{};
			for (byte_counter = 0; byte_counter < 8; byte_counter++) {
				address.rom[byte_counter] = search_ROM[byte_counter];
			}
			found_addresses.push_back(address);
			_last_device = _last_discrepancy == 0;
			return true;
		} else {
			return false;
		}
	}
}

void One_wire::match_rom(rom_address_t &address) {
	int i;
	if (reset_check_for_device()) {
		onewire_byte_out(MatchROMCommand);
		for (i = 0; i < 8; i++) {
			onewire_byte_out(address.rom[i]);
		}
	} else {
		printf("match_rom failed\n");
	}
}

void One_wire::skip_rom() {
	if (reset_check_for_device()) {
		onewire_byte_out(SkipROMCommand);
	} else {
		printf("skip_rom failed\n");
	}
}

bool One_wire::rom_checksum_error(uint8_t *address) {
	uint8_t crc = 0x00;
	int i;
	for (i = 0; i < 7; i++) {// Only going to shift the lower 7 bytes
		crc = crc_byte(crc, address[i]);
	}
	// After 7 bytes CRC should equal the 8th byte (ROM CRC)
	return (crc != address[7]);// will return true if there is a CRC checksum mis-match
}

bool One_wire::ram_checksum_error() {
	uint8_t crc = 0x00;
	int i;
	for (i = 0; i < 8; i++) {// Only going to shift the lower 8 bytes
		crc = crc_byte(crc, ram[i]);
	}
	// After 8 bytes CRC should equal the 9th byte (RAM CRC)
	return (crc != ram[8]);// will return true if there is a CRC checksum mis-match
}

uint8_t One_wire::crc_byte(uint8_t crc, uint8_t byte) {
	int j;
	for (j = 0; j < 8; j++) {
		if ((byte & 0x01) ^ (crc & 0x01)) {
			// DATA ^ LSB CRC = 1
			crc = crc >> 1;
			// Set the MSB to 1
			crc = (uint8_t) (crc | 0x80);
			// Check bit 3
			if (crc & 0x04) {
				crc = (uint8_t) (crc & 0xFB);// Bit 3 is set, so clear it
			} else {
				crc = (uint8_t) (crc | 0x04);// Bit 3 is clear, so set it
			}
			// Check bit 4
			if (crc & 0x08) {
				crc = (uint8_t) (crc & 0xF7);// Bit 4 is set, so clear it
			} else {
				crc = (uint8_t) (crc | 0x08);// Bit 4 is clear, so set it
			}
		} else {
			// DATA ^ LSB CRC = 0
			crc = crc >> 1;
			// clear MSB
			crc = (uint8_t) (crc & 0x7F);
			// No need to check bits, with DATA ^ LSB CRC = 0, they will remain unchanged
		}
		byte = byte >> 1;
	}
	return crc;
}

int One_wire::read_scratch_pad(rom_address_t &address) {
	match_rom(address);
	onewire_byte_out(ReadScratchPadCommand);
	ta1 = onewire_byte_in();
	ta2 = onewire_byte_in();
	e_s = onewire_byte_in();
	for (int i = 0; i < 10; i++) {
		ram[i] = onewire_byte_in();
	}
	return 0;
}

int One_wire::write_scratch_pad(rom_address_t &address, uint16_t data_address) {
	uint8_t data_addr_lsb = (uint8_t) data_address;
	uint8_t data_addr_msb = (uint8_t) (data_address >> 8);
	match_rom(address);
	onewire_byte_out(WriteScratchPadCommand);
	onewire_byte_out(data_addr_lsb);// TA1
	onewire_byte_out(data_addr_msb);// TA2
	for (int i=0; i < 8; i++) {
		onewire_byte_out(ram[i]);
	}
	ram[8] = onewire_byte_in();
	ram[9] = onewire_byte_in();
	return 0;
}

int One_wire::copy_scratch_pad(rom_address_t &address) {
	match_rom(address);
	onewire_byte_out(CopyScratchPadCommand);
	onewire_byte_out(ta1);
	onewire_byte_out(ta2);
	onewire_byte_out(e_s);
	gpio_put(_data_pin, true);
	sleep_ms(15);
	if (onewire_byte_in() == 0xAA) {
		return 0;
	} else if (onewire_byte_in() == 0xFF) {
		return 1;
	} else {
		return 10;
	}
}

int One_wire::read_memory(rom_address_t &address, uint16_t data_address, uint8_t* data, int len) {
	uint8_t data_addr_lsb = (uint8_t) data_address;
	uint8_t data_addr_msb = (uint8_t) (data_address >> 8);
	match_rom(address);
	onewire_byte_out(ReadMemoryCommand);
	onewire_byte_out(data_addr_lsb);// TA1
	onewire_byte_out(data_addr_msb);// TA2
	for (int i = 0; i < len; i++) {
		data[i] = onewire_byte_in();
	}
	return 0;
}
