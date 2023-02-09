#include "flash.h"

#include "stm32g4xx_hal.h"
#include "log.h"

#include <cassert>

#define W25Q32JV_DEVICE_ID 0x15


#define PAGE_PROGRAM					0x02
#define READ_DATA						0x03
#define READ_STATUS_REGISTER_1			0x05
#define WRITE_ENABLE					0x06
#define BLOCK_ERASE_32KB				0x52
#define RELEASE_POWER_DOWN_DEVICE_ID	0xAB


#define FLIGHT_FLASH_BUSY_MASK 			0x01
#define FLIGHT_FLASH_WEL_MASK  			0x02


static uint32_t flash_status_1();

static void spi_begin()
{
	HAL_GPIO_WritePin(flash->csPinBank, flash->csPin, GPIO_PIN_RESET);
}

static void spi_end()
{
	HAL_GPIO_WritePin(flash->csPinBank, flash->csPin, GPIO_PIN_SET);
}

void flash_setup()
{
	HAL_GPIO_WritePin(flash->csPinBank, flash->csPin, GPIO_PIN_SET);

	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::RELEASE_POWER_DOWN_DEVICE_ID);
	for (size_t i = 0; i < 3; ++i) {
		FLASH_SPI.transfer(0);
	}
	uint8_t device_id = FLASH_SPI.transfer(0);
	spi_end();

	if (device_id != W25Q32JV_DEVICE_ID) {
		Serial.println(device_id);
		Serial.println(F("Failed to set up flash!"));
		abort();
	} else {
		Serial.println(F("Flash detected."));
	}
}

void flash_write_enable()
{
	assert(!flash_busy());

	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::WRITE_ENABLE);
	spi_end();

	assert(flash_status_1() & FLIGHT_FLASH_WEL_MASK);
}

static uint32_t flash_status_1()
{
	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::READ_STATUS_REGISTER_1);
	uint8_t status = FLASH_SPI.transfer(0);
	spi_end();
	return status;
}

bool flash_busy()
{
	return static_cast<bool>(flash_status_1() & FLIGHT_FLASH_BUSY_MASK);
}

void flash_write(size_t page_addr, uint8_t page[FLIGHT_FLASH_PAGE_SIZE])
{
	flash_write_enable();

	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::PAGE_PROGRAM);

	FLASH_SPI.transfer((page_addr >> 8) & 0xFF);
	FLASH_SPI.transfer(page_addr & 0xFF);
	FLASH_SPI.transfer(0);

	for (size_t i = 0; i < FLIGHT_FLASH_PAGE_SIZE; ++i) {
		FLASH_SPI.transfer(page[i]);
	}
	spi_end();

	assert(flash_busy());

#ifndef NDEBUG
	// Validate write
	while (flash_busy()) { delayMicroseconds(1); }

	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::READ_DATA);

	FLASH_SPI.transfer((page_addr >> 8) & 0xFF);
	FLASH_SPI.transfer(page_addr & 0xFF);
	FLASH_SPI.transfer(0);

	for (size_t i = 0; i < FLIGHT_FLASH_PAGE_SIZE; ++i) {
		assert(page[i] == FLASH_SPI.transfer(0));
	}
	spi_end();
#endif
}

void flash_read(size_t page_addr, uint8_t page[FLIGHT_FLASH_PAGE_SIZE])
{
	assert(!flash_busy());

	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::READ_DATA);

	FLASH_SPI.transfer((page_addr >> 8) & 0xFF);
	FLASH_SPI.transfer(page_addr & 0xFF);
	FLASH_SPI.transfer(0);

	for (size_t i = 0; i < FLIGHT_FLASH_PAGE_SIZE; ++i) {
		page[i] = FLASH_SPI.transfer(0);
	}
	spi_end();
}

void flash_erase(size_t page_addr)
{
	flash_write_enable();
	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::BLOCK_ERASE_32KB);

	FLASH_SPI.transfer((page_addr >> 8) & 0xFF);
	FLASH_SPI.transfer(page_addr & 0xFF);
	FLASH_SPI.transfer(0x00);
	spi_end();

	assert(flash_busy());
}
