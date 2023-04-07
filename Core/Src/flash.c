#include "flash.h"
#include "util.h"
#include "main.h"

#define FLIGHT_FLASH_PAGE_SIZE (256)
#define FLIGHT_FLASH_SECTOR_SIZE (4096)
#define FLIGHT_FLASH_FLIGHTS (16)
#define FLIGHT_FLASH_PAGES_PER_SECTOR (FLIGHT_FLASH_SECTOR_SIZE / FLIGHT_FLASH_PAGE_SIZE)
// -1 for one sector to store general purpose metadata
#define FLIGHT_FLASH_SECTOR_COUNT (1024-1)
#define FLIGHT_FLASH_PAGE_COUNT (16384)
#define FLIGHT_FLASH_SECTORS_PER_FLIGHT (FLIGHT_FLASH_SECTOR_COUNT / FLIGHT_FLASH_FLIGHTS)
#define FLIGHT_FLASH_LOGS_PER_PAGE		8


// Number of log messages per sector
//int LOGS_PER_SECTOR
uint8_t LOGS_PER_PAGE = FLIGHT_FLASH_LOGS_PER_PAGE;
LogMessage logs[FLIGHT_FLASH_LOGS_PER_PAGE];
uint8_t flight = 0;
uint8_t wipe_flash = 0;
uint8_t last_wipe_flash = 0;
uint8_t flag_fullSectors = 0;

void LogInit() {
	W25qxx_Init();

	//uint32_t size = FLIGHT_FLASH_PAGE_SIZE / sizeof(LogMessage);

	// Get the current flight from the flash memory
	W25qxx_ReadSector(&flight, 0, 0, 1);
	W25qxx_ReadSector(&last_wipe_flash, 0, 1, 1);

	if (last_wipe_flash != wipe_flash) {
		W25qxx_EraseChip();
		flight = 0;
	} else {
		flight = (flight + 1) % FLIGHT_FLASH_FLIGHTS;
	}
}

void LogFlightSetup() {
	// Rewrite the flight
	// Got to erase before writing.
	// Writing seems to only set bits to 0.
	// Erasing sets all bits in Sector/Block to 1.
	W25qxx_EraseSector(0);
	W25qxx_WriteSector(&flight, 0, 0, 1);
	W25qxx_WriteSector(&wipe_flash, 0, 1, 1);


	// Erase all sectors for the current flight selected
	for (int i = 0; i < FLIGHT_FLASH_SECTORS_PER_FLIGHT; i++) {
		// Offset 1 sector so that the metadata sector is preserved
		uint32_t sector_i = 1 + flight * FLIGHT_FLASH_SECTORS_PER_FLIGHT + i;

		W25qxx_EraseSector(sector_i);
	}

}

void LogRead() {

	//uint32_t pages_per_flight = FLIGHT_FLASH_PAGES_PER_SECTOR * FLIGHT_FLASH_SECTORS_PER_FLIGHT;

	uint32_t pre_time = 0x00FFFFFF;
	uint32_t p_flight = 1;

	for (int i = 1; i < FLIGHT_FLASH_PAGE_COUNT; i++) {
		W25qxx_ReadPage((uint8_t *)logs, i, 0, sizeof(logs));

		for (int j = 0; j < FLIGHT_FLASH_LOGS_PER_PAGE; j++) {
			uint8_t check = calc_checksum((uint8_t *)(&logs[j]), sizeof(LogMessage) - 4);
			if (check == logs[j].checksum) {
				if (pre_time > logs[j].time_ms) {
					printf("START Flight Recording #%lu ---\n", p_flight );
					p_flight++;
				}
				printf("%lu, %f, %f, %f, %f, %f, %f\n",
						logs[j].time_ms,
						logs[j].accel_x,
						logs[j].accel_y,
						logs[j].accel_z,
						logs[j].gyro_x,
						logs[j].gyro_y,
						logs[j].gyro_z
				);

				pre_time = logs[j].time_ms;
				HAL_Delay(1);
			}
		}
	}

	return;

	for (int i = 0; i < FLIGHT_FLASH_FLIGHTS; i++) {
		printf("START Flight Recording #%i ---\n", i);
		for (int j = 0; j < FLIGHT_FLASH_SECTORS_PER_FLIGHT; j++) {
			uint32_t sector_i = 1 + i * FLIGHT_FLASH_SECTORS_PER_FLIGHT + j;
			for (int k = 0; k < FLIGHT_FLASH_PAGES_PER_SECTOR; k++) {
				uint32_t page_i = sector_i * FLIGHT_FLASH_PAGES_PER_SECTOR + k;

				W25qxx_ReadSector((uint8_t *)logs, page_i, 0, sizeof(logs));

				for (int log_index = 0; log_index < LOGS_PER_PAGE; log_index++) {
					uint8_t check = calc_checksum((uint8_t *)(&logs[log_index]), sizeof(LogMessage) - 4);
					printf("%lu, %f, %f, %f, %f, %f, %f\n",
							logs[log_index].time_ms,
							logs[log_index].accel_x,
							logs[log_index].accel_y,
							logs[log_index].accel_z,
							logs[log_index].gyro_x,
							logs[log_index].gyro_y,
							logs[log_index].gyro_z
					);
					if ((logs[log_index].checksum) == check) {

					} else {
//						printf("%u != %u | Checksum doesn't match\n", logs[k].checksum, check);
						log_index = LOGS_PER_PAGE;
						k = FLIGHT_FLASH_PAGES_PER_SECTOR;
						j = FLIGHT_FLASH_SECTORS_PER_FLIGHT;
					}
				}
			HAL_Delay(1);
			}
		}
		printf("END   Flight Recording #%i ---\n", i);
	}
}

// Works!!
void LogWrite() {
	static uint8_t page_sel = 0;
	static uint8_t sector_sel = 0;
	if (flag_fullSectors != 1) {
		uint32_t sector_i = 1 + flight * FLIGHT_FLASH_SECTORS_PER_FLIGHT + sector_sel;
		uint32_t page_i = sector_i * FLIGHT_FLASH_PAGES_PER_SECTOR + page_sel;

		W25qxx_WritePage((uint8_t *)logs, page_i, 0, sizeof(logs));
		logs[0].time_ms = 0;
		logs[7].time_ms = 0;
		W25qxx_ReadPage((uint8_t *)logs, page_i, 0, sizeof(logs));
//		W25qxx_WriteSector((uint8_t *)logs, sector_i, 0, sizeof(logs));
		page_sel++;
		if (page_sel >= FLIGHT_FLASH_PAGES_PER_SECTOR) {
			sector_sel++;
			page_sel = 0;
		}
		if (sector_sel >=  FLIGHT_FLASH_SECTORS_PER_FLIGHT) {
			flag_fullSectors = 1;
		}
	}
}

void LogData(uint32_t time_ms,
			float accel_x, float accel_y, float accel_z,
			float gyro_x, float gyro_y, float gyro_z) {
	static uint8_t log_index = 0;
	static uint32_t lastWrite = 0;
	if (log_index >= LOGS_PER_PAGE) {
		printf("Writing to Flash!!\n");
		uint32_t startWrite = HAL_GetTick();
		LogWrite();
		uint32_t endWrite = HAL_GetTick();
		printf("WriteSector Time: %lu\n", endWrite - startWrite);
		printf("Time Between Writes: %lu\n", startWrite - lastWrite);
		lastWrite = HAL_GetTick();
		log_index = 0;
	}
	logs[log_index].time_ms = time_ms;
	logs[log_index].accel_x = accel_x;
	logs[log_index].accel_y = accel_y;
	logs[log_index].accel_z = accel_z;
	logs[log_index].gyro_x = gyro_x;
	logs[log_index].gyro_y = gyro_y;
	logs[log_index].gyro_z = gyro_z;
	logs[log_index].checksum = calc_checksum((uint8_t *)(&logs[log_index]), sizeof(LogMessage) - 4);
	log_index++;
}

void LogWipe() {
	W25qxx_EraseChip();
}


