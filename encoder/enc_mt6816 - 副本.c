#include "enc_mt6816.h"

tMT6816  MT6816;
bool enc_mt6816_routine(void)
{
    uint16_t  pc1;
    uint16_t tx[3] = {0x8300, 0x8400, 0x8500};
    uint16_t rx[3] = {0};

    // CS
		cs_down;
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx[0], (uint8_t*)&rx[0], 1, 10);
		cs_up;
		cs_down;
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx[1], (uint8_t*)&rx[1], 1, 10);
		cs_up;
		cs_down;
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx[2], (uint8_t*)&rx[2], 1, 10);
		cs_up;
		
		// check pc1
    pc1 = (rx[0] << 8) | rx[1];
    pc1 ^= pc1 >> 8;
    pc1 ^= pc1 >> 4;
    pc1 ^= pc1 >> 2;
    pc1 ^= pc1 >> 1;
    if (pc1 & 1) {
        goto CHECK_ERR;
    }
		
		MT6816.no_mag     = rx[1] & 0x02;
    MT6816.over_speed = rx[2] & 0x08;
		MT6816.angle = ((rx[0] << 6) | (rx[1] >> 2));
		return true;

CHECK_ERR:
    if (MT6816.check_err_count < 0xFF) {
        MT6816.check_err_count++;
    }

    return false;
}
