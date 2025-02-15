#ifndef __ENC_MT6816_H_
#define __ENC_MT6816_H_
#include "stm32g4xx_hal.h"
#include "spi.h"
#include <stdbool.h>

typedef struct sMT6816
{
    bool     no_mag;
    bool     over_speed;
    uint32_t angle;
    uint8_t rx_err_count;
    uint8_t check_err_count;
} tMT6816;

extern tMT6816  MT6816;
bool enc_mt6816_routine(void);
#define cs_down HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_RESET);
#define cs_up HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_SET);

#endif
