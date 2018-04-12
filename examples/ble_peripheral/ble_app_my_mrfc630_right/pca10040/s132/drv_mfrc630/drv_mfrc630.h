#ifndef _DRV_MFRC630_H_
#define _DRV_MFRC630_H_

#include "mfrc630_def.h"
#include "mfrc630.h"
#include "nrf_drv_twi.h"

typedef void (*drv_mfrc630_data_handler_t)(drv_mfrc630_data_t const * p_evt);

typedef struct
{
    nrf_drv_twi_t         const * p_twi_instance;
    nrf_drv_twi_config_t  const * p_twi_cfg;
    uint8_t                       twi_addr;
    drv_mfrc630_data_handler_t      data_handler;
} drv_mfrc630_init_t;

uint32_t drv_mfrc630_init(drv_mfrc630_init_t * p_init);

uint32_t drv_mfrc630_sample(void);
