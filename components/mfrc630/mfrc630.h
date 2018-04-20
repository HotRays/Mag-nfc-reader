#ifndef MFRC630_H_
#define MFRC630_H_
#include <stdint.h>
#include "mfrc630_def.h"
#include "pca10040.h"
#include "nrf_drv_gpiote.h"
#define  NRF_LOG_MODULE_NAME "mfrc630     "
#include "nrf_log.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include <stdint.h>
#include <string.h>
#include "ble_nus.h"

#define MFRC630_ADDRESS 0x29	

uint32_t mfrc630_twi_init(void);

uint8_t mfrc630_read_reg(uint8_t reg);

void mfrc630_write_reg(uint8_t reg, uint8_t value);

void mfrc630_write_regs(uint8_t reg, const uint8_t* values, uint8_t len);

void mfrc630_write_fifo(const uint8_t* data, uint16_t len);

void mfrc630_read_fifo(uint8_t* rx, uint16_t len);

void mfrc630_cmd_read_E2(uint16_t address, uint16_t length);

void mfrc630_cmd_load_protocol(uint8_t rx, uint8_t tx);

void mfrc630_cmd_transceive(const uint8_t* data, uint16_t len);

void mfrc630_cmd_idle(void);

void mfrc630_cmd_load_key_E2(uint8_t key_nr);

void mfrc630_cmd_load_key(const uint8_t* key);

void mfrc630_cmd_auth(uint8_t key_type, uint8_t block_address, const uint8_t* card_uid);

void mfrc630_flush_fifo(void);

uint16_t mfrc630_fifo_length(void);

void mfrc630_clear_irq0(void);

void mfrc630_clear_irq1(void);

uint8_t mfrc630_irq0(void);

uint8_t mfrc630_irq1(void);

void mfrc630_print_block(const uint8_t* data, uint16_t len);

uint8_t mfrc630_transfer_E2_page(uint8_t* dest, uint8_t page);

void mfrc630_activate_timer(uint8_t timer, uint8_t active);

void mfrc630_timer_set_control(uint8_t timer, uint8_t value);

void mfrc630_timer_set_reload(uint8_t timer, uint16_t value);

void mfrc630_timer_set_value(uint8_t timer, uint16_t value);

uint16_t mfrc630_timer_get_value(uint8_t timer);

void mfrc630_AN1102_recommended_registers(uint8_t protocol);

void mfrc630_AN1102_recommended_registers_skip(uint8_t protocol, uint8_t skip);

uint16_t mfrc630_iso14443a_REQA(void);

uint16_t mfrc630_iso14443a_WUPA(void);

uint16_t mfrc630_iso14443a_WUPA_REQA(uint8_t instruction);

uint8_t mfrc630_iso14443a_select( uint8_t* uid, uint8_t* sak);

//uint8_t mfrc630_MF_auth(const uint8_t* uid, uint8_t key_type, uint8_t block);

//void mfrc630_MF_deauth(void);

//uint8_t mfrc630_MF_read_block(uint8_t block_address, uint8_t* dest);

//uint8_t mfrc630_MF_write_block(uint8_t block_address, const uint8_t* source);

//void mfrc630_AN11145_start_IQ_measurement(void);

//void mfrc630_AN11145_stop_IQ_measurement(void);

//void mfrc630_AN1102_recommended_registers_no_transmitter(uint8_t protocol);

#endif 
