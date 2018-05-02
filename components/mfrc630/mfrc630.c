/*
  The MIT License (MIT)

  Copyright (c) 2016 Ivor Wanders

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "mfrc630.h"
#include "nrf_drv_twi.h"
#include "twi_manager.h"
#include "mfrc630_def.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);   
volatile static bool twi_tx_done = false;
volatile static bool twi_rx_done = false;

uint32_t mfrc630_twi_init(void){	
  ret_code_t err_code;
  const nrf_drv_twi_config_t twi_mfrc630_config = {
	 .scl                = TWI_SCL,               
	 .sda                = TWI_SDA,
	 .frequency          = NRF_TWI_FREQ_400K,    
	 .interrupt_priority = APP_IRQ_PRIORITY_HIGH,  
	 .clear_bus_init     = false
  };

  err_code = nrf_drv_twi_init(&m_twi, 
							  &twi_mfrc630_config,
							  NULL, 
							  NULL
  );  
  if(err_code != NRF_SUCCESS)
		return err_code;

  nrf_drv_twi_enable(&m_twi);
  return NRF_SUCCESS;
}

// ---------------------------------------------------------------------------
// Register interaction functions.
// ---------------------------------------------------------------------------
uint8_t mfrc630_read_reg(uint8_t reg) {
  uint32_t err_code;
  uint8_t data;
  nrf_drv_twi_enable(&m_twi);
  uint8_t instruction_tx[1] = {reg};	
  err_code = nrf_drv_twi_tx(&m_twi,
                            MFRC630_ADDRESS, 
                            instruction_tx, 
                            1, 
                            true
  );

  if(err_code != 0)
    NRF_LOG_ERROR("MFRC630: read: %02x error %d\n", reg, err_code);

  err_code = nrf_drv_twi_rx(&m_twi, 
                            MFRC630_ADDRESS,
                            &data, 
                            2
  );
  
  if(err_code != 0)
    NRF_LOG_ERROR("MFRC630: read: %02x error %d\n", reg, err_code);
  nrf_drv_twi_enable(&m_twi);
  return data ; 
}

void mfrc630_write_reg(uint8_t reg, uint8_t value) {
  uint32_t err_code;
  uint8_t instruction_tx[2] = {reg , value};	
  nrf_drv_twi_enable(&m_twi);
  err_code = nrf_drv_twi_tx(&m_twi,
                            MFRC630_ADDRESS,
                            instruction_tx,
                            2, 
                            false
  );
							
  if(err_code != 0)
    NRF_LOG_ERROR("MFRC630: write: %02x error %d\n", reg, err_code);
    nrf_drv_twi_enable(&m_twi);
}

void mfrc630_write_regs(uint8_t reg, const uint8_t* values, uint8_t len) {
  uint8_t i;
  for (i=0 ; i < len; i++) {
  mfrc630_write_reg(reg+i,values[i]);  
  }
}

void mfrc630_write_fifo(const uint8_t* data, uint16_t len) {

  uint8_t i;
  for (i=0 ; i < len ; i++) {
		mfrc630_write_reg(MFRC630_REG_FIFODATA,data[i]); 
  }
}

void mfrc630_read_fifo(uint8_t* rx, uint16_t len) {	
  uint32_t err_code;
  nrf_drv_twi_enable(&m_twi);
  uint8_t instruction_tx[1] = {MFRC630_REG_FIFODATA};	
  err_code = nrf_drv_twi_tx(&m_twi,
                            MFRC630_ADDRESS, 
                            instruction_tx, 
                            1, 
                            true
  );

  if(err_code != 0)
    NRF_LOG_ERROR("MFRC630: read: %02x error %d\n", MFRC630_REG_FIFODATA, err_code);
	
  err_code = nrf_drv_twi_rx(&m_twi, 
                            MFRC630_ADDRESS,
                            rx, 
                            len
  );
  
  if(err_code != 0)
    NRF_LOG_ERROR("MFRC630: read: %02x error %d\n", MFRC630_REG_FIFODATA, err_code);
  nrf_drv_twi_enable(&m_twi);	
}
	
// ---------------------------------------------------------------------------
// Command functions.
// ---------------------------------------------------------------------------
void mfrc630_cmd_load_protocol(uint8_t rx, uint8_t tx) {
  uint8_t parameters[2] = {rx, tx};
  mfrc630_flush_fifo();
  mfrc630_write_fifo(parameters, 2);
  mfrc630_write_reg(MFRC630_REG_COMMAND, MFRC630_CMD_LOADPROTOCOL);
}

void mfrc630_cmd_transceive(const uint8_t* data, uint16_t len) {
  mfrc630_cmd_idle();
  mfrc630_flush_fifo();
  mfrc630_write_fifo(data, len);
  mfrc630_write_reg(MFRC630_REG_COMMAND, MFRC630_CMD_TRANSCEIVE);
}

void mfrc630_cmd_idle() {
  mfrc630_write_reg(MFRC630_REG_COMMAND, MFRC630_CMD_IDLE);
}

// ---------------------------------------------------------------------------
// Utility functions.
// ---------------------------------------------------------------------------
void mfrc630_flush_fifo(void) {
  mfrc630_write_reg(MFRC630_REG_FIFOCONTROL, 1<<4);
}

void mfrc630_clear_irq0(void) {
  mfrc630_write_reg(MFRC630_REG_IRQ0, (uint8_t) ~(1<<7));
}

void mfrc630_clear_irq1(void) {
  mfrc630_write_reg(MFRC630_REG_IRQ1, (uint8_t) ~(1<<7));
}

uint8_t mfrc630_irq0(void) {
  return mfrc630_read_reg(MFRC630_REG_IRQ0);
}

uint8_t mfrc630_irq1(void) {
  return mfrc630_read_reg(MFRC630_REG_IRQ1);
}

void mfrc630_print_block(const uint8_t* data, uint16_t len) {
  uint16_t i;
  for (i=0; i < len; i++) {
		printf("%02X ", data[i]);
  }
}

// ---------------------------------------------------------------------------
// Timer functions
// ---------------------------------------------------------------------------
void mfrc630_timer_set_control(uint8_t timer, uint8_t value) {
  mfrc630_write_reg(MFRC630_REG_T0CONTROL + (5 * timer), value);
}

void mfrc630_timer_set_reload(uint8_t timer, uint16_t value) {
  mfrc630_write_reg(MFRC630_REG_T0RELOADHI + (5 * timer), value >> 8);
  mfrc630_write_reg(MFRC630_REG_T0RELOADLO + (5 * timer), 0xFF);
}

void mfrc630_timer_set_value(uint8_t timer, uint16_t value) {
  mfrc630_write_reg(MFRC630_REG_T0COUNTERVALHI + (5 * timer), value >> 8);
  mfrc630_write_reg(MFRC630_REG_T0COUNTERVALLO + (5 * timer), 0xFF);
}

uint16_t mfrc630_timer_get_value(uint8_t timer) {
  uint16_t res = mfrc630_read_reg(MFRC630_REG_T0COUNTERVALHI + (5 * timer)) << 8;
  res += mfrc630_read_reg(MFRC630_REG_T0COUNTERVALLO + (5 * timer));
  return res;
}

// ---------------------------------------------------------------------------
// From documentation
// ---------------------------------------------------------------------------
void mfrc630_AN1102_recommended_registers_skip(uint8_t protocol, uint8_t skip) {
  switch (protocol) {
    case MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER:
      {
        const uint8_t buf[] = MFRC630_RECOM_14443A_ID1_106;
        mfrc630_write_regs(MFRC630_REG_DRVMOD+skip, buf+skip, sizeof(buf)-skip);
      }
      break;
    case MFRC630_PROTO_ISO14443A_212_MILLER_BPSK:
      {
        const uint8_t buf[] = MFRC630_RECOM_14443A_ID1_212;
        mfrc630_write_regs(MFRC630_REG_DRVMOD+skip, buf+skip, sizeof(buf)-skip);
      }
      break;
    case MFRC630_PROTO_ISO14443A_424_MILLER_BPSK:
      {
        const uint8_t buf[] = MFRC630_RECOM_14443A_ID1_424;
        mfrc630_write_regs(MFRC630_REG_DRVMOD+skip, buf+skip, sizeof(buf)-skip);
      }
      break;
    case MFRC630_PROTO_ISO14443A_848_MILLER_BPSK:
      {
        const uint8_t buf[] = MFRC630_RECOM_14443A_ID1_848;
        mfrc630_write_regs(MFRC630_REG_DRVMOD+skip, buf+skip, sizeof(buf)-skip);
      }
      break;
  }
}

void mfrc630_AN1102_recommended_registers(uint8_t protocol) {
  mfrc630_AN1102_recommended_registers_skip(protocol, 0);
}

// ---------------------------------------------------------------------------
// ISO 14443A
// ---------------------------------------------------------------------------
uint16_t mfrc630_iso14443a_REQA(void) {
  return mfrc630_iso14443a_WUPA_REQA(MFRC630_ISO14443_CMD_REQA);
}

uint16_t mfrc630_iso14443a_WUPA_REQA(uint8_t instruction) {
  mfrc630_cmd_idle();
  mfrc630_flush_fifo();
//  Set register such that we sent 7 bits, set DataEn such that we can send data.	
  mfrc630_write_reg(MFRC630_REG_TXDATANUM, 7 | MFRC630_TXDATANUM_DATAEN);
//  disable the CRC registers.	
  mfrc630_write_reg(MFRC630_REG_TXCRCPRESET, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_OFF);
  mfrc630_write_reg(MFRC630_REG_RXCRCCON, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_OFF);
  mfrc630_write_reg(MFRC630_REG_RXBITCTRL, 0);	
// ready the request.	
  uint8_t send_req[] = {instruction};
// clear interrupts  
  mfrc630_clear_irq0();
  mfrc630_clear_irq1();
// enable the global IRQ for Rx done and Errors.  
  mfrc630_write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_RX_IRQEN | MFRC630_IRQ0EN_ERR_IRQEN);
  mfrc630_write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_TIMER0_IRQEN); 
// configure a timeout timer.  
  uint8_t timer_for_timeout = 0;
// Set timer to 221 kHz clock, start at the end of Tx.
  mfrc630_timer_set_control(timer_for_timeout, MFRC630_TCONTROL_CLK_211KHZ | MFRC630_TCONTROL_START_TX_END );     
// Frame waiting time: FWT = (256 x 16/fc) x 2 FWI
// FWI defaults to four... so that would mean wait for a maximum of ~ 5ms
  mfrc630_timer_set_reload(timer_for_timeout, 1000);    // 1000 ticks of 5 usec is 5 ms.
  mfrc630_timer_set_value(timer_for_timeout, 1000);
// Go into send, then straight after in receive.  
  mfrc630_cmd_transceive(send_req, 1);    //0026 写入fifo
  NRF_LOG_INFO("Sending REQA\r\n");
	
  // block until we are done
  uint8_t irq1_value = 0;  
  while (!(irq1_value & (1 << timer_for_timeout))) 
  {
    irq1_value = mfrc630_irq1();
    if (irq1_value & MFRC630_IRQ1_GLOBAL_IRQ) {  // either ERR_IRQ or RX_IRQ
			NRF_LOG_INFO("either ERR_IRQ or RX_IRQ\r\n");		
      break;  // stop polling irq1 and quit the timeout loop.
    }
  }
  NRF_LOG_INFO("After waiting for answer\r\n");
  mfrc630_cmd_idle();
  
// if no Rx IRQ, or if there's an error somehow, return 0  
  uint8_t irq0 = 0;
	irq0 = mfrc630_irq0();
  if ((!(irq0 & MFRC630_IRQ0_RX_IRQ)) || (irq0 & MFRC630_IRQ0_ERR_IRQ)) {
		NRF_LOG_INFO("No RX, irq1: %hhx irq0: %hhx\r\n", irq1_value, irq0);
    return 0;
  }
  
  uint16_t res;  
  mfrc630_read_fifo((uint8_t*) &res, 2);   // ATQA should answer with 2 bytes.
	NRF_LOG_INFO("ATQA answer: ");
	mfrc630_print_block((uint8_t*) &res, 2);
	NRF_LOG_INFO("\r\n");
	NRF_LOG_INFO("res = %d\r\n", res);  
  return res;  
}

uint8_t mfrc630_iso14443a_select( uint8_t* uid, uint8_t* sak){
  mfrc630_cmd_idle();
  mfrc630_flush_fifo(); //清空 FIFOdata	
// enable the global IRQ for Rx done and Errors.	
	NRF_LOG_INFO("Starting select\r\n");
  mfrc630_write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_RX_IRQEN | MFRC630_IRQ0EN_ERR_IRQEN);
  mfrc630_write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_TIMER0_IRQEN);  // only trigger on timer for irq1
// configure a timeout timer, use timer 0.	
  uint8_t timer_for_timeout = 0;
// Set timer to 221 kHz clock, start at the end of Tx.	
  mfrc630_timer_set_control(timer_for_timeout, MFRC630_TCONTROL_CLK_211KHZ | MFRC630_TCONTROL_START_TX_END);
// Frame waiting time: FWT = (256 x 16/fc) x 2 FWI
// FWI defaults to four... so that would mean wait for a maximum of ~ 5ms	
  mfrc630_timer_set_reload(timer_for_timeout, 1000);  
  mfrc630_timer_set_value(timer_for_timeout, 1000);
  uint8_t cascade_level = 1;
	
  for (cascade_level=1; cascade_level <= 3; cascade_level++) { //防碰撞的过程分成三个级别		
		uint8_t cmd = 0;
		uint8_t known_bits = 0;       // known bits of the UID at this level so far.
		uint8_t send_req[7] = {0};    // used as Tx buffer.
		uint8_t* uid_this_level = &(send_req[2]);
		uint8_t message_length;
	// pointer to the UID so far, by placing this pointer in the send_req
	// array we prevent copying the UID con

		switch (cascade_level) {
			case 1:
				cmd = MFRC630_ISO14443_CAS_LEVEL_1;
				break;
			case 2:
				cmd = MFRC630_ISO14443_CAS_LEVEL_2;
				break;
			case 3:
				cmd = MFRC630_ISO14443_CAS_LEVEL_3;
				break;
		}	

	// disable CRC in anticipation of the anti collision protocol
		mfrc630_write_reg(MFRC630_REG_TXCRCPRESET, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_OFF);
		mfrc630_write_reg(MFRC630_REG_RXCRCCON, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_OFF);

	// max 32 loops of the collision loop.
		uint8_t collision_n;
		uint8_t buf[5];  
		for (collision_n=0; collision_n < 32; collision_n++) {
			NRF_LOG_INFO("CL: %hhd, coll loop: %hhd, kb %hhd long: \r\n", cascade_level, collision_n, known_bits);
			mfrc630_clear_irq0();
			mfrc630_clear_irq1();
			send_req[0] = cmd;
			send_req[1] = 0x20 + known_bits;
			mfrc630_write_reg(MFRC630_REG_TXDATANUM, (known_bits % 8) | MFRC630_TXDATANUM_DATAEN);
			uint8_t rxalign = known_bits % 8;
			NRF_LOG_INFO("Setting rx align to: %hhd\r\n", rxalign);

			mfrc630_write_reg(MFRC630_REG_RXBITCTRL, (0<<7) | (rxalign<<4));

			if ((known_bits % 8) == 0) {
				message_length = ((known_bits / 8)) + 2;
			} else {
				message_length = ((known_bits / 8) + 1) + 2;
			}
			
		  NRF_LOG_INFO("Send:%hhd long: ", message_length);
		  mfrc630_print_block(send_req, message_length);
		  NRF_LOG_INFO("\r\n");
			mfrc630_cmd_transceive(send_req, message_length);	  
	// block until we are done	  
			uint8_t irq1_value = 0;
			while (!(irq1_value & (1 << timer_for_timeout))) {
				irq1_value = mfrc630_irq1();
				if (irq1_value & MFRC630_IRQ1_GLOBAL_IRQ) { // either ERR_IRQ or RX_IRQ or Timer
					break;  // stop polling irq1 and quit the timeout loop.
				}
			}

			mfrc630_cmd_idle();
	// next up, we have to check what happened.	  
			uint8_t irq0 = mfrc630_irq0();
			uint8_t error = mfrc630_read_reg(MFRC630_REG_ERROR);
			uint8_t coll = mfrc630_read_reg(MFRC630_REG_RXCOLL);
			NRF_LOG_INFO("irq0: %hhX\r\n", irq0);
			NRF_LOG_INFO("error: %hhX\r\n", error);
			NRF_LOG_INFO("coll: %hhX\r\n", coll);
			
			uint8_t collision_pos = 0;
			if (irq0 & MFRC630_IRQ0_ERR_IRQ) {  // some error occured.
	// Check what kind of error.
	// error = mfrc630_read_reg(MFRC630_REG_ERROR);		  
				if (error & MFRC630_ERROR_COLLDET) {
					if (coll & (1<<7)) {
						collision_pos = coll & (~(1<<7));
						NRF_LOG_INFO("Collision at %hhX\r\n", collision_pos);
						uint8_t choice_pos = known_bits + collision_pos;
						uint8_t selection = (uid[((choice_pos + (cascade_level-1)*3)/8)] >> ((choice_pos) % 8))&1;

						uid_this_level[((choice_pos)/8)] |= selection << ((choice_pos) % 8);
						known_bits++;  
						NRF_LOG_INFO("uid_this_level now kb %hhd long: ", known_bits);
						mfrc630_print_block(uid_this_level, 10);
						NRF_LOG_INFO("\r\n");
					} else {
						NRF_LOG_INFO("Collision but no valid collpos.\r\n");
						collision_pos = 0x20 - known_bits;
					}
				} else {
					NRF_LOG_INFO("No collision, error was: %hhx, setting collision_pos to: %hhx\r\n", error, collision_pos);
					collision_pos = 0x20 - known_bits;
				}
			} else if (irq0 & MFRC630_IRQ0_RX_IRQ) {
				collision_pos = 0x20 - known_bits;
				NRF_LOG_INFO("Got data, no collision, setting to: %hhx\r\n", collision_pos);				
			} else {
				return 0;
			}
			NRF_LOG_INFO("collision_pos: %hhX\r\n", collision_pos);

			mfrc630_read_fifo(buf,5);
			
			  NRF_LOG_INFO("Fifo %hhd long: ", 5);
			  mfrc630_print_block(buf, 5);
			  NRF_LOG_INFO("\r\n");
			
			uint8_t rbx;
			NRF_LOG_INFO("uid_this_level kb %hhd long: ", known_bits);
			mfrc630_print_block(uid_this_level, (known_bits + 8 - 1) / 8);
			NRF_LOG_INFO("\r\n");
			for (rbx = 0; (rbx < 5); rbx++) {
				uid_this_level[(known_bits / 8) + rbx] |= buf[rbx];
			}
			known_bits += collision_pos;
			NRF_LOG_INFO("known_bits: %hhX\r\n", known_bits);
		
			if ((known_bits >= 32)) {
				NRF_LOG_INFO("exit collision loop\r\n");
				NRF_LOG_INFO("uid_this_level kb %hhd long: ", known_bits);				
				mfrc630_print_block(uid_this_level, 10);		
				NRF_LOG_INFO("\r\n");				
				break; 
			}
		}  
	// check if the BCC matches
//		uint8_t bcc_val = uid_this_level[4];  
		
		uint8_t bcc_val = buf[4];  
		uint8_t bcc_calc = uid_this_level[0]^uid_this_level[1]^uid_this_level[2]^uid_this_level[3];

		if (bcc_val != bcc_calc) {
			memcpy(uid, buf, sizeof(buf));
			NRF_LOG_INFO("Something went wrong, BCC does not match[%02x:%02x].\r\n", bcc_val, bcc_calc);
			return 0;
		}

		mfrc630_clear_irq0();
		mfrc630_clear_irq1();
		send_req[0] = cmd;
		send_req[1] = 0x70;
		send_req[6] = bcc_calc;
		message_length = 7;

		mfrc630_write_reg(MFRC630_REG_TXCRCPRESET, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_ON);
		mfrc630_write_reg(MFRC630_REG_RXCRCCON, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_ON);
		mfrc630_write_reg(MFRC630_REG_TXDATANUM, (known_bits % 8) | MFRC630_TXDATANUM_DATAEN);
		uint8_t rxalign = 0;
		mfrc630_write_reg(MFRC630_REG_RXBITCTRL, (0 << 7) | (rxalign << 4));
		mfrc630_cmd_transceive(send_req, message_length);
		NRF_LOG_INFO("send_req %hhd long: ", message_length);
		mfrc630_print_block(send_req, message_length);
		NRF_LOG_INFO("\r\n");				
		
		uint8_t irq1_value = 0;
		while (!(irq1_value & (1 << timer_for_timeout))) {
			irq1_value = mfrc630_irq1();
			if (irq1_value & MFRC630_IRQ1_GLOBAL_IRQ) { 
				break; 
			}
		}
		mfrc630_cmd_idle();
		uint8_t irq0_value = mfrc630_irq0();
		if (irq0_value & MFRC630_IRQ0_ERR_IRQ) {
			uint8_t error = mfrc630_read_reg(MFRC630_REG_ERROR);
			if (error & MFRC630_ERROR_COLLDET) {
				return 0;
			}
		}

		uint8_t sak_len = 1;
		mfrc630_read_fifo(sak, sak_len);	
		NRF_LOG_INFO("SAK answer: ");
		mfrc630_print_block(sak, 1);
		NRF_LOG_INFO("\r\n");				

		if (*sak & (1 << 2)) {
			uint8_t UIDn;
			for (UIDn=0; UIDn < 3; UIDn++) {
				uid[(cascade_level-1)*3 + UIDn] = uid_this_level[UIDn + 1];
			}
		} else {
			uint8_t UIDn;
			for (UIDn=0; UIDn < 4; UIDn++) {
				uid[(cascade_level-1)*3 + UIDn] = uid_this_level[UIDn];
			}
			return cascade_level*3 + 1;
		}
    NRF_LOG_INFO("Exit cascade %hhd long: \r\n", cascade_level);
    mfrc630_print_block(uid, 10);		
	NRF_LOG_INFO("\r\n");
	}  
	return 0; 
}


