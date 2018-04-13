#include "mfrc630.h"
#include "nrf_drv_twi.h"
#include "twi_manager.h"
#include "macros_common.h"
#include "mfrc630_def.h"
#include "nrf_delay.h"


#define MPU_TWI_BUFFER_SIZE     	  14 

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);   
volatile static bool twi_tx_done = false;
volatile static bool twi_rx_done = false;
uint8_t twi_tx_buffer[MPU_TWI_BUFFER_SIZE];


uint32_t mfrc630_twi_init(void){	
  ret_code_t err_code;
  const nrf_drv_twi_config_t twi_mfrc630_config = {
	 .scl                = TWI_SCL,               
	 .sda                = TWI_SDA,
	 .frequency          = NRF_TWI_FREQ_400K,    
	 .interrupt_priority = APP_IRQ_PRIORITY_HIGH,  
	 .clear_bus_init     = false
  };

  err_code = nrf_drv_twi_init(&m_twi, &twi_mfrc630_config, NULL, NULL);  
  if(err_code != NRF_SUCCESS)
  {
	return err_code;
  }   

  nrf_drv_twi_enable(&m_twi);
  return NRF_SUCCESS;
}

uint8_t mfrc630_read_reg(uint8_t reg) {
  uint32_t err_code;
  uint8_t data;
  nrf_drv_twi_enable(&m_twi);
  uint8_t instruction_tx[1] = {reg};	
  err_code = nrf_drv_twi_tx(&m_twi, MFRC630_ADDRESS, instruction_tx, 1, true);

  if(err_code != 0)
     printf("err_code = %d\n",err_code) ;

  err_code = nrf_drv_twi_rx(&m_twi, MFRC630_ADDRESS ,&data, 2);
  if(err_code != 0)
     printf("err_code = %d\n",err_code) ;
  nrf_drv_twi_enable(&m_twi);
  return data ; 
}

void mfrc630_write_reg(uint8_t reg, uint8_t value) {
  uint32_t err_code;
 uint8_t instruction_tx[2] = {reg , value};	
  nrf_drv_twi_enable(&m_twi);
  err_code = nrf_drv_twi_tx(&m_twi, MFRC630_ADDRESS,instruction_tx, 2, false);
  if(err_code != 0)
  {
    printf("err_code = %d\n",err_code);
  }
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
  err_code = nrf_drv_twi_tx(&m_twi, MFRC630_ADDRESS, instruction_tx, 1, true);

  if(err_code != 0)
     printf("err_code = %d\n",err_code) ;

  err_code = nrf_drv_twi_rx(&m_twi, MFRC630_ADDRESS ,rx, len);
  if(err_code != 0)
     printf("err_code = %d\n",err_code) ;
  nrf_drv_twi_enable(&m_twi);	
}
	
void mfrc630_cmd_read_E2(uint16_t address, uint16_t length) {
  uint8_t parameters[3] = {(uint8_t) (address >> 8), (uint8_t) (address & 0xFF), length};
  mfrc630_flush_fifo();
  mfrc630_write_fifo(parameters, 3);
  mfrc630_write_reg(MFRC630_REG_COMMAND, MFRC630_CMD_READE2);
}

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

void mfrc630_cmd_load_key_E2(uint8_t key_nr) {
  uint8_t parameters[1] = {key_nr};
  mfrc630_flush_fifo();
  mfrc630_write_fifo(parameters, 1);
  mfrc630_write_reg(MFRC630_REG_COMMAND, MFRC630_CMD_LOADKEYE2);
}

void mfrc630_cmd_auth(uint8_t key_type, uint8_t block_address, const uint8_t* card_uid) {
  mfrc630_cmd_idle();
  uint8_t parameters[6] = {key_type, block_address, card_uid[0], card_uid[1], card_uid[2], card_uid[3]};
  mfrc630_flush_fifo();
  mfrc630_write_fifo(parameters, 6);
  mfrc630_write_reg(MFRC630_REG_COMMAND, MFRC630_CMD_MFAUTHENT);
}

void mfrc630_cmd_load_key(const uint8_t* key) {
  mfrc630_cmd_idle();
  mfrc630_flush_fifo();
  mfrc630_write_fifo(key, 6);
  mfrc630_write_reg(MFRC630_REG_COMMAND, MFRC630_CMD_LOADKEY);
}

void mfrc630_flush_fifo(void) {
  mfrc630_write_reg(MFRC630_REG_FIFOCONTROL, 1<<4);
}

uint16_t mfrc630_fifo_length(void) {
  return mfrc630_read_reg(MFRC630_REG_FIFOLENGTH);
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

uint8_t mfrc630_transfer_E2_page(uint8_t* dest, uint8_t page) {
  mfrc630_cmd_read_E2(page*64, 64);
  uint8_t res = mfrc630_fifo_length();
  mfrc630_read_fifo(dest, 64);
  return res;
}

void mfrc630_print_block(const uint8_t* data, uint16_t len) {
  uint16_t i;
  for (i=0; i < len; i++) {
  printf("%02X ", data[i]);
  }
}

void mfrc630_activate_timer(uint8_t timer, uint8_t active) {
  mfrc630_write_reg(MFRC630_REG_TCONTROL, ((active << timer) << 4) | (1 << timer));
}

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

void mfrc630_AN1102_recommended_registers_no_transmitter(uint8_t protocol) {
  mfrc630_AN1102_recommended_registers_skip(protocol, 5);
}

uint16_t mfrc630_iso14443a_REQA(void) {
  return mfrc630_iso14443a_WUPA_REQA(MFRC630_ISO14443_CMD_REQA);
}

uint16_t mfrc630_iso14443a_WUPA(void) {
  return mfrc630_iso14443a_WUPA_REQA(MFRC630_ISO14443_CMD_WUPA);
}

uint16_t mfrc630_iso14443a_WUPA_REQA(uint8_t instruction) {
  mfrc630_cmd_idle();
  mfrc630_flush_fifo();
  mfrc630_write_reg(MFRC630_REG_TXDATANUM, 7 | MFRC630_TXDATANUM_DATAEN);
  mfrc630_write_reg(MFRC630_REG_TXCRCPRESET, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_OFF);
  mfrc630_write_reg(MFRC630_REG_RXCRCCON, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_OFF);
  mfrc630_write_reg(MFRC630_REG_RXBITCTRL, 0);		
  uint8_t send_req[] = {instruction};
  mfrc630_clear_irq0();
  mfrc630_clear_irq1();
  mfrc630_write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_RX_IRQEN | MFRC630_IRQ0EN_ERR_IRQEN);
  mfrc630_write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_TIMER0_IRQEN); 
  uint8_t timer_for_timeout = 0;
  mfrc630_timer_set_control(timer_for_timeout, MFRC630_TCONTROL_CLK_211KHZ | MFRC630_TCONTROL_START_TX_END );                                            
  mfrc630_timer_set_reload(timer_for_timeout, 1000); 
  mfrc630_timer_set_value(timer_for_timeout, 1000);
  mfrc630_cmd_transceive(send_req, 1);    //0026 写入fifo
  printf("Sending REQA\n");
  
  uint8_t irq1_value = 0;  
  while (!(irq1_value & (1 << timer_for_timeout))) 
  {
    irq1_value = mfrc630_irq1();
    if (irq1_value & MFRC630_IRQ1_GLOBAL_IRQ) {  
      break;  
    }
  }
  printf("After waiting for answer\n");
  mfrc630_cmd_idle();
  
  uint8_t irq0 = mfrc630_irq0();
  if ((!(irq0 & MFRC630_IRQ0_RX_IRQ)) || (irq0 & MFRC630_IRQ0_ERR_IRQ)) {
	printf("No RX, irq1: %hhx irq0: %hhx\n", irq1_value, irq0);
    return 0;
  }
  printf("irq1: %hhx irq0: %hhx\n", irq1_value, irq0);
  
  
  uint16_t res;  
  mfrc630_read_fifo((uint8_t*) &res, 2);
  printf("ATQA answer: ");
  mfrc630_print_block((uint8_t*) &res, 2);
  printf("\n");

  return res;  
}

uint8_t mfrc630_iso14443a_select(uint8_t* uid, uint8_t* sak) {
  mfrc630_cmd_idle();
  mfrc630_flush_fifo(); //清空 FIFOdata
	
  printf("\nStarting select\n");
  mfrc630_write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_RX_IRQEN | MFRC630_IRQ0EN_ERR_IRQEN);
  mfrc630_write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_TIMER0_IRQEN);  
  uint8_t timer_for_timeout = 0;
  mfrc630_timer_set_control(timer_for_timeout, MFRC630_TCONTROL_CLK_211KHZ | MFRC630_TCONTROL_START_TX_END);
  mfrc630_timer_set_reload(timer_for_timeout, 1000);  
  mfrc630_timer_set_value(timer_for_timeout, 1000);
  uint8_t cascade_level;
  for (cascade_level=1; cascade_level <= 3; cascade_level++) {
    uint8_t cmd = 0;
    uint8_t known_bits = 0;  
    uint8_t send_req[7] = {0}; 
    uint8_t* uid_this_level = &(send_req[2]);
    uint8_t message_length;

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

    mfrc630_write_reg(MFRC630_REG_TXCRCPRESET, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_OFF);
    mfrc630_write_reg(MFRC630_REG_RXCRCCON, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_OFF);

    uint8_t collision_n;
	uint8_t buf[5];  
    for (collision_n=0; collision_n < 32; collision_n++) {
	  printf("\nCL: %hhd, coll loop: %hhd, kb %hhd long: ", cascade_level, collision_n, known_bits);
      mfrc630_print_block(uid_this_level, (known_bits + 8 - 1) / 8);
	  printf("\n");

      mfrc630_clear_irq0();
      mfrc630_clear_irq1();

      send_req[0] = cmd;
      send_req[1] = 0x20 + known_bits;

      mfrc630_write_reg(MFRC630_REG_TXDATANUM, (known_bits % 8) | MFRC630_TXDATANUM_DATAEN);

      uint8_t rxalign = known_bits % 8;
	  printf("Setting rx align to: %hhd\n", rxalign);
      mfrc630_write_reg(MFRC630_REG_RXBITCTRL, (0<<7) | (rxalign<<4));

      if ((known_bits % 8) == 0) {
        message_length = ((known_bits / 8)) + 2;
      } else {
        message_length = ((known_bits / 8) + 1) + 2;
      }

	  printf("Send:%hhd long: ", message_length);
      mfrc630_print_block(send_req, message_length);
	  printf("\n");	  

      mfrc630_cmd_transceive(send_req, message_length);
	  
      uint8_t irq1_value = 0;
      while (!(irq1_value & (1 << timer_for_timeout))) {
        irq1_value = mfrc630_irq1();
        if (irq1_value & MFRC630_IRQ1_GLOBAL_IRQ) {
          break;  
        }
      }

      mfrc630_cmd_idle();
	  
      uint8_t irq0 = mfrc630_irq0();
      uint8_t error = mfrc630_read_reg(MFRC630_REG_ERROR);
      uint8_t coll = mfrc630_read_reg(MFRC630_REG_RXCOLL);

	  printf("irq0: %hhX\n", irq0);
	  printf("error: %hhX\n", error);
	  

	  
      uint8_t collision_pos = 0;
      if (irq0 & MFRC630_IRQ0_ERR_IRQ) {  
        if (error & MFRC630_ERROR_COLLDET) {
          if (coll & (1<<7)) {
            collision_pos = coll & (~(1<<7));
			printf("Collision at %hhX\n", collision_pos);  

            uint8_t choice_pos = known_bits + collision_pos;
            uint8_t selection = (uid[((choice_pos + (cascade_level-1)*3)/8)] >> ((choice_pos) % 8))&1;

            uid_this_level[((choice_pos)/8)] |= selection << ((choice_pos) % 8);
            known_bits++;  
			printf("uid_this_level now kb %hhd long: ", known_bits);
            mfrc630_print_block(uid_this_level, 10);
			printf("\n");
          } else {
			printf("Collision but no valid collpos.\n");  
            collision_pos = 0x20 - known_bits;
          }
        } else {
          collision_pos = 0x20 - known_bits;
		  printf("No collision, error was: %hhx, setting collision_pos to: %hhx\n", error, collision_pos);
        }
      } else if (irq0 & MFRC630_IRQ0_RX_IRQ) {
        collision_pos = 0x20 - known_bits;
		printf("Got data, no collision, setting  to: %hhx\n", collision_pos);  
  
      } else {
        return 0;
      }
	  printf("collision_pos: %hhX\n", collision_pos);


      mfrc630_read_fifo(buf,5);
	  printf("Fifo %hhd long: ", 5);
      mfrc630_print_block(buf, 5);
	  printf("\n");	  
	  	  
	  printf("uid_this_level kb %hhd long: ", known_bits);
      mfrc630_print_block(uid_this_level, (known_bits + 8 - 1) / 8);
	  printf("\n");
	
      uint8_t rbx;

//      for (rbx = 0; (rbx < rx_len); rbx++) {	  
      for (rbx = 0; (rbx < 5); rbx++) {
        uid_this_level[(known_bits / 8) + rbx] |= buf[rbx];
      }
      known_bits += collision_pos;
	  printf("known_bits: %hhX\n", known_bits);
	  
      if ((known_bits >= 32)) {
		printf("exit collision loop: uid_this_level kb %hhd long: ", known_bits);  
        mfrc630_print_block(uid_this_level, 10);
        printf("\n");
        break; 
      }
    }  

    uint8_t bcc_val = uid_this_level[4];  
    uint8_t bcc_calc = uid_this_level[0]^uid_this_level[1]^uid_this_level[2]^uid_this_level[3];
	
    if (bcc_val != bcc_calc) {
	  printf("Something went wrong, BCC does not match.\n");
	  memcpy(uid, buf, sizeof(buf));
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
	printf("send_req %hhd long: ", message_length);
    mfrc630_print_block(send_req, message_length);
	printf("\n");

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
//    uint8_t sak_len = mfrc630_fifo_length();
//    if (sak_len != 1) {
//      return 0;
//    }
    uint8_t sak_len = 1;
    uint8_t sak_value;
    mfrc630_read_fifo(&sak_value, sak_len);
	printf("SAK answer: ");
    mfrc630_print_block(&sak_value, 1);
    printf("\n");

    if (sak_value & (1 << 2)) {
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
	
    printf("Exit cascade %hhd long: ", cascade_level);
    mfrc630_print_block(uid, 10);
    printf("\n");
  }  
  return 0; 
}

uint8_t mfrc630_MF_auth(const uint8_t* uid, uint8_t key_type, uint8_t block) {

  uint8_t timer_for_timeout = 0; 

  mfrc630_write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_IDLE_IRQEN | MFRC630_IRQ0EN_ERR_IRQEN);
  mfrc630_write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_TIMER0_IRQEN); 
  mfrc630_timer_set_control(timer_for_timeout, MFRC630_TCONTROL_CLK_211KHZ | MFRC630_TCONTROL_START_TX_END);
  mfrc630_timer_set_reload(timer_for_timeout, 2000); 
  mfrc630_timer_set_value(timer_for_timeout, 2000);

  uint8_t irq1_value = 0;

  mfrc630_clear_irq0(); 
  mfrc630_clear_irq1(); 

  mfrc630_cmd_auth(key_type, block, uid);

  while (!(irq1_value & (1 << timer_for_timeout))) {
    irq1_value = mfrc630_irq1();
    if (irq1_value & MFRC630_IRQ1_GLOBAL_IRQ) {
      break; 
    }
  }

  if (irq1_value & (1 << timer_for_timeout)) {
    return 0; 
  }

  uint8_t status = mfrc630_read_reg(MFRC630_REG_STATUS);
  return (status & MFRC630_STATUS_CRYPTO1_ON);
}

uint8_t mfrc630_MF_read_block(uint8_t block_address, uint8_t* dest) {
  mfrc630_flush_fifo();

  mfrc630_write_reg(MFRC630_REG_TXCRCPRESET, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_ON);
  mfrc630_write_reg(MFRC630_REG_RXCRCCON, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_ON);

  uint8_t send_req[2] = {MFRC630_MF_CMD_READ, block_address};

  uint8_t timer_for_timeout = 0; 

  mfrc630_write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_IDLE_IRQEN | MFRC630_IRQ0EN_ERR_IRQEN);
  mfrc630_write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_TIMER0_IRQEN);
  mfrc630_timer_set_control(timer_for_timeout, MFRC630_TCONTROL_CLK_211KHZ | MFRC630_TCONTROL_START_TX_END);

  mfrc630_timer_set_reload(timer_for_timeout, 2000);  
  mfrc630_timer_set_value(timer_for_timeout, 2000);

  uint8_t irq1_value = 0;
  uint8_t irq0_value = 0;
  mfrc630_clear_irq0();  
  mfrc630_clear_irq1(); 

  mfrc630_cmd_transceive(send_req, 2);

  while (!(irq1_value & (1 << timer_for_timeout))) {
    irq1_value = mfrc630_irq1();
    if (irq1_value & MFRC630_IRQ1_GLOBAL_IRQ) {
      break;  
    }
  }
  mfrc630_cmd_idle();

  if (irq1_value & (1 << timer_for_timeout)) {
    return 0;
  }

  irq0_value = mfrc630_irq0();
  if (irq0_value & MFRC630_IRQ0_ERR_IRQ) {
    return 0;
  }

  uint8_t buffer_length = mfrc630_fifo_length();
  uint8_t rx_len = (buffer_length <= 16) ? buffer_length : 16;
  mfrc630_read_fifo(dest, rx_len);
  return rx_len;
}

uint8_t mfrc630_MF_write_block(uint8_t block_address, const uint8_t* source) {
  mfrc630_flush_fifo();

  mfrc630_write_reg(MFRC630_REG_TXCRCPRESET, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_ON);
  mfrc630_write_reg(MFRC630_REG_RXCRCCON, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_OFF);
  uint8_t timer_for_timeout = 0;

  mfrc630_write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_IDLE_IRQEN | MFRC630_IRQ0EN_ERR_IRQEN);
  mfrc630_write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_TIMER0_IRQEN);

  mfrc630_timer_set_control(timer_for_timeout, MFRC630_TCONTROL_CLK_211KHZ | MFRC630_TCONTROL_START_TX_END);
	
  mfrc630_timer_set_reload(timer_for_timeout, 2000); 
  mfrc630_timer_set_value(timer_for_timeout, 2000);

  uint8_t irq1_value = 0;
  uint8_t irq0_value = 0;

  uint8_t res;
  uint8_t send_req[2] = {MFRC630_MF_CMD_WRITE, block_address};

  mfrc630_clear_irq0();  
  mfrc630_clear_irq1();  

  mfrc630_cmd_transceive(send_req, 2);

  while (!(irq1_value & (1 << timer_for_timeout))) {
    irq1_value = mfrc630_irq1();
    if (irq1_value & MFRC630_IRQ1_GLOBAL_IRQ) {
      break; 
    }
  }
  mfrc630_cmd_idle();

  if (irq1_value & (1 << timer_for_timeout)) {
    return 0;
  }
  irq0_value = mfrc630_irq0();
  if (irq0_value & MFRC630_IRQ0_ERR_IRQ) {
    return 0;
  }
  uint8_t buffer_length = mfrc630_fifo_length();
  if (buffer_length != 1) {
    return 0;
  }
  mfrc630_read_fifo(&res, 1);
  if (res != MFRC630_MF_ACK) {
    return 0;
  }

  mfrc630_clear_irq0();  
  mfrc630_clear_irq1();  

  mfrc630_cmd_transceive(source, 16);

  while (!(irq1_value & (1 << timer_for_timeout))) {
    irq1_value = mfrc630_irq1();
    if (irq1_value & MFRC630_IRQ1_GLOBAL_IRQ) {
      break;  
    }
  }

  mfrc630_cmd_idle();

  if (irq1_value & (1 << timer_for_timeout)) {
    return 0;
  }
  irq0_value = mfrc630_irq0();
  if (irq0_value & MFRC630_IRQ0_ERR_IRQ) {
    return 0;
  }
  buffer_length = mfrc630_fifo_length();
  if (buffer_length != 1) {
    return 0;
  }
  mfrc630_read_fifo(&res, 1);
  if (res == MFRC630_MF_ACK) {
    return 16; 
  }
  return 0;
}

void mfrc630_MF_deauth(void) {
  mfrc630_write_reg(MFRC630_REG_STATUS, 0);
}

