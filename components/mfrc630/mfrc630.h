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

#ifndef MFRC630_H_
#define MFRC630_H_
#include <stdint.h>
#include "mfrc630_def.h"
#include "pca10040.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include <stdint.h>
#include <string.h>
#include "ble_nus.h"

#define MFRC630_ADDRESS 0x29	

uint32_t mfrc630_twi_init(void);

/**@brief Reads a register.
 *
 * @param[in] reg Specifies which register to read.
 * @return the value of the register to be read.
 */
uint8_t mfrc630_read_reg(uint8_t reg);

/**@brief Write a register.
 *
 * @details Sets a single register to the provided value.
 *
 * @param[in] reg Specifies which register to write.
 * @param[in] value Specifies the value to write to that register.
 */
void mfrc630_write_reg(uint8_t reg, uint8_t value);

/**@brief Write multiple registers.
 *
 * @details Sets consecutive registers to the provided values.
 *
 * @param[in] reg Specifies at which register writing starts.
 * @param[in] values An array of the values to write to the register starting from `reg`.
			  The first value (`values[0]`) gets written to `reg`, the second (`values[1]`) to `reg+1`, and so on.
 * @param[in] len The number of register to write.
 */
void mfrc630_write_regs(uint8_t reg, const uint8_t* values, uint8_t len);

/**@brief  Write data to FIFO.
 *
 * @details   The FIFO is located at register `#MFRC630_REG_FIFODATA`. 
 *			  Writes to this register do not automatically increment the write pointer in the chip and multiple bytes may be written
 *            to this register to place them into the FIFO buffer.
 *			  This function does not clear the FIFO beforehand, it only provides the raw transfer functionality.
 * @param[in] data The data to be written into the FIFO.
 * @param[in] len The number of bytes to be written into the FIFO.
 */
void mfrc630_write_fifo(const uint8_t* data, uint16_t len);
 
/**@brief  Read data from FIFO.
 *
 * @details   This function reads data from the FIFO into an array on the microcontroller.
 *
 * @param[out] rx The data read from the FIFO is placed into this array.
 * @param[in] len The number of bytes to be read from the FIFO.
 *
 * @warning  This reads regardless of `#MFRC630_REG_FIFOLENGTH`, if there aren't enough bytes present in the FIFO, they
 *         are read from the chip anyway, these bytes should not be used. (The returned bytes from an empty FIFO are
 *        often identical to the last valid byte that was read from it.)
 */ 
void mfrc630_read_fifo(uint8_t* rx, uint16_t len);

/**@brief  Read data from EEPROM into the FIFO buffer.
 *
 * @details   This instruction transfers data from the EEPROM (section 2) at the given address locaction into the FIFO buffer.
 *
 * @param[in] address The start address in the EEPROM to start reading from.
 * @param[in] length The number of bytes to read from the EEPROM into the FIFO buffer.
 */
void mfrc630_cmd_read_E2(uint16_t address, uint16_t length);

/**@brief  Load protocol settings.
 *
 * @details     Loads register settings for the protocol indicated. Can configure different protocols for rx and tx. The most common
 *              protocol is `#MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER` which is the default protocol for the SELECT procedure.
 *              The most common protocols are listed in the datasheet, but the MFRC630 Quickstart Guide AN11022 gives a complete
 *              description.
 *
 * @param[in] rx The protocol number to load for the receiving frontend.
 * @param[in] tx The protocol number to load for the tranmitting frontend.
 */ 
void mfrc630_cmd_load_protocol(uint8_t rx, uint8_t tx);
 
/**@brief  Transmit the bytes provided and go into receive mode.
 *
 * @details  This function loads the data from the `data` array into the FIFO, and then issues the `MFRC630_CMD_TRANSCEIVE`
 *           command, which sends the data in the FIFO and switches to receiving mode afterwards.
 *
 * @param[in] data The data to be transmitted.
 * @param[in] len The numeber of bytes to be read from `data` and be transmitted..
 */ 
void mfrc630_cmd_transceive(const uint8_t* data, uint16_t len);

/**@brief  Set the device into idle mode.
 *
 * @details  Stops the currently active command and return to idle mode.
 */ 
void mfrc630_cmd_idle(void);

/**@brief  Loads a key from the EEPROM into the key buffer.
 *
 * @details    This function can load a key from the MIFARE key area in the EEPROM into the key buffer. This section of the EEPROM
 *             can only be written. The key buffer is a part of memory in the MFRC630 used for the MIFARE authentication procedure.
 *
 * @param[in] [in] key_nr Loads the key stored for this index.
 */ 
void mfrc630_cmd_load_key_E2(uint8_t key_nr);

/**@brief  Loads the provided key into the key buffer.
 *
 * @details  This function reads 6 bytes from the `key` array into the FIFO and then loads the key into the key buffer.
 *
 * @param[in] key Array which holds the MIFARE key, it is always 6 bytes long.
 */ 
void mfrc630_cmd_load_key(const uint8_t* key);

/**@brief Perform MIFARE authentication procedure with a card.
 *
 * @details   This function attemps to authenticate with the specified card using the key which is currently in the key buffer.
 *            This function is usually preceded by either the `mfrc630_cmd_load_key_E2()` or `mfrc630_cmd_load_key()` functions.
 *
 * @param[in] key_type The MIFARE key A or B (`MFRC630_MF_AUTH_KEY_A` = 0x60 or `MFRC630_MF_AUTH_KEY_B` = 0x61) to use.
 * @param[in] block_address The block on which to authenticate.
 * @param[in] card_uid The authentication procedure required the first four bytes of the card's UID to authenticate.
 */ 
void mfrc630_cmd_auth(uint8_t key_type, uint8_t block_address, const uint8_t* card_uid);

/**@brief  Flush the FIFO buffer.
 *
 * @details This function clears all contents that are currently in the FIFO.
 */ 
void mfrc630_flush_fifo(void);

/**@brief Get the FIFO length.
 *
 * @details  Returns the current number of bytes in the FIFO.
 *
 * @warning This function only returns the first 8 bits of the FIFO length, if the 512 byte FIFO is used, only the least
 *          significant eight bits will be returned.
 *
 * @return The number of bytes currently in the FIFO.
 */ 
uint16_t mfrc630_fifo_length(void);

/**@brief  Clear the interrupt0 flags.
 *
 * @details Resets the interrupt 0 register (`MFRC630_REG_IRQ0`).
 */ 
void mfrc630_clear_irq0(void);

/**@brief  Clear the interrupt1 flags.
 *
 * @details Resets the interrupt 1 register (`MFRC630_REG_IRQ1`).
 */ 
void mfrc630_clear_irq1(void);

/**@brief  Get the value of the interrupt 0 register.
 *
 * @return The value of the `MFRC630_REG_IRQ0` register.
 */ 
uint8_t mfrc630_irq0(void);

/**@brief  Get the value of the interrupt 1 register.
 *
 * @return The value of the `MFRC630_REG_IRQ1` register.
 */ 
uint8_t mfrc630_irq1(void);

/**@brief Print an array in hexadecimal format using `MFRC630_PRINTF`.
 *
 * @details     Prints the bytes in `data` in hexadecimal format, separated by spaces using the `MFRC630_PRINTF` macro, if defined.
 *
 * @param[in] data The array to be printed.
 * @param[in] len The number of bytes to print..
 */ 
void mfrc630_print_block(const uint8_t* data, uint16_t len);

/**@brief Copy a page from EEPROM into an array on the MCU.
 *
 * @details     This instruction transfers a page from the EEPROM into the FIFO and then transfers this data from the FIFO
 *              into an array. It always transfers 64 bytes, as such `dest` must be (atleast) 64 bytes long.
 *              This basically calls mfrc630_cmd_read_E2() and then transfers the FIFO with mfrc630_read_fifo(). This is useful for
 *              dumping the entire EEPROM.
 *
 * @param[out] dest The array to write the data into.
 * @param[in] page The page to read from the EEPROM. (This gets multiplied by 64 to obtain the start address).
 * @return The number of bytes transmitted from the FIFO into `dest`.
 */ 
uint8_t mfrc630_transfer_E2_page(uint8_t* dest, uint8_t page);

/**@brief Activates a timer.
 *
 * @details     This sets the the `MFRC630_REG_TCONTROL` register to enable or disable this timer.
 *
 * @param[in] timer Specifies which timer to use (0, 1, 2 or 3).
 * @param[in] active Should be `0` to deactivate the timer, `1` to activate it.
 */  
void mfrc630_activate_timer(uint8_t timer, uint8_t active);

/**@brief  Sets the timer control register.
 *
 * @details  This sets the `T[0-3]Control` register to the provided value. The value speficies the propertief of StopRx, Start
 *           AutoRestart and Clk for this timer.
 *
 * @param[in] timer Specifies which timer to use (0, 1, 2 or 3).
 * @param[in] value This can be a combination of the defines associated with the Timer controls.
 */
void mfrc630_timer_set_control(uint8_t timer, uint8_t value);

/**@brief  Sets the reload value of the timer.
 *
 * @details  This counter starts counting down from this reload value, an underflow occurs when the timer reaches zero.
 *
 * @param[in] timer Specifies which timer to use (0, 1, 2 or 3).
 * @param[in] value The value from which to start the counter. 
 */
void mfrc630_timer_set_reload(uint8_t timer, uint16_t value);

/**@brief  Sets the current value of this timer..
 *
 * @details  Sets the current value of this counter, it counts down from this given value.
 *
 * @param[in] timer Specifies which timer to use (0, 1, 2 or 3).
 * @param[in] value The value to set the counter to. 
 */
void mfrc630_timer_set_value(uint8_t timer, uint16_t value);

/**@brief  Retrieve the current value of a timer.
 *
 * @details    Reads the current value of the given timer and returns the result.
 *
 * @param[in] timer Specifies which timer to use (0, 1, 2 or 3).
 * @return The current value of this timer.
 */
uint16_t mfrc630_timer_get_value(uint8_t timer);

/**@brief  Set the registers to the recommended values.
 *
 * @details  This function uses the recommended registers from the datasheets, it should yield identical results to the 
 *           `mfrc630_cmd_load_protocol()` function.
 *
 * @param[in] [in] protocol The protocol index to set the registers to. Only
 *            `MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER`, `MFRC630_PROTO_ISO14443A_212_MILLER_BPSK`,
 *            `MFRC630_PROTO_ISO14443A_424_MILLER_BPSK` and `MFRC630_PROTO_ISO14443A_848_MILLER_BPSK` were copied. The
 *            recommended values for the other protocols can be found in the application note.
 */
void mfrc630_AN1102_recommended_registers(uint8_t protocol);

/**@brief Set the registers to the recommended values, skipping the first `skip` registers.
 *
 * @details   Sets the recommended registers but allows for an arbitrary number of registers to be skipped at the start.
 *
 * @param[in] [in] protocol The protocol index to set the registers to. Only
 *            `MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER`, `MFRC630_PROTO_ISO14443A_212_MILLER_BPSK`,
 *            `MFRC630_PROTO_ISO14443A_424_MILLER_BPSK` and `MFRC630_PROTO_ISO14443A_848_MILLER_BPSK` were copied. The
 *            recommended values for the other protocols can be found in the application note.
 * @param[in] skip The number of registers to skip from the start.
 */
void mfrc630_AN1102_recommended_registers_skip(uint8_t protocol, uint8_t skip);

/**@brief Sends an Request Command, Type A.
 *
 * @details This sends the ISO14443 REQA request, cards in IDLE mode should answer to this.
 *
 * @return The Answer to request A byte (ATQA), or zero in case of no answer.
 */
uint16_t mfrc630_iso14443a_REQA(void);

/**@brief Sends an Wake-Up Command, Type A.
 *
 * @details This sends the ISO14443 WUPA request, cards in IDLE or HALT mode should answer to this.
 *
 * @return The Answer to request A byte (ATQA), or zero in case of no answer.
 */
uint16_t mfrc630_iso14443a_WUPA(void);

/**@brief Used to send WUPA and REQA.
 *
 * @details This actually sends WUPA and REQA and returns the response byte.
 *
 * @return The Answer to request A byte (ATQA), or zero in case of no answer.
 */
uint16_t mfrc630_iso14443a_WUPA_REQA(uint8_t instruction);

/*! \brief Performs the SELECT procedure to discover a card's UID.

  This performs the SELECT procedure as explained in ISO 14443A, this determines the UID of the card, if multiple cards
  are present, a collision will occur, which is handled according to the norm. This collision handling is explained
  quite complex in the norm, but conceptually it is not all that complex:

    - The cascade level can be seen as a prefix to ensure both the PICC and PCD are working on identifying the same
      part of the UID.
    - The entire anti-collision scheme is more of a binary search, the PICC sends the CASCADE level prefix, then the
      NVB byte, this field determines how many bits of the UID will follow, this allows the PICC's to listen to this
      and respond if their UID's match these first bits with the UID that is transmitted. After this all PICC's (that
      have matched the UID bits already sent) respond with the remainder of their UIDS. This results in either a
      complete UID, or in case two PICC's share a few bits but then differ a bit, a collision occurs on this bit. This
      collision is detected by the PCD, at which point it can chose either to pursue the PICC(s) that has a 0b1 at that
      position, or pursue the 0b0 at that position. The ISO norm states: A typical implementation adds a (1)b.
      I use the bit value that's in the pointer at the same position as the collision, or atleast for the first cascade
      level that works, after that it's off by a byte because of the cascade tag, see the actual implementation.

  \param [out] uid: The UID of the card will be stored into this array. This array is also used to determine the choice
                    between an 0b1 or 0b0 when a collision occurs. The bit that's in `uid` at the collision position is
                    selected.
  \param [out] sak: The last SAK byte received during the SELECT procedure is placed here, this often holds information
                    about the type of card.
  \return The length of the UID in bytes (4, 7, 10), or 0 in case of failure.
*/

/**@brief Performs the SELECT procedure to discover a card's UID.
 *
 * @details  This performs the SELECT procedure as explained in ISO 14443A, this determines the UID of the card, if multiple cards
 *           are present, a collision will occur, which is handled according to the norm. This collision handling is explained
 *           quite complex in the norm, but conceptually it is not all that complex:
 *
 *           - The cascade level can be seen as a prefix to ensure both the PICC and PCD are working on identifying the same
 *             part of the UID.
 *           - The entire anti-collision scheme is more of a binary search, the PICC sends the CASCADE level prefix, then the
 *             NVB byte, this field determines how many bits of the UID will follow, this allows the PICC's to listen to this
 *             and respond if their UID's match these first bits with the UID that is transmitted. After this all PICC's (that
 *             have matched the UID bits already sent) respond with the remainder of their UIDS. This results in either a
 *             complete UID, or in case two PICC's share a few bits but then differ a bit, a collision occurs on this bit. This
 *             collision is detected by the PCD, at which point it can chose either to pursue the PICC(s) that has a 0b1 at that
 *             position, or pursue the 0b0 at that position. The ISO norm states: A typical implementation adds a (1)b.
 *             I use the bit value that's in the pointer at the same position as the collision, or atleast for the first cascade
 *             level that works, after that it's off by a byte because of the cascade tag, see the actual implementation.
 *
 * @param [out] uid: The UID of the card will be stored into this array. This array is also used to determine the choice
 *                   between an 0b1 or 0b0 when a collision occurs. The bit that's in `uid` at the collision position is
 *                   selected.
 * @param [out] sak: The last SAK byte received during the SELECT procedure is placed here, this often holds information
 *                   about the type of card.
 * @return The length of the UID in bytes (4, 7, 10), or 0 in case of failure.
 */
uint8_t mfrc630_iso14443a_select( uint8_t* uid, uint8_t* sak);

//uint8_t mfrc630_MF_auth(const uint8_t* uid, uint8_t key_type, uint8_t block);

//void mfrc630_MF_deauth(void);

//uint8_t mfrc630_MF_read_block(uint8_t block_address, uint8_t* dest);

//uint8_t mfrc630_MF_write_block(uint8_t block_address, const uint8_t* source);

//void mfrc630_AN11145_start_IQ_measurement(void);

//void mfrc630_AN11145_stop_IQ_measurement(void);

//void mfrc630_AN1102_recommended_registers_no_transmitter(uint8_t protocol);

#endif 
