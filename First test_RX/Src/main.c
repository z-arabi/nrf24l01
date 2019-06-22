
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//*** NRF24L01 pins and handles ***//
//CE and CSN pins
static GPIO_TypeDef		*nrf24_PORT;
static uint16_t				nrf24_CSN_PIN;
static uint16_t				nrf24_CE_PIN;
//SPI handle
static SPI_HandleTypeDef nrf24_hspi;
//Debugging UART handle
static UART_HandleTypeDef nrf24_huart;

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define ifpositive(x) (((x)>0) ? 1:0)

//*** Library variables ***//
static int ack_payload_available; /**< Whether there is an ack payload waiting */
static uint8_t ack_payload_length; /**< Dynamic size of pending ack payload. */
static uint8_t payload_size; /**< Fixed size of payloads */
static int dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */ 
static int p_variant; /* False for RF24L01 and true for RF24L01P */


uint64_t RxpipeAddrs = 0x11223344AA;
char myRxData[50];
/*
Library:				NRF24L01 software library for STM32 MCUs
Written by:			Mohamed Yaqoob
Date written:		25/10/2018
Last modified:	-/-
Description:

*/

#define leftshift(x) (1<<(x))

/* Memory Map */
#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_OBSERVE_TX  0x08
#define REG_CD          0x09
#define REG_RX_ADDR_P0  0x0A
#define REG_RX_ADDR_P1  0x0B
#define REG_RX_ADDR_P2  0x0C
#define REG_RX_ADDR_P3  0x0D
#define REG_RX_ADDR_P4  0x0E
#define REG_RX_ADDR_P5  0x0F
#define REG_TX_ADDR     0x10
#define REG_RX_PW_P0    0x11
#define REG_RX_PW_P1    0x12
#define REG_RX_PW_P2    0x13
#define REG_RX_PW_P3    0x14
#define REG_RX_PW_P4    0x15
#define REG_RX_PW_P5    0x16
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD	    	0x1C
#define REG_FEATURE	    0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define BIT_EN_CRC      3
#define BIT_CRCO        2
#define BIT_PWR_UP      1
#define BIT_PRIM_RX     0
#define BIT_ENAA_P5     5
#define BIT_ENAA_P4     4
#define BIT_ENAA_P3     3
#define BIT_ENAA_P2     2
#define BIT_ENAA_P1     1
#define BIT_ENAA_P0     0
#define BIT_ERX_P5      5
#define BIT_ERX_P4      4
#define BIT_ERX_P3      3
#define BIT_ERX_P2      2
#define BIT_ERX_P1      1
#define BIT_ERX_P0      0
#define BIT_AW          0
#define BIT_ARD         4
#define BIT_ARC         0
#define BIT_PLL_LOCK    4
#define BIT_RF_DR       3
#define BIT_RF_PWR      6
#define BIT_RX_DR       6
#define BIT_TX_DS       5
#define BIT_MAX_RT      4
#define BIT_RX_P_NO     1
#define BIT_TX_FULL     0
#define BIT_PLOS_CNT    4
#define BIT_ARC_CNT     0
#define BIT_TX_REUSE    6
#define BIT_FIFO_FULL   5
#define BIT_TX_EMPTY    4
#define BIT_RX_FULL     1
#define BIT_RX_EMPTY    0
#define BIT_DPL_P5	    5
#define BIT_DPL_P4	    4
#define BIT_DPL_P3	    3
#define BIT_DPL_P2	    2
#define BIT_DPL_P1	    1
#define BIT_DPL_P0	    0
#define BIT_EN_DPL	    2
#define BIT_EN_ACK_PAY  1
#define BIT_EN_DYN_ACK  0

/* Instruction Mnemonics */
#define CMD_R_REGISTER    0x00
#define CMD_W_REGISTER    0x20
#define CMD_REGISTER_MASK 0x1F
#define CMD_ACTIVATE      0x50
#define CMD_R_RX_PL_WID   0x60
#define CMD_R_RX_PAYLOAD  0x61
#define CMD_W_TX_PAYLOAD  0xA0
#define CMD_W_ACK_PAYLOAD 0xA8
#define CMD_FLUSH_TX      0xE1
#define CMD_FLUSH_RX      0xE2
#define CMD_REUSE_TX_PL   0xE3
#define CMD_NOP           0xFF

/* Non-P omissions */
#define LNA_HCURR   0

/* P model memory Map */
#define REG_RPD         0x09

/* P model bit Mnemonics */
#define RF_DR  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

//1. Power Amplifier function, NRF24_setPALevel() 
typedef enum { 
	RF24_PA_m18dB = 0,
	RF24_PA_m12dB,
	RF24_PA_m6dB,
	RF24_PA_0dB,
	RF24_PA_ERROR 
}rf24_pa_dbm_e ;
//2. NRF24_setDataRate() input
typedef enum { 
	RF24_1MBPS = 0,
	RF24_2MBPS,
	RF24_250KBPS
}rf24_datarate_e;
//3. NRF24_setCRCLength() input
typedef enum { 
	RF24_CRC_DISABLED = 0,
	RF24_CRC_8_bit,
	RF24_CRC_16_bit
}rf24_crclength_e;
//4. Pipe address registers
static const uint8_t NRF24_ADDR_REGS[7] = {
		REG_RX_ADDR_P0,
		REG_RX_ADDR_P1,
		REG_RX_ADDR_P2,
		REG_RX_ADDR_P3,
		REG_RX_ADDR_P4,
		REG_RX_ADDR_P5,
		REG_TX_ADDR
};
//5. RX_PW_Px registers addresses
static const uint8_t RF24_RX_PW_PIPE[6] = {
		REG_RX_PW_P0, 
		REG_RX_PW_P1,
		REG_RX_PW_P2,
		REG_RX_PW_P3,
		REG_RX_PW_P4,
		REG_RX_PW_P5
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//Chip Select function
void NRF24_DelayMicroSeconds(uint32_t uSec)
{
	uint32_t uSecVar = uSec;
	uSecVar = uSecVar* ((SystemCoreClock/1000000)/3);
	while(uSecVar--);
}
void NRF24_csn(int state)
{
	if(state) HAL_GPIO_WritePin(nrf24_PORT, nrf24_CSN_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(nrf24_PORT, nrf24_CSN_PIN, GPIO_PIN_RESET);
}
//Chip Enable
void NRF24_ce(int state)
{
	if(state) HAL_GPIO_WritePin(nrf24_PORT, nrf24_CE_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(nrf24_PORT, nrf24_CE_PIN, GPIO_PIN_RESET);
}
//Read single byte from a register
uint8_t NRF24_read_register(uint8_t reg)
{
	uint8_t spiBuf[2];
	uint8_t retData;
	//Put CSN low
	NRF24_csn(0);
	//Transmit register address
	spiBuf[0] = reg&0x1F;//read format
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf , 1, 100);
	//Receive data
	HAL_SPI_Receive(&nrf24_hspi, &spiBuf[1], 1, 100);
	retData = spiBuf[1];
	//Bring CSN high
	NRF24_csn(1);
	return retData;
}
//Read multiple bytes register
void NRF24_read_registerN(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t spiBuf[2];
	//Put CSN low
	NRF24_csn(0);
	//Transmit register address
	spiBuf[0] = reg&0x1F;//read format
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf, 1, 100);
	//Receive data
	HAL_SPI_Receive(&nrf24_hspi, buf, len, 100);
	//Bring CSN high
	NRF24_csn(1);
}
//Write single byte register
void NRF24_write_register(uint8_t reg, uint8_t value)
{
	uint8_t spiBuf[2];
	//Put CSN low
	NRF24_csn(0);
	//Transmit register address and data
	spiBuf[0] = reg|0x20;//write format
	spiBuf[1] = value;
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf, 2, 100);
	//Bring CSN high
	NRF24_csn(1);
}
//Write multipl bytes register
void NRF24_write_registerN(uint8_t reg, const uint8_t* buf, uint8_t len)
{
	uint8_t spiBuf;
	//Put CSN low
	NRF24_csn(0);
	//Transmit register address and data
	spiBuf = reg|0x20;//write format
	HAL_SPI_Transmit(&nrf24_hspi, &spiBuf, 1, 100);
	HAL_SPI_Transmit(&nrf24_hspi, (uint8_t*)buf, len, 100);
	//Bring CSN high
	NRF24_csn(1);
}
void NRF24_ACTIVATE_cmd(void)
{
	uint8_t cmdRxBuf[2];
	//Read data from Rx payload buffer
	NRF24_csn(0);
	cmdRxBuf[0] = CMD_ACTIVATE;//activate spi
	cmdRxBuf[1] = 0x73;//activate nrf
	HAL_SPI_Transmit(&nrf24_hspi, cmdRxBuf, 2, 100);
	NRF24_csn(1);
}

void printRadioSettings(void)
{
	uint8_t reg_val;
	char uartTxBuffer[100];
	sprintf(uartTxBuffer, "\n**********************************************\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	//Get CRC settings - Config Register
	reg_val = NRF24_read_register(0x00);
	if(reg_val & (1 << 3))
	{
		if(reg_val & (1 << 2)) sprintf(uartTxBuffer, "CRC: Enabled, 2 Bytes \n");
		else sprintf(uartTxBuffer, "CRC: Enabled, 1 Byte \n");	
	}
	else
	{
		sprintf(uartTxBuffer, "CRC: Disabled \n");
	}
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	//AutoAck on pipes
	reg_val = NRF24_read_register(0x01);
	sprintf(uartTxBuffer, "ENAA:	P0:	%d\n		P1:	%d\n		P2:	%d\n		P3:	%d\n		P4:	%d\n		P5:	%d\n",
	ifpositive(reg_val&(1<<0)), ifpositive(reg_val&(1<<1)), ifpositive(reg_val&(1<<2)), ifpositive(reg_val&(1<<3)), ifpositive(reg_val&(1<<4)), ifpositive(reg_val&(1<<5)));
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	//Enabled Rx addresses
	reg_val = NRF24_read_register(0x02);
	sprintf(uartTxBuffer, "EN_RXADDR: P0:	%d\n		P1:	%d\n		P2:	%d\n		P3:	%d\n		P4:	%d\n		P5:	%d\n",
	ifpositive(reg_val&(1<<0)), ifpositive(reg_val&(1<<1)), ifpositive(reg_val&(1<<2)), ifpositive(reg_val&(1<<3)), ifpositive(reg_val&(1<<4)), ifpositive(reg_val&(1<<5)));
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	//Address width
	reg_val = (NRF24_read_register(0x03))&0x03;//Reserved
	reg_val +=2;//show bytes
	sprintf(uartTxBuffer, "SETUP_AW: %d bytes \n", reg_val);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	//RF channel
	reg_val = (NRF24_read_register(0x05))&0x7F;//Rserved
	sprintf(uartTxBuffer, "RF_CH: %d CH \n", reg_val);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	//Data rate & RF_PWR
	reg_val = NRF24_read_register(0x06);
	if(reg_val & (1 << 3)) sprintf(uartTxBuffer, "Data Rate: 2Mbps \n");
	else sprintf(uartTxBuffer, "Data Rate: 1Mbps \n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	reg_val &= (3 << 1);
	reg_val = (reg_val>>1);
	switch (reg_val) 
   { 
			 case 0: sprintf(uartTxBuffer, "RF_PWR: -18dB \n"); 
       case 1: sprintf(uartTxBuffer, "RF_PWR: -12dB \n"); 
       case 2: sprintf(uartTxBuffer, "RF_PWR: -6dB \n"); 
       case 3: sprintf(uartTxBuffer, "RF_PWR:  0dB \n");
       default: sprintf(uartTxBuffer, "Error");                
   } 
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	//RX pipes addresses
	uint8_t pipeAddrs[5];
	NRF24_read_registerN(0x0A, pipeAddrs, 5);
	sprintf(uartTxBuffer, "RX_Pipe0 Addrs: %02X,%02X,%02X,%02X,%02X  \n", pipeAddrs[4], pipeAddrs[3], pipeAddrs[2],pipeAddrs[1],pipeAddrs[0]);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	
	NRF24_read_registerN(0x0B, pipeAddrs, 5);
	sprintf(uartTxBuffer, "RX_Pipe1 Addrs: %02X,%02X,%02X,%02X,%02X  \n", pipeAddrs[4], pipeAddrs[3], pipeAddrs[2],pipeAddrs[1],pipeAddrs[0]);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	
	NRF24_read_registerN(0x0C, pipeAddrs, 1);
	sprintf(uartTxBuffer, "RX_Pipe2 Addrs: %02X  \n", pipeAddrs[0]);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	
	NRF24_read_registerN(0x0D, pipeAddrs, 1);
	sprintf(uartTxBuffer, "RX_Pipe3 Addrs: %02X  \n", pipeAddrs[0]);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	
	NRF24_read_registerN(0x0E, pipeAddrs, 1);
	sprintf(uartTxBuffer, "RX_Pipe4 Addrs: %02X  \n", pipeAddrs[0]);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	
	NRF24_read_registerN(0x0F, pipeAddrs, 1);
	sprintf(uartTxBuffer, "RX_Pipe5 Addrs: %02X  \n", pipeAddrs[0]);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	
	NRF24_read_registerN(0x10, pipeAddrs, 5);
	sprintf(uartTxBuffer, "TX Addrs: %02X,%02X,%02X,%02X,%02X  \n", pipeAddrs[4], pipeAddrs[3], pipeAddrs[2],pipeAddrs[1],pipeAddrs[0]);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	
	//RX PW (Payload Width 0 - 32)
	reg_val = (NRF24_read_register(0x11))&0x3F;//Reserved
	sprintf(uartTxBuffer, "RX_PW_P0: %d bytes \n", reg_val);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	
	reg_val = (NRF24_read_register(0x12))&0x3F;//Reserved
	sprintf(uartTxBuffer, "RX_PW_P1: %d bytes \n", reg_val);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	
	reg_val = (NRF24_read_register(0x13))&0x3F;//Reserved
	sprintf(uartTxBuffer, "RX_PW_P2: %d bytes \n", reg_val);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	
	reg_val = (NRF24_read_register(0x14))&0x3F;//Reserved
	sprintf(uartTxBuffer, "RX_PW_P3: %d bytes \n", reg_val);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	
	reg_val = (NRF24_read_register(0x15))&0x3F;//Reserved
	sprintf(uartTxBuffer, "RX_PW_P4: %d bytes \n", reg_val);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	
	reg_val = (NRF24_read_register(0x16))&0x3F;//Reserved
	sprintf(uartTxBuffer, "RX_PW_P5: %d bytes \n", reg_val);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	
	//Dynamic payload enable for each pipe
	reg_val = NRF24_read_register(0x1c);
	sprintf(uartTxBuffer, "DYNPD_pipe: P0:	%d\n		P1:	%d\n		P2:	%d\n		P3:	%d\n		P4:	%d\n		P5:	%d\n",
	ifpositive(reg_val&(1<<0)), ifpositive(reg_val&(1<<1)), ifpositive(reg_val&(1<<2)), ifpositive(reg_val&(1<<3)), ifpositive(reg_val&(1<<4)), ifpositive(reg_val&(1<<5)));
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
	
	//EN_DPL (is Dynamic payload feature enabled ?)
	reg_val = NRF24_read_register(0x1d);
	if(reg_val&(1<<2)) sprintf(uartTxBuffer, "EN_DPL: Enabled \n");
	else sprintf(uartTxBuffer, "EN_DPL: Disabled \n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
		
	sprintf(uartTxBuffer, "\n**********************************************\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 10);
}
void NRF24_setRetries(uint8_t delay, uint8_t count)
{
	NRF24_write_register(REG_SETUP_RETR,delay<<BIT_ARD | count<<BIT_ARC);
}

//Set RF channel frequency
void NRF24_setChannel(uint8_t channel)
{
	const uint8_t max_channel = 127;
  NRF24_write_register(REG_RF_CH,MIN(channel,max_channel));
}
void NRF24_setPayloadSize(uint8_t size)
{
	const uint8_t max_payload_size = 32;
  payload_size = MIN(size,max_payload_size);
}
void NRF24_setPALevel( rf24_pa_dbm_e level )
{
	uint8_t setup = NRF24_read_register(REG_RF_SETUP) ;
  setup &= ~(leftshift(RF_PWR_LOW) | leftshift(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( level == RF24_PA_0dB)
  {
    setup |= (leftshift(RF_PWR_LOW) | leftshift(RF_PWR_HIGH)) ;
  }
  else if ( level == RF24_PA_m6dB )
  {
    setup |= leftshift(RF_PWR_HIGH) ;
  }
  else if ( level == RF24_PA_m12dB )
  {
    setup |= leftshift(RF_PWR_LOW);
  }
  else if ( level == RF24_PA_m18dB )
  {
    // nothing
  }
  else if ( level == RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= (leftshift(RF_PWR_LOW) | leftshift(RF_PWR_HIGH)) ;
  }

  NRF24_write_register( REG_RF_SETUP, setup ) ;
}
//Get transmit power level
rf24_pa_dbm_e NRF24_getPALevel( void )
{
	rf24_pa_dbm_e result = RF24_PA_ERROR ;
  uint8_t power = NRF24_read_register(REG_RF_SETUP) & (leftshift(RF_PWR_LOW) | leftshift(RF_PWR_HIGH));

  // switch uses RAM (evil!)
  if ( power == (leftshift(RF_PWR_LOW) | leftshift(RF_PWR_HIGH)) )
  {
    result = RF24_PA_0dB ;
  }
  else if ( power == leftshift(RF_PWR_HIGH) )
  {
    result = RF24_PA_m6dB ;
  }
  else if ( power == leftshift(RF_PWR_LOW) )
  {
    result = RF24_PA_m12dB ;
  }
  else
  {
    result = RF24_PA_m18dB ;
  }

  return result ;
}
void NRF24_setDataRate(rf24_datarate_e speed)
{
  uint8_t setup = NRF24_read_register(REG_RF_SETUP) ;
	
  setup &= ~(leftshift(RF_DR)) ;
	
	if ( speed == RF24_2MBPS )
	{
		setup |= leftshift(RF_DR);
	}
		
  NRF24_write_register(REG_RF_SETUP,setup);
}
void NRF24_setCRCLength(rf24_crclength_e length)
{
	uint8_t config = NRF24_read_register(REG_CONFIG) & ~( leftshift(BIT_CRCO) | leftshift(BIT_EN_CRC)) ;
  
  // switch uses RAM
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above. 
  }
  else if ( length == RF24_CRC_8_bit)
  {
    config |= leftshift(BIT_EN_CRC);
  }
  else
  {
    config |= leftshift(BIT_EN_CRC);
    config |= leftshift( BIT_CRCO );
  }
  NRF24_write_register( REG_CONFIG, config );
}
rf24_crclength_e NRF24_getCRCLength(void)
{
	rf24_crclength_e result = RF24_CRC_DISABLED;
  uint8_t config = NRF24_read_register(REG_CONFIG) & ( leftshift(BIT_CRCO) | leftshift(BIT_EN_CRC)) ;

  if ( config & leftshift(BIT_EN_CRC ) )
  {
    if ( config & leftshift(BIT_CRCO) )
      result = RF24_CRC_16_bit;
    else
      result = RF24_CRC_8_bit;
  }

  return result;
}
void NRF24_powerDown(void)
{
	NRF24_write_register(REG_CONFIG,NRF24_read_register(REG_CONFIG) & ~leftshift(BIT_PWR_UP));
}
void NRF24_disableDynamicPayloads(void)
{
	NRF24_write_register(REG_FEATURE,(NRF24_read_register(REG_FEATURE) &  ~(leftshift(BIT_EN_DPL))) );
	//Disable for all pipes 
	NRF24_write_register(REG_DYNPD,0x00);
	dynamic_payloads_enabled = 0;
}
void NRF24_setAutoAck(int enable)
{
	if ( enable )
    NRF24_write_register(REG_EN_AA, 0x3F);
  else
    NRF24_write_register(REG_EN_AA, 0x00);
}
void NRF24_begin(GPIO_TypeDef *nrf24PORT, uint16_t nrfCSN_Pin, uint16_t nrfCE_Pin, SPI_HandleTypeDef nrfSPI)
{
	//Copy SPI handle variable
	memcpy(&nrf24_hspi, &nrfSPI, sizeof(nrfSPI));
	//Copy Pins and Port variables
	nrf24_PORT = nrf24PORT;
	nrf24_CSN_PIN = nrfCSN_Pin;
	nrf24_CE_PIN = nrfCE_Pin;
	
	//Put pins to idle state
	NRF24_csn(1);
	NRF24_ce(0);
	//1.5 ms initial delay(internal clock)
	NRF24_DelayMicroSeconds(1500);
	
	//**** Soft Reset Registers default values ****//
	NRF24_write_register(REG_CONFIG, 0x08);
	NRF24_write_register(REG_EN_AA, 0x3f);
	NRF24_write_register(REG_EN_RXADDR, 0x03);
	NRF24_write_register(REG_SETUP_AW, 0x03);
	NRF24_write_register(REG_SETUP_RETR, 0x03);
	NRF24_write_register(REG_RF_CH, 0x02);
	NRF24_write_register(REG_RF_SETUP, 0x0f);
	NRF24_write_register(REG_STATUS, 0x0e);
	NRF24_write_register(REG_OBSERVE_TX, 0x00);
	NRF24_write_register(REG_CD, 0x00);
	uint8_t pipeAddrVar[5];
	pipeAddrVar[4]=0xE7; pipeAddrVar[3]=0xE7; pipeAddrVar[2]=0xE7; pipeAddrVar[1]=0xE7; pipeAddrVar[0]=0xE7; 
	NRF24_write_registerN(REG_RX_ADDR_P0, pipeAddrVar, 5);
	pipeAddrVar[4]=0xC2; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; 
	NRF24_write_registerN(REG_RX_ADDR_P1, pipeAddrVar, 5);
	NRF24_write_register(REG_RX_ADDR_P2, 0xC3);
	NRF24_write_register(REG_RX_ADDR_P3, 0xC4);
	NRF24_write_register(REG_RX_ADDR_P4, 0xC5);
	NRF24_write_register(REG_RX_ADDR_P5, 0xC6);
	pipeAddrVar[4]=0xE7; pipeAddrVar[3]=0xE7; pipeAddrVar[2]=0xE7; pipeAddrVar[1]=0xE7; pipeAddrVar[0]=0xE7; 
	NRF24_write_registerN(REG_TX_ADDR, pipeAddrVar, 5);
	NRF24_write_register(REG_RX_PW_P0, 0);
	NRF24_write_register(REG_RX_PW_P1, 0);
	NRF24_write_register(REG_RX_PW_P2, 0);
	NRF24_write_register(REG_RX_PW_P3, 0);
	NRF24_write_register(REG_RX_PW_P4, 0);
	NRF24_write_register(REG_RX_PW_P5, 0);
	
	NRF24_ACTIVATE_cmd();
	NRF24_write_register(REG_DYNPD, 0);
	NRF24_write_register(REG_FEATURE, 0);
	printRadioSettings();
}
void NRF24_initialize()
{
	//Initialise retries 15 and delay 1250 usec
	NRF24_setRetries(9, 8);
	NRF24_setAutoAck(0);
	//Initialise PA level to max (0dB)
	NRF24_setPALevel(RF24_PA_m6dB);
	//Initialise data rate to 1Mbps
	NRF24_setDataRate(RF24_2MBPS);
	//Initalise CRC length to 16-bit (2 bytes)
	NRF24_setCRCLength(RF24_CRC_16_bit);
	//Disable dynamic payload
	NRF24_disableDynamicPayloads();
	//Set payload size
	NRF24_setPayloadSize(32);
	
	//Reset status register
	NRF24_write_register(REG_STATUS,leftshift(BIT_RX_DR) | leftshift(BIT_TX_DS) | leftshift(BIT_MAX_RT) );
	//Initialise channel to 76
	NRF24_setChannel(76);
	//Flush TX buffer
	NRF24_write_register(CMD_FLUSH_TX, 0xFF);
  //Flush Rx buffer
	NRF24_write_register(CMD_FLUSH_RX, 0xFF);
	
	NRF24_powerDown();
	
	printRadioSettings();
}
void NRF24_write_payload(const void* buf, uint8_t len)
{
	uint8_t wrPayloadCmd;
	//Bring CSN low
	NRF24_csn(0);
	//Send Write Tx payload command followed by pbuf data
	wrPayloadCmd = CMD_W_TX_PAYLOAD;
	HAL_SPI_Transmit(&nrf24_hspi, &wrPayloadCmd, 1, 100);
	HAL_SPI_Transmit(&nrf24_hspi, (uint8_t *)buf, len, 100);
	//Bring CSN high
	NRF24_csn(1);
}
void nrf24_DebugUART_Init(UART_HandleTypeDef nrf24Uart)
{
	memcpy(&nrf24_huart, &nrf24Uart, sizeof(nrf24Uart));
}
void NRF24_stopListening(void)
{
	NRF24_ce(0);
	NRF24_write_register(CMD_FLUSH_TX, 0xFF);
	NRF24_write_register(CMD_FLUSH_RX, 0xFF);
}
void NRF24_openWritingPipe(uint64_t address)
{
	NRF24_write_registerN(REG_RX_ADDR_P0, (uint8_t *)(&address), 5);
  NRF24_write_registerN(REG_TX_ADDR, (uint8_t *)(&address), 5);
	
	const uint8_t max_payload_size = 32;
  NRF24_write_register(REG_RX_PW_P0,MIN(payload_size,max_payload_size));
}
void NRF24_startWrite( const void* buf, uint8_t len )
{
	// Transmitter power-up
  NRF24_write_register(REG_CONFIG, ( NRF24_read_register(REG_CONFIG) | leftshift(BIT_PWR_UP) ) & ~leftshift(BIT_PRIM_RX) );
  NRF24_DelayMicroSeconds(150);

  // Send the payload
  NRF24_write_payload( buf, len );

  // Enable Tx for 15usec
  NRF24_ce(1);
  NRF24_DelayMicroSeconds(15);
  NRF24_ce(0);
}
void NRF24_whatHappened(int *tx_ok,int *tx_fail,int *rx_ready)
{
	uint8_t status = NRF24_read_register(REG_STATUS);
	*tx_ok = 0;
	NRF24_write_register(REG_STATUS,leftshift(BIT_RX_DR) | leftshift(BIT_TX_DS) | leftshift(BIT_MAX_RT) );
  // Report to the user what happened
  *tx_ok = status & leftshift(BIT_TX_DS);
  *tx_fail = status & leftshift(BIT_MAX_RT);
  *rx_ready = status & leftshift(BIT_RX_DR);
}
int NRF24_available()
{
	uint8_t status = NRF24_read_register(REG_STATUS);

  int result = ( status & leftshift(BIT_RX_DR));

  if (result)
  {
    //set RX_status bit
    NRF24_write_register(REG_STATUS,(leftshift(BIT_RX_DR)) );

    //set RX_status bit and clear Tx_status bit
    if ( status & leftshift(BIT_TX_DS) )
    {
      NRF24_write_register( REG_STATUS,~(leftshift(BIT_TX_DS))| (leftshift(BIT_RX_DR)) );
    }
  }
  return result;
}
int NRF24_write( const void* buf, uint8_t len )
{
	int retStatus;
	//Start writing
	NRF24_write_register(REG_STATUS,leftshift(BIT_RX_DR) | leftshift(BIT_TX_DS) | leftshift(BIT_MAX_RT) );
	NRF24_startWrite(buf,len);
	//Data monitor
  uint8_t observe_tx;
  uint8_t status;
  uint32_t sent_at = HAL_GetTick();
	const uint32_t timeout = 10; //ms to wait for timeout
	do
  {
    NRF24_read_registerN(REG_OBSERVE_TX,&observe_tx,1);
		//Get status register
		status = NRF24_read_register(REG_STATUS);
  }
  while( ! ( status & ( leftshift(BIT_TX_DS) | leftshift(BIT_MAX_RT) ) ) && ( HAL_GetTick() - sent_at < timeout ) );
	
	int tx_ok, tx_fail;
  NRF24_whatHappened(&tx_ok,&tx_fail, &ack_payload_available);
	retStatus = tx_ok;
	//Power down
	NRF24_available();
	NRF24_write_register(CMD_FLUSH_TX, 0xFF);
	return retStatus;
}
void NRF24_startListening(void)
{
	//Power up and set to RX mode
	NRF24_write_register(REG_CONFIG, NRF24_read_register(REG_CONFIG) | 0x03);
	//Flush buffers
	NRF24_write_register(CMD_FLUSH_TX, 0xFF);
	NRF24_write_register(CMD_FLUSH_RX, 0xFF);
	//Set CE HIGH to start listenning
	NRF24_ce(1);
	//Wait for 130 uSec for the radio to come on
	NRF24_DelayMicroSeconds(150);
}
uint8_t NRF24_getPayloadSize(void)
{
	return payload_size;
}
void NRF24_read( void* buf, uint8_t len )
{
	uint8_t cmdRxBuf;
	uint8_t data_len = MIN(len, payload_size);
	//Read data from Rx payload buffer
	NRF24_csn(0);
	cmdRxBuf = CMD_R_RX_PAYLOAD;//cmd start read operation
	HAL_SPI_Transmit(&nrf24_hspi, &cmdRxBuf, 1, 100);
	HAL_SPI_Receive(&nrf24_hspi, buf, data_len, 100);
	NRF24_csn(1);
}
void NRF24_openReadingPipe(uint8_t number, uint64_t address)
{
	if(number <= 6)
	{
		if(number < 2)
		{
			//Address width is 5 bytes
			NRF24_write_registerN(NRF24_ADDR_REGS[number], (uint8_t *)(&address), 5);
		}
		else
		{
			//Address width is 1 byte
			NRF24_write_registerN(NRF24_ADDR_REGS[number], (uint8_t *)(&address), 1);
		}
		//Write payload size
		NRF24_write_register(RF24_RX_PW_PIPE[number],payload_size);
		//Enable pipe
		NRF24_write_register(REG_EN_RXADDR, (NRF24_read_register(REG_EN_RXADDR)) | leftshift(number));
	}	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	nrf24_DebugUART_Init(huart1);
	NRF24_begin(GPIOA, CSN_Pin, CE_Pin, hspi2);
	NRF24_initialize();
	
	NRF24_openReadingPipe(1, RxpipeAddrs);	
	NRF24_startListening();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		
		//	HAL_UART_Transmit(&huart1, (uint8_t *)"while\n", 5, 10);
		if(NRF24_available())
		{
			
			NRF24_read(myRxData, 32);
			myRxData[32] = '\n';
			HAL_UART_Transmit(&huart1, (uint8_t *)myRxData, 33, 10);
			//HAL_UART_Transmit(&huart1, (uint8_t *)"available\n", 5, 10);
		}
		
	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CE_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
