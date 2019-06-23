#include "NRF24.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define ifpositive(x) (((x)>0) ? 1:0)

static GPIO_TypeDef		*nrf24_PORT;
static uint16_t				nrf24_CSN_PIN;
static uint16_t				nrf24_CE_PIN;
//SPI handle
static SPI_HandleTypeDef nrf24_hspi;
//Debugging UART handle
static UART_HandleTypeDef nrf24_huart;

static uint64_t pipe0_reading_address;
static uint8_t payload_size;
static int dynamic_payloads_enabled; 

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
void NRF24_stopListening(void)
{
	NRF24_ce(0);
	NRF24_write_register(CMD_FLUSH_TX, 0xFF);
	NRF24_write_register(CMD_FLUSH_RX, 0xFF);
}
void NRF24_openWritingPipe(uint8_t number,uint64_t address)
{	
	const uint8_t max_payload_size = 32;
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
		NRF24_write_registerN(REG_TX_ADDR, (uint8_t *)(&address), 5);
		//Write payload size
		NRF24_write_register(RF24_RX_PW_PIPE[number],MIN(payload_size,max_payload_size));
	}
}

void NRF24_startWrite( const void* buf, uint8_t len )
{
	//mode Transmitter and power-up
  NRF24_write_register(REG_CONFIG, ( NRF24_read_register(REG_CONFIG) | leftshift(BIT_PWR_UP) ) & ~leftshift(BIT_PRIM_RX) );
  NRF24_DelayMicroSeconds(150); //more than 130us

  // Send the payload
	uint8_t wrPayloadCmd;
	//Bring CSN Low
  NRF24_csn(0);
	//Send Write Tx payload command followed by pbuf data
	wrPayloadCmd = CMD_W_TX_PAYLOAD; //0xA0
	HAL_SPI_Transmit(&nrf24_hspi, &wrPayloadCmd, 1, 100);
	HAL_SPI_Transmit(&nrf24_hspi, (uint8_t *)buf, len, 100);
	//Bring CSN high
	NRF24_csn(1);

  // Enable Tx for 15usec
  NRF24_ce(1);
  NRF24_DelayMicroSeconds(20); //more than 10us
  NRF24_ce(0);
}
int NRF24_write( const void* buf, uint8_t len )
{
	//int retStatus;
	int retstatus = (NRF24_read_register(REG_STATUS)) & leftshift(BIT_TX_DS) ;
  if (retstatus)
  {
    //set Tx_status bit
    NRF24_write_register(REG_STATUS,(leftshift(BIT_TX_DS)) );

    //set RX_status bit and clear Tx_status bit
    if ( retstatus & leftshift(BIT_RX_DR) )
    {
      NRF24_write_register( REG_STATUS,~(leftshift(BIT_RX_DR))| (leftshift(BIT_TX_DS)) );
    }
  }
	NRF24_startWrite(buf,len);
	//Data monitor
  uint8_t observe_tx;
  uint8_t status;
  uint32_t sent_at = HAL_GetTick();
	const uint32_t timeout = 10; //ms to wait for timeout
	do
  {
   status = NRF24_read_register(REG_STATUS);
  }
  while( ! ( status & ( leftshift(BIT_TX_DS) | leftshift(BIT_MAX_RT) ) ) && ( HAL_GetTick() - sent_at < timeout ) );

	retstatus = status & leftshift(BIT_TX_DS);
	//Power down
	NRF24_write_register(CMD_FLUSH_TX, 0xFF);
	return retstatus;
}

