#include "main.h"
#include "SX126x.h"
#include "spi.h"


//Unneeded since we changed the class to a stuct

SX126x mainRadio;
void SX126xCreate(GPIO_PIN_TypeDef spiSelect, GPIO_PIN_TypeDef reset, GPIO_PIN_TypeDef busy, GPIO_PIN_TypeDef interrupt)
{
  mainRadio.SX126x_SPI_SELECT = spiSelect;
  mainRadio.SX126x_RESET      = reset;
  mainRadio.SX126x_BUSY       = busy;
  mainRadio.SX126x_INT0       = interrupt;

  mainRadio.txActive          = 0;

	/*
  pinMode(SX126x_SPI_SELECT, OUTPUT);
  pinMode(SX126x_RESET, OUTPUT);
  pinMode(SX126x_BUSY, INPUT);
  pinMode(SX126x_INT0, INPUT);

  SPI.begin();
	*/
}

int16_t begin(uint8_t packetType, uint32_t frequencyInHz, int8_t txPowerInDbm) 
{
  if ( txPowerInDbm > 22 )
    txPowerInDbm = 22;
  if ( txPowerInDbm < -3 )
    txPowerInDbm = -3;
  
  Reset();
  
  if ( 0x2A != GetStatus() )
  {
    //Serial.println("SX126x error, maybe no SPI connection?");
    return ERR_INVALID_MODE;
  }
  SetStandby(SX126X_STANDBY_RC);
  
  SetDio3AsTcxoCtrl(SX126X_DIO3_OUTPUT_3_3, RADIO_TCXO_SETUP_TIME << 6); // convert from ms to SX126x time base
  
	Calibrate(  SX126X_CALIBRATE_IMAGE_ON
                  | SX126X_CALIBRATE_ADC_BULK_P_ON
                  | SX126X_CALIBRATE_ADC_BULK_N_ON
                  | SX126X_CALIBRATE_ADC_PULSE_ON
                  | SX126X_CALIBRATE_PLL_ON
                  | SX126X_CALIBRATE_RC13M_ON
                  | SX126X_CALIBRATE_RC64K_ON
                  );

  SetDio2AsRfSwitchCtrl(1);

  SetStandby(SX126X_STANDBY_RC); 
  SetRegulatorMode(SX126X_REGULATOR_DC_DC);
  SetBufferBaseAddress(0, 0);
  SetPaConfig(0x04, 0x07, 0x00, 0x01);
  SetOvercurrentProtection(0x38);  // current max 30mA for the whole device
  SetPowerConfig(txPowerInDbm, SX126X_PA_RAMP_200U); //0 fuer Empfaenger
  SetDioIrqParams(SX126X_IRQ_ALL,  //all interrupts enabled
                  (SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE |SX126X_IRQ_TIMEOUT), //interrupts on DIO1
                  SX126X_IRQ_NONE,  //interrupts on DIO2
                  SX126X_IRQ_NONE); //interrupts on DIO3

  SetRfFrequency(frequencyInHz);
	return 0;
}


int16_t LoRaConfig(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint16_t preambleLength, uint8_t payloadLen, bool crcOn, bool invertIrq) 
{
  uint8_t ldro; //LowDataRateOptimize

  SetStopRxTimerOnPreambleDetect(0);
  SetLoRaSymbNumTimeout(0); 
  SetPacketType(SX126X_PACKET_TYPE_LORA); //RadioSetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
  SetModulationParams(spreadingFactor, bandwidth, codingRate, ldro);
  
  mainRadio.PacketParams[0] = (preambleLength >> 8) & 0xFF;
  mainRadio.PacketParams[1] = preambleLength;
  if ( payloadLen )
  {
    //fixed payload length
    mainRadio.PacketParams[2] = 0x01;
    mainRadio.PacketParams[3] = payloadLen;
  }
  else
  {
    mainRadio.PacketParams[2] = 0x00;
    mainRadio.PacketParams[3] = 0xFF;
  }

  if ( crcOn )
    mainRadio.PacketParams[4] = 0x01;
  else
    mainRadio.PacketParams[4] = 0x00;

  if ( invertIrq )
    mainRadio.PacketParams[5] = 0x01;
  else
    mainRadio.PacketParams[5] = 0x00;

  SPIwriteCommand(SX126X_CMD_SET_PACKET_PARAMS, mainRadio.PacketParams, 6);
  SetDioIrqParams(SX126X_IRQ_ALL,  //all interrupts enabled
                  (SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE, SX126X_IRQ_TIMEOUT), //interrupts on DIO1
                  SX126X_IRQ_NONE,  //interrupts on DIO2
                  SX126X_IRQ_NONE);
  //receive state no receive timeoout
  SetRx(0xFFFFFF);
	return 0;
}


uint8_t Receive(uint8_t *pData, uint16_t len) 
{
  uint8_t rxLen = 0;
  uint16_t irqRegs = GetIrqStatus();
  
  if( irqRegs & SX126X_IRQ_RX_DONE )
  {
    ClearIrqStatus(SX126X_IRQ_RX_DONE);
    ReadBuffer(pData, &rxLen, len);
  }
  
  return rxLen;
}


bool Send(uint8_t *pData, uint8_t len, uint8_t mode)
{
  uint16_t irq;
  bool rv = 0;
  
  if ( mainRadio.txActive == 0 )
  {
	  mainRadio.txActive = 1;
	  mainRadio.PacketParams[2] = 0x00; //Variable length packet (explicit header)
	  mainRadio.PacketParams[3] = len;
	  SPIwriteCommand(SX126X_CMD_SET_PACKET_PARAMS, mainRadio.PacketParams, 6);
	  
	  ClearIrqStatus(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT);
	  
	  WriteBuffer(pData, len);
	  SetTx(500);
	  
	  if ( mode & SX126x_TXMODE_SYNC )
	  {
	    irq = GetIrqStatus();
	    while ( (!(irq & SX126X_IRQ_TX_DONE)) && (!(irq & SX126X_IRQ_TIMEOUT)) )
	    {
	      irq = GetIrqStatus();
	    }
	    mainRadio.txActive = 0;
	
	    SetRx(0xFFFFFF);
	
	    if ( irq != SX126X_IRQ_TIMEOUT )
	    	rv = 1;
	  }
	  else
	  {
	    rv = 1;
	  }
	}
	
	return rv;
}


bool ReceiveMode(void)
{
  uint16_t irq;
  bool rv = 0;

  if ( mainRadio.txActive == 0 )
  {
    rv = 1;
  }
  else
  {
    irq = GetIrqStatus();
    if ( irq & (SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT) )
    { 
      SetRx(0xFFFFFF);
      mainRadio.txActive = 0;
      rv = 1;
    }
  }

  return rv;
}


void ReceiveStatus(uint8_t *rssiPacket, uint8_t *snrPacket)
{
    uint8_t buf[3];
     
    SPIreadCommand( SX126X_CMD_GET_PACKET_STATUS, buf, 3 );

    ( buf[1] < 128 ) ? ( *snrPacket = buf[1] >> 2 ) : ( *snrPacket = ( ( buf[1] - 256 ) >> 2 ) );
    *rssiPacket = -buf[0] >> 1;
}


void SetTxPower(int8_t txPowerInDbm)
{
  SetPowerConfig(txPowerInDbm, SX126X_PA_RAMP_200U);
}


void Reset()
{
	HAL_GPIO_WritePin(mainRadio.SX126x_RESET.port,mainRadio.SX126x_RESET.pin,0);
  HAL_Delay(20);
	HAL_GPIO_WritePin(mainRadio.SX126x_RESET.port,mainRadio.SX126x_RESET.pin,1);
  HAL_Delay(10);
}


void Wakeup()
{
  GetStatus();
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command SetStandby(...) is used to set the device in a configuration mode which is at an intermediate level of
//  consumption. In this mode, the chip is placed in halt mode waiting for instructions via SPI. This mode is dedicated to chip
//  configuration using high level commands such as SetPacketType(...).
//  By default, after battery insertion or reset operation (pin NRESET goes low), the chip will enter in STDBY_RC mode running
//  with a 13 MHz RC clock
//
//  Parameters
//  ----------
//  0: Device running on RC13M, set STDBY_RC mode
//  1: Device running on XTAL 32MHz, set STDBY_XOSC mode
//
//  Return value
//  ------------
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetStandby(uint8_t mode)
{
  uint8_t data = mode;
  SPIwriteCommand(SX126X_CMD_SET_STANDBY, &data, 1);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The host can retrieve chip status directly through the command GetStatus() : this command can be issued at any time and
//  the device returns the status of the device. The command GetStatus() is not strictly necessary since device returns status
//  information also on command bytes.
//
//  Parameters:
//  none
//
//  Return value:
//  Bit 6:4 Chipmode:0x0: Unused
//  Bit 3:1 Command Status
//  Bit 0: unused
//  Bit 7: unused
//----------------------------------------------------------------------------------------------------------------------------
uint8_t GetStatus(void)
{
  uint8_t rv;
  SPIreadCommand(SX126X_CMD_GET_STATUS, &rv, 1);
  return rv;
}


//----------------------------------------------------------------------------------------------------------------------------
//  The BUSY line is mandatory to ensure the host controller is ready to accept SPI commands.
//  When BUSY is high, the host controller must wait until it goes down again before sending another command.
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------



//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetDio3AsTcxoCtrl(uint8_t tcxoVoltage, uint32_t timeout)
{
    uint8_t buf[4];

    buf[0] = tcxoVoltage & 0x07;
    buf[1] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( timeout & 0xFF );

    SPIwriteCommand(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void Calibrate(uint8_t calibParam)
{
  uint8_t data = calibParam;
  SPIwriteCommand(SX126X_CMD_CALIBRATE, &data, 1);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetDio2AsRfSwitchCtrl(uint8_t enable)
{
  uint8_t data = enable;
  SPIwriteCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetRfFrequency(uint32_t frequency)
{
  uint8_t buf[4];
  uint32_t freq = 0;

  CalibrateImage(frequency);

  freq = (uint32_t)((double)frequency / FREQ_STEP);
  buf[0] = (uint8_t)((freq >> 24) & 0xFF);
  buf[1] = (uint8_t)((freq >> 16) & 0xFF);
  buf[2] = (uint8_t)((freq >> 8) & 0xFF);
  buf[3] = (uint8_t)(freq & 0xFF);
  SPIwriteCommand(SX126X_CMD_SET_RF_FREQUENCY, buf, 4);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void CalibrateImage(uint32_t frequency)
{
  uint8_t calFreq[2];

  if( frequency> 900000000 )
  {
      calFreq[0] = 0xE1;
      calFreq[1] = 0xE9;
  }
  else if( frequency > 850000000 )
  {
      calFreq[0] = 0xD7;
      calFreq[1] = 0xD8;
  }
  else if( frequency > 770000000 )
  {
      calFreq[0] = 0xC1;
      calFreq[1] = 0xC5;
  }
  else if( frequency > 460000000 )
  {
      calFreq[0] = 0x75;
      calFreq[1] = 0x81;
  }
  else if( frequency > 425000000 )
  {
      calFreq[0] = 0x6B;
      calFreq[1] = 0x6F;
  }
  SPIwriteCommand(SX126X_CMD_CALIBRATE_IMAGE, calFreq, 2);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetRegulatorMode(uint8_t mode)
{
    uint8_t data = mode;
    SPIwriteCommand(SX126X_CMD_SET_REGULATOR_MODE, &data, 1);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    SPIwriteCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetPowerConfig(int8_t power, uint8_t rampTime)
{
    uint8_t buf[2];

    if( power > 22 )
    {
        power = 22;
    }
    else if( power < -3 )
    {
        power = -3;
    }
    
    buf[0] = power;
    buf[1] = ( uint8_t )rampTime;
    SPIwriteCommand(SX126X_CMD_SET_TX_PARAMS, buf, 2);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut)
{
    uint8_t buf[4];

    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    SPIwriteCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The OCP is configurable by steps of 2.5 mA and the default value is re-configured automatically each time the function
//  SetPaConfig(...) is called. If the user wants to adjust the OCP value, it is necessary to change the register as a second 
//  step after calling the function SetPaConfig.
//
//  Parameters:
//  value: steps of 2,5mA (0x18 = 60mA)
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetOvercurrentProtection(uint8_t value)
{
  uint8_t buf[3];

  buf[0] = ((SX126X_REG_OCP_CONFIGURATION & 0xFF00) >> 8);
  buf[1] = (SX126X_REG_OCP_CONFIGURATION & 0x00FF);
  buf[2] = value;
  SPIwriteCommand(SX126X_CMD_WRITE_REGISTER, buf,3);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);
    SPIwriteCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetStopRxTimerOnPreambleDetect( bool enable )
{
  uint8_t data = (uint8_t)enable;
  SPIwriteCommand(SX126X_CMD_STOP_TIMER_ON_PREAMBLE, &data, 1);
}


//----------------------------------------------------------------------------------------------------------------------------
//  In LoRa mode, when going into Rx, the modem will lock as soon as a LoRa® symbol has been detected which may lead to
//  0 detection. This phenomena is quite rare but nevertheless possible. To avoid this, the command
//  SetLoRaSymbNumTimeout can be used to define the number of symbols which will be used to validate the correct
//  reception of a packet.
//
//  Parameters:
//  0:      validate the reception as soon as a LoRa® Symbol has been detected
//  1..255: When SymbNum is different from 0, the modem will wait for a total of SymbNum LoRa® symbol to validate, or not, the
//          correct detection of a LoRa packet. If the various states of the demodulator are not locked at this moment, the radio will
//          generate the RxTimeout IRQ.
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetLoRaSymbNumTimeout(uint8_t SymbNum)
{
  uint8_t data = SymbNum;
  SPIwriteCommand(SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT, &data, 1);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetPacketType(uint8_t packetType)
{
    uint8_t data = packetType;
    SPIwriteCommand(SX126X_CMD_SET_PACKET_TYPE, &data, 1);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetModulationParams(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint8_t lowDataRateOptimize)
{
  uint8_t data[4];
  //currently only LoRa supported
  data[0] = spreadingFactor;
  data[1] = bandwidth;
  data[2] = codingRate;
  data[3] = lowDataRateOptimize;
  SPIwriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, data, 4);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
uint16_t GetIrqStatus( void )
{
    uint8_t data[2];
    SPIreadCommand(SX126X_CMD_GET_IRQ_STATUS, data, 2);
    return (data[0] << 8) | data[1];
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void ClearIrqStatus(uint16_t irq)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irq >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irq & 0x00FF);
    SPIwriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetRx(uint32_t timeoutInMs)
{
    uint8_t buf[3];
    uint32_t tout = (timeoutInMs * SystemCoreClock)/1000;
    buf[0] = (uint8_t)((tout >> 16) & 0xFF);
    buf[1] = (uint8_t)((tout >> 8) & 0xFF);
    buf[2] = (uint8_t )(tout & 0xFF);
    SPIwriteCommand(SX126X_CMD_SET_RX, buf, 3);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command SetTx() sets the device in transmit mode. When the last bit of the packet has been sent, an IRQ TX_DONE
//  is generated. A TIMEOUT IRQ is triggered if the TX_DONE IRQ is not generated within the given timeout period.
//  The chip goes back to STBY_RC mode after a TIMEOUT IRQ or a TX_DONE IRQ.
//  he timeout duration can be computed with the formula: Timeout duration = Timeout * 15.625 μs
//
//  Parameters:
//  0: Timeout disable, Tx Single mode, the device will stay in TX Mode until the packet is transmitted
//  other: Timeout in milliseconds, timeout active, the device remains in TX mode. The maximum timeout is then 262 s.
//  
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void SetTx(uint32_t timeoutInMs)
{
	
    uint8_t buf[3];
    uint32_t tout = (timeoutInMs * SystemCoreClock)/1000;
    buf[0] = (uint8_t)((tout >> 16) & 0xFF);
    buf[1] = (uint8_t)((tout >> 8) & 0xFF);
    buf[2] = (uint8_t )(tout & 0xFF);
    SPIwriteCommand(SX126X_CMD_SET_TX, buf, 3);
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
void GetRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBufferPointer)
{
    uint8_t buf[2];

    SPIreadCommand( SX126X_CMD_GET_RX_BUFFER_STATUS, buf, 2);

    *payloadLength = buf[0];
    *rxStartBufferPointer = buf[1];
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
uint8_t ReadBuffer(uint8_t *rxData, uint8_t *rxDataLen,  uint8_t maxLen)
{
    uint8_t offset = 0, cmd1 = SX126X_CMD_READ_BUFFER, cmd2 = SX126X_CMD_NOP;
    
    GetRxBufferStatus(rxDataLen, &offset);
    if( *rxDataLen > maxLen )
    {
        return 1;
    }
    
    while(HAL_GPIO_ReadPin(mainRadio.SX126x_BUSY.port,mainRadio.SX126x_BUSY.pin))
		{
			HAL_Delay(100);
		}
    
		//bring low CS
		HAL_GPIO_WritePin(mainRadio.SX126x_SPI_SELECT.port,mainRadio.SX126x_SPI_SELECT.pin,0);
		
		
		//write the data
		HAL_SPI_Transmit(&hspi1,&cmd1,1,1000);
		HAL_SPI_Transmit(&hspi1,&offset,1,1000);
		HAL_SPI_Transmit(&hspi1,&cmd2,1,1000);
		
		HAL_SPI_Receive(&hspi1,rxData,*rxDataLen,1000);

		//bring high CS
		HAL_GPIO_WritePin(mainRadio.SX126x_SPI_SELECT.port,mainRadio.SX126x_SPI_SELECT.pin,1);

    return 0;
}


//----------------------------------------------------------------------------------------------------------------------------
//  The command...
//
//  Parameters:
//  none
//
//
//  Return value:
//  none
//  
//----------------------------------------------------------------------------------------------------------------------------
uint8_t WriteBuffer(uint8_t *txData, uint8_t txDataLen)
{
	uint8_t cmd = SX126X_CMD_WRITE_BUFFER;
	
	//wait until Busy goes low
	while(HAL_GPIO_ReadPin(mainRadio.SX126x_BUSY.port,mainRadio.SX126x_BUSY.pin))
	{
		HAL_Delay(100);
	}
	
	//bring low CS
	HAL_GPIO_WritePin(mainRadio.SX126x_SPI_SELECT.port,mainRadio.SX126x_SPI_SELECT.pin,0);
	
	//Send the cmd
	HAL_SPI_Transmit(&hspi1,&cmd,1,1000);
	
	//write the data
	HAL_SPI_Transmit(&hspi1,txData,txDataLen,1000);
	
	//bring High CS
	HAL_GPIO_WritePin(mainRadio.SX126x_SPI_SELECT.port,mainRadio.SX126x_SPI_SELECT.pin,1);
	

   return 0;
}

//Transmit the command and then reads an SPI packet
void SPIwriteCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes)
{
	//wait until Busy goes low
	while(HAL_GPIO_ReadPin(mainRadio.SX126x_BUSY.port,mainRadio.SX126x_BUSY.pin))
	{
		HAL_Delay(100);
	}
	
	//bring low CS
	HAL_GPIO_WritePin(mainRadio.SX126x_SPI_SELECT.port,mainRadio.SX126x_SPI_SELECT.pin,0);
	
	//Send data
	HAL_SPI_Transmit(&hspi1,&cmd,1,1000);
	
	//read expected amount of data
	HAL_SPI_Receive(&hspi1,data,numBytes,1000);
	
	//bring High CS
	HAL_GPIO_WritePin(mainRadio.SX126x_SPI_SELECT.port,mainRadio.SX126x_SPI_SELECT.pin,1);
	
}

//Transmit the buffer after busy goes low
void SPIreadCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes) {
  //SPItransfer(cmd, 0, NULL, data, numBytes, waitForBusy);

	
	//wait until Busy goes low
	while(HAL_GPIO_ReadPin(mainRadio.SX126x_BUSY.port,mainRadio.SX126x_BUSY.pin))
	{
		HAL_Delay(100);
	}
	
	//bring low CS
	HAL_GPIO_WritePin(mainRadio.SX126x_SPI_SELECT.port,mainRadio.SX126x_SPI_SELECT.pin,0);
	
	
	//Send comand
	HAL_SPI_Transmit(&hspi1,&cmd,1,1000);
	
	//Send data
	HAL_SPI_Transmit(&hspi1,data,numBytes,1000);
	
	
	//bring High CS
	HAL_GPIO_WritePin(mainRadio.SX126x_SPI_SELECT.port,mainRadio.SX126x_SPI_SELECT.pin,1);
}

/*
void SPItransfer(bool write, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes, bool waitForBusy) {
  
  // ensure BUSY is low (state meachine ready)
  // TODO timeout
	
  //while(digitalRead(SX126x_BUSY));
	
	//wait until Busy goes low
	while(HAL_GPIO_ReadPin(mainRadio.SX126x_BUSY.port,mainRadio.SX126x_BUSY.pin))
	{
		HAL_Delay(100);
	}

  // start transfer
	HAL_GPIO_WritePin(mainRadio.SX126x_SPI_SELECT.port,mainRadio.SX126x_SPI_SELECT.pin,0);
  //digitalWrite(SX126x_SPI_SELECT, LOW);
  //SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
	HAL_SPI_Transmit(&hspi1,dataIn,sizeof(dataIn),1000);
  // send command byte
  //SPI.transfer(cmd);
  
  // send/receive all bytes
  if(write) {
    //Serial.print("SPI write: CMD=0x");
    //Serial.print(cmd, HEX);
    //Serial.print(" DataOut: ");
    for(uint8_t n = 0; n < numBytes; n++) {
      uint8_t in = SPI.transfer(dataOut[n]);
      //Serial.print(dataOut[n], HEX);
      //Serial.print(" ");
    }
    //Serial.println();
  } else {
    //Serial.print("SPI read:  CMD=0x");
    //Serial.print(cmd, HEX);
    // skip the first byte for read-type commands (status-only)
    uint8_t in = SPI.transfer(SX126X_CMD_NOP);
    ////Serial.println((SX126X_CMD_NOP, HEX));
    //Serial.print(" DataIn: ");

    for(uint8_t n = 0; n < numBytes; n++) {
      dataIn[n] = SPI.transfer(SX126X_CMD_NOP);
      ////Serial.println((SX126X_CMD_NOP, HEX));
      //Serial.print(dataIn[n], HEX);
      //Serial.print(" ");
    }
    //Serial.println();
  }

  // stop transfer
  SPI.endTransaction();
  digitalWrite(SX126x_SPI_SELECT, HIGH);

  // wait for BUSY to go high and then low
  // TODO timeout
  if(waitForBusy) {
    delayMicroseconds(1);
    while(digitalRead(SX126x_BUSY));
  }
}
	*/
