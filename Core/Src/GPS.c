#include "GPSConfig.h"
#include "GPS.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

GPS_t GPS;

//##################################################################################################################
double convertDegMinToDecDeg (float degMin)
{
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}
//##################################################################################################################
void	GPS_Init(void)
{
	GPS.rxIndex=0;
	GPS.GPGGA.Latitude=0;
	GPS.GPGGA.Longitude=0;
	HAL_UART_Receive_IT(&_GPS_USART,&GPS.rxTmp,1);	
}
//##################################################################################################################
void	GPS_CallBack(void)
{
	GPS.LastTime=HAL_GetTick();
	if(GPS.rxIndex < sizeof(GPS.rxBuffer)-2)
	{
		GPS.rxBuffer[GPS.rxIndex] = GPS.rxTmp;
		GPS.rxIndex++;
	}	
	HAL_UART_Receive_IT(&_GPS_USART,&GPS.rxTmp,1);
}
//##################################################################################################################
uint8_t	GPS_Process(void)
{
	uint8_t returnVal=1;
	if( (HAL_GetTick()-GPS.LastTime>50) && (GPS.rxIndex>0))
	{
		char	*str;
		#if (_GPS_DEBUG==1)
		printf("%s",GPS.rxBuffer);
		#endif
		str=strstr((char*)GPS.rxBuffer,"$GNRMC,");
		if(str!=NULL)
		{
			//memset(&GPS.GPGGA,0,sizeof(GPS.GPGGA));
			GPS_Parse_GNRMC(str,&GPS);
			//sscanf(str,"$GPGGA,%2hhd%2hhd%2hhd.%3hd,%f,%c,%f,%c,%hhd,%hhd,%f,%f,%c,%hd,%s,*%2s\r\n",&GPS.GPGGA.UTC_Hour,&GPS.GPGGA.UTC_Min,&GPS.GPGGA.UTC_Sec,&GPS.GPGGA.UTC_MicroSec,&GPS.GPGGA.Latitude,&GPS.GPGGA.NS_Indicator,&GPS.GPGGA.Longitude,&GPS.GPGGA.EW_Indicator,&GPS.GPGGA.PositionFixIndicator,&GPS.GPGGA.SatellitesUsed,&GPS.GPGGA.HDOP,&GPS.GPGGA.MSL_Altitude,&GPS.GPGGA.MSL_Units,&GPS.GPGGA.AgeofDiffCorr,GPS.GPGGA.DiffRefStationID,GPS.GPGGA.CheckSum);
			/*
			if(GPS.GPGGA.NS_Indicator==0)
				GPS.GPGGA.NS_Indicator='-';
			if(GPS.GPGGA.EW_Indicator==0)
				GPS.GPGGA.EW_Indicator='-';
			if(GPS.GPGGA.Geoid_Units==0)
				GPS.GPGGA.Geoid_Units='-';
			if(GPS.GPGGA.MSL_Units==0)
				GPS.GPGGA.MSL_Units='-';
			*/
			if(GPS.GPGGA.Latitude != 0 || GPS.GPGGA.Longitude !=0 )
			{
				returnVal = 0;
				GPS.GPGGA.LatitudeDecimal=convertDegMinToDecDeg(GPS.GPGGA.Latitude);
				GPS.GPGGA.LongitudeDecimal=convertDegMinToDecDeg(GPS.GPGGA.Longitude);	
			}		
		}	
		memset(GPS.rxBuffer,0,sizeof(GPS.rxBuffer));
		GPS.rxIndex=0;
	}
	HAL_UART_Receive_IT(&_GPS_USART,&GPS.rxTmp,1);
	return returnVal;
}


uint8_t	GPS_Process_GNRMC(void)
{
	uint8_t returnVal=1;
	if( (HAL_GetTick()-GPS.LastTime>50) && (GPS.rxIndex>0))
	{
		char	*str;
		#if (_GPS_DEBUG==1)
		printf("%s",GPS.rxBuffer);
		#endif
		str=strstr((char*)GPS.rxBuffer,"$GNRMC,");
		if(str!=NULL)
		{
			GPS_Parse_GNRMC(str,&GPS);
			if(GPS.GPGGA.Latitude != 0 || GPS.GPGGA.Longitude !=0 )
			{
				returnVal = 0;
				GPS.GPGGA.LatitudeDecimal=convertDegMinToDecDeg(GPS.GPGGA.Latitude);
				GPS.GPGGA.LongitudeDecimal=convertDegMinToDecDeg(GPS.GPGGA.Longitude);	
			}		
		}	
		memset(GPS.rxBuffer,0,sizeof(GPS.rxBuffer));
		GPS.rxIndex=0;
	}
	HAL_UART_Receive_IT(&_GPS_USART,&GPS.rxTmp,1);
	return returnVal;
}

uint8_t	GPS_Process_GPGSV(void)
{
	uint8_t returnVal=1;
	if( (HAL_GetTick()-GPS.LastTime>50) && (GPS.rxIndex>0))
	{
		char	*str;
		#if (_GPS_DEBUG==1)
		printf("%s",GPS.rxBuffer);
		#endif
		str=strstr((char*)GPS.rxBuffer,"$GPGSV,");
		if(str!=NULL)
		{
			GPS_Parse_GPGSV(str,&GPS);
			if(GPS.GPGGA.SatellitesVisible!=0)
			{
				returnVal = 0;	
			}		
		}	
		memset(GPS.rxBuffer,0,sizeof(GPS.rxBuffer));
		GPS.rxIndex=0;
	}
	HAL_UART_Receive_IT(&_GPS_USART,&GPS.rxTmp,1);
	return returnVal;
}
//##################################################################################################################

void Print_GPS_Data(UART_HandleTypeDef *huart, GPS_t* gpsData)
{
	uint8_t dataPacket[40];
	uint16_t dataUsed = snprintf((char *)dataPacket,40,"%f,%f-%d/%d\r\n",GPS.GPGGA.LatitudeDecimal,GPS.GPGGA.LongitudeDecimal,GPS.GPGGA.SatellitesUsed,GPS.GPGGA.SatellitesVisible);
	//snprint(40,"%f,%f-%d/%d\n",GPS.GPGGA.Latitude,GPS.GPGGA.Longitude,GPS.GPGGA.SatellitesUsed,GPS.GPGGA.SatellitesVisible);
	//HAL_UART_Transmit(&_COM_USART,dataPacket,dataUsed,100);
}


/**
  * @brief  Parses NMEA GNRMC sentences.
  * @note   Example sentence: "$GNRMC,042149.00,A,3259.28330,N,11715.37928,W,0.468,  ,030921,   , ,A*76"
  * @param  nmeaSentenceIn Pointer to a char array containing a GNRMC sentence.
  * @param  GPS Pointer to a GPS_t data type where the collected info will be stored.
  */
void GPS_Parse_GNRMC(char* nmeaSentenceIn, GPS_t* GPS)
{
    sscanf(nmeaSentenceIn,"$GNRMC,%2hhd%2hhd%2hhd.%3hd,A,%f,%c,%f,%c,%hhd,%hhd,%f,%f,%c,%hd,%s,*%2s\r\n",&GPS->GPGGA.UTC_Hour,&GPS->GPGGA.UTC_Min,&GPS->GPGGA.UTC_Sec,&GPS->GPGGA.UTC_MicroSec,&GPS->GPGGA.Latitude,&GPS->GPGGA.NS_Indicator,&GPS->GPGGA.Longitude,&GPS->GPGGA.EW_Indicator,&GPS->GPGGA.PositionFixIndicator,&GPS->GPGGA.SatellitesUsed,&GPS->GPGGA.HDOP,&GPS->GPGGA.MSL_Altitude,&GPS->GPGGA.MSL_Units,&GPS->GPGGA.AgeofDiffCorr,GPS->GPGGA.DiffRefStationID,GPS->GPGGA.CheckSum);

}

/**
  * @brief  Parses NMEA GNGGA sentences.
  * @note   Example sentence: "$GNGGA,042149.00,3259.28330,N,11715.37928,W,1,06,2.56,6.3,M,-33.6,M,,*70"
  * @param  nmeaSentenceIn Pointer to a char array containing a GNRMC sentence.
  * @param  GPS Pointer to a GPS_t data type where the collected info will be stored.
  */
void GPS_Parse_GNGGA(char* nmeaSentenceIn, GPS_t* GPS)
{
    char gpsQuality;    //0: GNSS fix not available, 1: GNSS fix valid, 4: RTK fixed ambiguities, 5: RTK float ambiguities
    sscanf(nmeaSentenceIn,"$GNGGA,%2hhd%2hhd%2hhd.%3hd,%f,%c,%f,%c,%hhd,%hhd,%f,%f,%c,%f,%c,%hd,*%2s\r\n",&GPS->GPGGA.UTC_Hour,&GPS->GPGGA.UTC_Min,&GPS->GPGGA.UTC_Sec,&GPS->GPGGA.UTC_MicroSec,&GPS->GPGGA.Latitude,&GPS->GPGGA.NS_Indicator,&GPS->GPGGA.Longitude,&GPS->GPGGA.EW_Indicator,&gpsQuality,&GPS->GPGGA.SatellitesUsed,&GPS->GPGGA.HDOP,&GPS->GPGGA.MSL_Altitude,&GPS->GPGGA.MSL_Units,&GPS->GPGGA.Geoid_Separation,&GPS->GPGGA.Geoid_Units,&GPS->GPGGA.AgeofDiffCorr,GPS->GPGGA.CheckSum);

}

/**
  * @brief  Parses NMEA GNGLL sentences.
  * @note   Example sentence: "$GNGLL,3259.28286,N,11715.37930,W,042148.00,A,A*68"
  * @param  nmeaSentenceIn Pointer to a char array containing a GNRMC sentence.
  * @param  GPS Pointer to a GPS_t data type where the collected info will be stored.
  */
void GPS_Parse_GNGLL(char* nmeaSentenceIn, GPS_t* GPS)
{
    char dataSatus; // A = Data valid, V = Data invalid
    char modeInd;   //Positioning system mode indicator - A = Autonomous, D = Differential, M = Manual input, N = Data not valid
    sscanf(nmeaSentenceIn,"$GNGLL,%f,%c,%f,%c,%2hhd%2hhd%2hhd.%3hd,%c,%c*%2s\r\n",&GPS->GPGGA.Latitude,&GPS->GPGGA.NS_Indicator,&GPS->GPGGA.Longitude,&GPS->GPGGA.EW_Indicator,&GPS->GPGGA.UTC_Hour,&GPS->GPGGA.UTC_Min,&GPS->GPGGA.UTC_Sec,&GPS->GPGGA.UTC_MicroSec,&dataSatus, &modeInd, GPS->GPGGA.CheckSum);

}

/**
  * @brief  Parses NMEA GPGSV sentences.
  * @note   Example sentence: "$GPGSV,4,3,13,20,00,326,,22,10,170,20,26,09,041,,27,21,102,*71"
  * @param  nmeaSentenceIn Pointer to a char array containing a GNRMC sentence.
  * @param  GPS Pointer to a GPS_t data type where the collected info will be stored.
  */
void GPS_Parse_GPGSV(char* nmeaSentenceIn, GPS_t* GPS)
{
    int msgCount,msgNumer;
    sscanf(nmeaSentenceIn,"$GPGSV,%d,%d,%hhd,",&msgCount,&msgNumer, &GPS->GPGGA.SatellitesVisible);
}

