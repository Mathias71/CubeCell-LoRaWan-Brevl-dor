#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into GPIO5 on the CubeCell
#define ONE_WIRE_BUS1 GPIO1
#define ONE_WIRE_BUS2 GPIO2

OneWire oneWire1(ONE_WIRE_BUS1);
OneWire oneWire2(ONE_WIRE_BUS2);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors1(&oneWire1);
DallasTemperature sensors2(&oneWire2);

/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

/* OTAA para*/
uint8_t devEui[] = { 0xac, 0x1f, 0x09, 0xff, 0xfe, 0x01, 0x5e, 0x05 };
uint8_t appEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x03, 0xE5, 0x75 };
uint8_t appKey[] = { 0x3f, 0x21, 0x6e, 0xc7, 0x98, 0x42, 0x9d, 0xf5, 0x27, 0x10, 0xee, 0xaa, 0xfb, 0x93, 0xdb, 0xba };

/* ABP para*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

/*LoraWan channelsmask*/
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 1000 * 60    * 29; //1000ms * 60s * 30min

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 3;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	*appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	*if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	*if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	*for example, if use REGION_CN470, 
	*the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	*/
  //Vext ON
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext,LOW);
  delay(500);

  uint16_t batteryVoltage = getBatteryVoltage();
    
  sensors1.begin();
  sensors2.begin();

  sensors1.requestTemperatures(); // Send the command to get temperatures
  sensors2.requestTemperatures(); // Send the command to get temperatures
  
  float temp1 = sensors1.getTempCByIndex(0);
  float temp2 = sensors2.getTempCByIndex(0);
  
  digitalWrite(Vext,HIGH);
  pinMode(Vext,INPUT);
    
  appDataSize = 10;
  appData[0] = (uint8_t)(batteryVoltage>>8);
  appData[1] = (uint8_t)getBatteryVoltage();

  unsigned char *puc;
  puc = (unsigned char *)(&temp1);
  appData[2] = puc[0];
  appData[3] = puc[1];
  appData[4] = puc[2];
  appData[5] = puc[3];
  //Serial.println(temp1);
  
  puc = (unsigned char *)(&temp2);
  appData[6] = puc[0];
  appData[7] = puc[1];
  appData[8] = puc[2];
  appData[9] = puc[3];
  //Serial.println(temp2);
  //Serial.println();
}


void setup() {
	Serial.begin(115200);

#if(AT_SUPPORT)
	enableAt();
#endif
  //LoRaWAN.displayMcuInit();
	deviceState = DEVICE_STATE_INIT;
  
	LoRaWAN.ifskipjoin();
}

void loop()
{
	switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
#if(LORAWAN_DEVEUI_AUTO)
			LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
			getDevParam();
#endif
			printDevParam();
			LoRaWAN.init(loraWanClass,loraWanRegion);
			deviceState = DEVICE_STATE_JOIN;
			break;
		}
		case DEVICE_STATE_JOIN:
		{
      //LoRaWAN.displayJoining();
			LoRaWAN.join();
			break;
		}
		case DEVICE_STATE_SEND:
		{
      //LoRaWAN.displaySending();
			prepareTxFrame( appPort );
			LoRaWAN.send();
			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
			// Schedule next packet transmission
			txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
			LoRaWAN.cycle(txDutyCycleTime);
			deviceState = DEVICE_STATE_SLEEP;
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
      //LoRaWAN.displayAck();
			LoRaWAN.sleep();
			break;
		}
		default:
		{
			deviceState = DEVICE_STATE_INIT;
			break;
		}
	}
}
