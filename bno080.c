#include "bno080.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "math.h"

//Registers
const uint8_t CHANNEL_COMMAND = 0;
const uint8_t CHANNEL_EXECUTABLE = 1;
const uint8_t CHANNEL_CONTROL = 2;
const uint8_t CHANNEL_REPORTS = 3;
const uint8_t CHANNEL_WAKE_REPORTS = 4;
const uint8_t CHANNEL_GYRO = 5;

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define BNO080_ADDR0 0x4B
#define BNO080_ADDR1 0x4A

#define BNO_TWI_TIMEOUT 100000

#define BNO_TWI_SCL_PIN_0 15
#define BNO_TWI_SDA_PIN_0 16

#define BNO_TWI_BUFFER_SIZE 32

static const nrf_drv_twi_t m_twi_instance_0 = NRF_DRV_TWI_INSTANCE(0);

volatile static bool twi_tx_done = false;
volatile static bool twi_rx_done = false;

uint8_t twi_tx_buffer[BNO_TWI_BUFFER_SIZE];

//Global Variables
int16_t rotationVector_Q1 = 14;
int16_t rotationVectorAccuracy_Q1 = 12; //Heading accuracy estimate in radians. The Q point is 12.
int16_t accelerometer_Q1 = 8;
int16_t linear_accelerometer_Q1 = 8;
int16_t gyro_Q1 = 9;
int16_t magnetometer_Q1 = 4;
int16_t angular_velocity_Q1 = 10;


bool bno080_begin(bno080 * bno)
{
	//Begin by resetting the IMU
	bno080_softReset(bno);

	//Check communication with device
	bno->shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	bno->shtpData[1] = 0;							  //Reserved

	//Transmit packet on channel 2, 2 bytes
	bno080_sendPacket(CHANNEL_CONTROL, 2, bno);

	//Now we wait for response
	if (bno080_receivePacket(bno) == true)
	{
		if (bno->shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			return (true);
		}
	}

	return (false); //Something went wrong
}


static void nrf_drv_bno_twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch(p_event->xfer_desc.type)
            {
                case NRF_DRV_TWI_XFER_TX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXTX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                    twi_rx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXRX:
                    twi_rx_done = true;
                    break;
                default:
                    break;
            }
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            break;
        default:
            break;
    }
}

uint32_t nrf_drv_bno_init(void)
{
    uint32_t err_code;
    
    const nrf_drv_twi_config_t twi_bno_config_0 = {
       .scl                = BNO_TWI_SCL_PIN_0,
       .sda                = BNO_TWI_SDA_PIN_0,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = 0,
       .clear_bus_init     = false
    };
		
    err_code = nrf_drv_twi_init(&m_twi_instance_0, &twi_bno_config_0, nrf_drv_bno_twi_event_handler, NULL);
		
    if(err_code != NRF_SUCCESS)
		{
			return err_code;
		}
    
    nrf_drv_twi_enable(&m_twi_instance_0);
		
		return NRF_SUCCESS;
}

uint32_t nrf_drv_bno_read_registers(uint8_t addr, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    uint32_t timeout = BNO_TWI_TIMEOUT;
	
    do{
				 err_code = nrf_drv_twi_rx(&m_twi_instance_0, addr, p_data, length);
						 
				 if(err_code != NRF_SUCCESS) continue;
				
				 timeout = BNO_TWI_TIMEOUT;
				 while((!twi_rx_done) && --timeout);
				 if(!timeout) continue;
				 twi_rx_done = false;
				 break;
	  }
		while(true);

    return err_code;
}

bool bno080_receivePacket(bno080 * bno)
{
	uint32_t err_code;
		
	if(bno->no == 0)
			err_code = nrf_drv_bno_read_registers(BNO080_ADDR0, bno->p_data, 4);
	else
			err_code = nrf_drv_bno_read_registers(BNO080_ADDR1, bno->p_data, 4);

  if(err_code != NRF_SUCCESS) return false;
	
	//Get the first four bytes, aka the packet header
	uint8_t packetLSB = bno->p_data[0];
	uint8_t packetMSB = bno->p_data[1];
	uint8_t channelNumber = bno->p_data[2];
	uint8_t sequenceNumber = bno->p_data[3]; //Not sure if we need to store this or not

	//Store the header info.
	bno->shtpHeader[0] = packetLSB;
	bno->shtpHeader[1] = packetMSB;
	bno->shtpHeader[2] = channelNumber;
	bno->shtpHeader[3] = sequenceNumber;

	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)packetMSB << 8 | packetLSB);
	dataLength &= ~(1 << 15); //Clear the MSbit.
	
	if (dataLength == 0)
	{
		//Packet is empty
		return (false); //All done
	}
	dataLength -= 4; //Remove the header bytes from the data count

	bno080_getData(dataLength, bno);
	
	return (true); //done
}

bool bno080_getData(uint16_t bytesRemaining, bno080 * bno)
{
	uint32_t err_code;
	
	if(bytesRemaining > 124) bytesRemaining = 124;
	
	if(bno->no == 0)
			err_code = nrf_drv_bno_read_registers(BNO080_ADDR0, bno->data, bytesRemaining+4);
	else
			err_code = nrf_drv_bno_read_registers(BNO080_ADDR1, bno->data, bytesRemaining+4);
	if(err_code != NRF_SUCCESS) return err_code;
	
	uint16_t dataSpot = 0; //Start at the beginning of shtpData array
	
	for (uint16_t x = 0; x < MAX_PACKET_SIZE-4; x++)
	{	
		uint8_t incoming = bno->data[4+x];
		if (dataSpot < MAX_PACKET_SIZE)
		{
			bno->shtpData[dataSpot++] = incoming; //Store data into the shtpData array
		}
		else
		{
			//Do nothing with the data
		}
	}

	return (true); //Done!
}

void bno080_softReset(bno080 * bno)
{
	bno->shtpData[0] = 1; //Reset

	//Attempt to start communication with sensor
	bno080_sendPacket(CHANNEL_EXECUTABLE, 1, bno); //Transmit packet on channel 1, 1 byte
	
	//Read all incoming data and flush it
	nrf_delay_ms(50);
	while (bno080_receivePacket(bno) == true);
	
	nrf_delay_ms(50);
	while (bno080_receivePacket(bno) == true);
}


bool bno080_sendPacket(uint8_t channelNumber, uint8_t dataLength, bno080 * bno)
{	
	uint32_t err_code;
	uint32_t timeout = BNO_TWI_TIMEOUT;
	uint8_t packetLength = dataLength + 4; //Add four bytes for the header

	//Send the 4 byte packet header	
	uint8_t p_data[4] = {packetLength & 0xFF,packetLength >> 8,channelNumber,bno->sequenceNumber[channelNumber]++};
	
	memcpy(bno->buffer, p_data, 4);
	
	//Send the user's data packet
	for (uint8_t i = 0; i < dataLength; i++)
	{
			memcpy(bno->buffer+4+i, &(bno->shtpData[i]), 1);
	}

	 do{
				if(bno->no == 0)
						err_code = nrf_drv_twi_tx(&m_twi_instance_0, BNO080_ADDR0, bno->buffer, packetLength, false);
				else
						err_code = nrf_drv_twi_tx(&m_twi_instance_0, BNO080_ADDR1, bno->buffer, packetLength, false);
				
				if(err_code != NRF_SUCCESS) continue;
	
				timeout = BNO_TWI_TIMEOUT;
				while((!twi_tx_done) && --timeout){};
				if(!timeout) continue;
				twi_tx_done = false;
				break;
	  }
		while(true);	
	
	return true;
}


void bno080_setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, bno080 * bno)
{
	long microsBetweenReports = (long)timeBetweenReports * 1000L;

	bno->shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;	 //Set feature command. Reference page 55
	bno->shtpData[1] = reportID;							   //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	bno->shtpData[2] = 0;								   //Feature flags
	bno->shtpData[3] = 0;								   //Change sensitivity (LSB)
	bno->shtpData[4] = 0;								   //Change sensitivity (MSB)
	bno->shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	bno->shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
	bno->shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
	bno->shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
	bno->shtpData[9] = 0;								   //Batch Interval (LSB)
	bno->shtpData[10] = 0;								   //Batch Interval
	bno->shtpData[11] = 0;								   //Batch Interval
	bno->shtpData[12] = 0;								   //Batch Interval (MSB)
	bno->shtpData[13] = 0;	   //Sensor-specific config (LSB)
	bno->shtpData[14] = 0;	   //Sensor-specific config
	bno->shtpData[15] = 0;	  //Sensor-specific config
	bno->shtpData[16] = 0;	  //Sensor-specific config (MSB)

	//Transmit packet on channel 2, 17 bytes
	bno080_sendPacket(CHANNEL_CONTROL, 17, bno);
}


void bno080_enableRotationVector(uint16_t timeBetweenReports, bno080 * bno)
{
	bno080_setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports, bno);
}


bool bno080_dataAvailable(bno080 * bno)
{
		if (bno080_receivePacket(bno) == true)
				{
				//Check to see if this packet is a sensor reporting its data to us
				if (bno->shtpHeader[2] == CHANNEL_REPORTS && bno->shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
				{
					bno080_parseInputReport(bno); //This will update the rawAccelX, etc variables depending on which feature report is found
					return (true);
				}
				else if (bno->shtpHeader[2] == CHANNEL_CONTROL)
				{
					bno080_parseCommandReport(bno); //This will update responses to commands, calibrationStatus, etc.
					return (true);
				}
				else if(bno->shtpHeader[2] == CHANNEL_GYRO)
				{
					bno080_parseInputReport(bno); //This will update the rawAccelX, etc variables depending on which feature report is found
					return (true);
				}
		}
		return (false);
}


void bno080_parseInputReport(bno080 * bno)
{
	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)bno->shtpHeader[1] << 8 | bno->shtpHeader[0]);
	//Clear the MSbit. This bit indicates if this package is a continuation of the last.
	dataLength &= ~(1 << 15); 
	//Ignore it for now. TODO catch this as an error and exit

	dataLength -= 4; //Remove the header bytes from the data count

	bno->timeStamp = ((uint32_t)bno->shtpData[4] << (8 * 3)) | 
									 ((uint32_t)bno->shtpData[3] << (8 * 2)) | 
	                 ((uint32_t)bno->shtpData[2] << (8 * 1)) | 
	                 ((uint32_t)bno->shtpData[1] << (8 * 0));

	// The gyro-integrated input reports are sent via the special gyro channel 
	// and do no include the usual ID, sequence, and status fields
	if(bno->shtpHeader[2] == CHANNEL_GYRO) {
		bno->rawQuatI = (uint16_t)bno->shtpData[1] << 8 | bno->shtpData[0];
		bno->rawQuatJ = (uint16_t)bno->shtpData[3] << 8 | bno->shtpData[2];
		bno->rawQuatK = (uint16_t)bno->shtpData[5] << 8 | bno->shtpData[4];
		bno->rawQuatReal = (uint16_t)bno->shtpData[7] << 8 | bno->shtpData[6];
		bno->rawFastGyroX = (uint16_t)bno->shtpData[9] << 8 | bno->shtpData[8];
		bno->rawFastGyroY = (uint16_t)bno->shtpData[11] << 8 | bno->shtpData[10];
		bno->rawFastGyroZ = (uint16_t)bno->shtpData[13] << 8 | bno->shtpData[12];

		return;
	}

	
	uint8_t status = bno->shtpData[5 + 2] & 0x03; //Get status bits
	uint16_t data1 = (uint16_t)bno->shtpData[5 + 5] << 8 | bno->shtpData[5 + 4];
	uint16_t data2 = (uint16_t)bno->shtpData[5 + 7] << 8 | bno->shtpData[5 + 6];
	uint16_t data3 = (uint16_t)bno->shtpData[5 + 9] << 8 | bno->shtpData[5 + 8];
	uint16_t data4 = 0;
	uint16_t data5 = 0; // We would need to change this to uin32_t to 
										  // capture time stamp value on Raw Accel/Gyro/Mag reports
	
	if (dataLength - 5 > 9)
	{
		data4 = (uint16_t)bno->shtpData[5 + 11] << 8 | bno->shtpData[5 + 10];
	}
	if (dataLength - 5 > 11)
	{
		data5 = (uint16_t)bno->shtpData[5 + 13] << 8 | bno->shtpData[5 + 12];
	}

	//Store these generic values to their proper global variable
	if (bno->shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR)
	{
		bno->quatAccuracy = status;
		bno->rawQuatI = data1;
		bno->rawQuatJ = data2;
		bno->rawQuatK = data3;
		bno->rawQuatReal = data4;

		//Only available on rotation vector and ar/vr stabilized rotation vector,
		// not game rot vector and not ar/vr stabilized rotation vector
		bno->rawQuatRadianAccuracy = data5;
	}
	else if (bno->shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION)
	{
		bno->accelLinAccuracy = status;
		bno->rawLinAccelX = data1;
		bno->rawLinAccelY = data2;
		bno->rawLinAccelZ = data3;
	}
	else if (bno->shtpData[5] == SENSOR_REPORTID_GYROSCOPE)
	{
		bno->gyroAccuracy = status;
		bno->rawGyroX = data1;
		bno->rawGyroY = data2;
		bno->rawGyroZ = data3;
	}
	else if (bno->shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD)
	{
		bno->magAccuracy = status;
		bno->rawMagX = data1;
		bno->rawMagY = data2;
		bno->rawMagZ = data3;
	}
	else if (bno->shtpData[5] == SENSOR_REPORTID_ACCELEROMETER ||
		bno->shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
		bno->shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR ||
		bno->shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR)
	{
		bno->accelAccuracy = status;
		bno->rawAccelX = data1;
		bno->rawAccelY = data2;
		bno->rawAccelZ = data3;
	}
	else if (bno->shtpData[5] == SENSOR_REPORTID_STEP_COUNTER)
	{
		bno->stepCount = data3; //Bytes 8/9
	}
	else if (bno->shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER)
	{
		bno->stabilityClassifier = bno->shtpData[5 + 4]; //Byte 4 only
	}
	else if (bno->shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER)
	{
		bno->activityClassifier = bno->shtpData[5 + 5]; //Most likely state

		//Load activity classification confidences into the array
		for (uint8_t x = 0; x < 9; x++)					   //Hardcoded to max of 9. TODO - bring in array size
			bno->activityConfidences[x] = bno->shtpData[5 + 6 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
	}
	else if (bno->shtpData[5] == SENSOR_REPORTID_RAW_ACCELEROMETER)
	{
		bno->memsRawAccelX = data1;
		bno->memsRawAccelY = data2;
		bno->memsRawAccelZ = data3;
	}
	else if (bno->shtpData[5] == SENSOR_REPORTID_RAW_GYROSCOPE)
	{
		bno->memsRawGyroX = data1;
		bno->memsRawGyroY = data2;
		bno->memsRawGyroZ = data3;
	}
	else if (bno->shtpData[5] == SENSOR_REPORTID_RAW_MAGNETOMETER)
	{
		bno->memsRawMagX = data1;
		bno->memsRawMagY = data2;
		bno->memsRawMagZ = data3;
	}
	else if (bno->shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = bno->shtpData[5 + 2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			bno->calibrationStatus = bno->shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
		}
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
	}
}

void bno080_parseCommandReport(bno080 * bno)
{
		if (bno->shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
		{
				//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
				uint8_t command = bno->shtpData[2]; //This is the Command byte of the response

				if (command == COMMAND_ME_CALIBRATE)
				{
					bno->calibrationStatus = bno->shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
				}
		}
		else
		{
				//This sensor report ID is unhandled.
				//See reference manual to add additional feature reports as needed
		}
}

float bno080_qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
		float qFloat = fixedPointValue;
		qFloat *= pow(2, qPoint * -1);
		return (qFloat);
}

float bno080_getQuatI(bno080 * bno)
{
		float quat = bno080_qToFloat(bno->rawQuatI, rotationVector_Q1);
		return (quat);
}

float bno080_getQuatJ(bno080 * bno)
{
		float quat = bno080_qToFloat(bno->rawQuatJ, rotationVector_Q1);
		return (quat);
}

float bno080_getQuatK(bno080 * bno)
{
		float quat = bno080_qToFloat(bno->rawQuatK, rotationVector_Q1);
		return (quat);
}

float bno080_getQuatReal(bno080 * bno)
{
		float quat = bno080_qToFloat(bno->rawQuatReal, rotationVector_Q1);
		return (quat);
}

float bno080_getQuatRadianAccuracy(bno080 * bno)
{
		float quat = bno080_qToFloat(bno->rawQuatRadianAccuracy, rotationVectorAccuracy_Q1);
		return (quat);
}
