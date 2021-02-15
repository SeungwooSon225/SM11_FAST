/*
  This is a library written for the BNO080
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  The BNO080 IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO080 and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.3

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef __BNO080_H__
#else

#define __BNO080_H__
#include "sdk_common.h"

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)

typedef struct bno080
{
		uint8_t no;			// index of IMU (we use 2 IMUs)
	
		uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
		uint8_t shtpData[MAX_PACKET_SIZE];
		uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
	
		uint8_t p_data[4];
		uint8_t data[128];
		uint8_t buffer[128];
	
		uint8_t calibrationStatus;							  //Byte R0 of ME Calibration Response
		uint32_t timeStamp;
		uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
		uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
		uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
		uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
		uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
		uint16_t stepCount;
		uint8_t stabilityClassifier;
		uint8_t activityClassifier;
		uint8_t* activityConfidences;						  //Array that store the confidences of the 9 possible activities
		uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ; //Raw readings from MEMS sensor
		uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;	//Raw readings from MEMS sensor
		uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;		  //Raw readings from MEMS sensor
			
		uint8_t sequenceNumber[6]; //There are 6 com channels. Each channel has its own seqnum
		uint8_t commandSequenceNumber;				//Commands have a seqNum as well. These are inside command packet, 
																					//the header uses its own seqNum per channel
		uint32_t metaData[MAX_METADATA_SIZE];			//There is more than 10 words in a metadata record but we'll stop at Q point 3
		
}bno080;


bool bno080_begin(bno080 * bno); //By default use the default I2C addres, and use Wire port, and don't declare an INT pin
void bno080_softReset(bno080 * bno);	  //Try to reset the IMU via software
bool bno080_receivePacket(bno080 * bno);
bool bno080_sendPacket(uint8_t channelNumber, uint8_t dataLength, bno080 * bno);
bool bno080_getData(uint16_t bytesRemaining, bno080 * bno); //Given a number of bytes, send the requests in I2C_BUFFER_LENGTH chunks
uint32_t nrf_drv_bno_init(void);
void bno080_setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, bno080 * bno);
void bno080_enableRotationVector(uint16_t timeBetweenReports, bno080 * bno);
bool bno080_dataAvailable(bno080 * bno);
void bno080_parseInputReport(bno080 * bno);   //Parse sensor readings out of report
void bno080_parseCommandReport(bno080 * bno); //Parse command responses out of report
float bno080_getQuatI(bno080 * bno);
float bno080_getQuatJ(bno080 * bno);
float bno080_getQuatK(bno080 * bno);
float bno080_getQuatReal(bno080 * bno);
float bno080_getQuatRadianAccuracy(bno080 * bno);
float bno080_qToFloat(int16_t fixedPointValue, uint8_t qPoint); //Given a Q value, converts fixed point floating to regular floating point number


#endif
