/*******************************************************************************************************************************************
* xridgeM for RFduino / Simblee
*
* BLE Transmitter for read blood glucose from the Freestyle Libre Sensor via NFC and transfer to
* xDrip+ via BLE using the xbridge2 protocol (xbridge wixel)

* Supports backfilling of older BG readings which were not transfered at the time of reading.
*
* Code based on on the work of @keencave, @UPetersen, @jstevensog, @savek-cc and @bertrooode
*
* Marek Macner, 04/2018
*
* Main changes from @keencave LBridge V.09.15:
*										- only one project selector: RFD => RFDuino/Simblee
*										- debug options: DEBUG, SHOW_BLESTATECHANGE, SERIALOFF
*										- permanent implementation: USE_SPIKE_FILTER, USE_XBRIDGE2, USE_SHADOWFRAM, SHORTSTARTCYCLE, USE_WDT																	
*										- not implemented options: USE_DEAD_SENSOR, SIM_MINUTES_UPCOUNT, BLE_REAL_LEN, SEND_LIMITTER_SA
*										- all parameters transmferred data structures changed to static data
*
********************************************************************************************************************************************/

//#define RFD                     // uncomment for RFduino platform, otherwise Simblee platform will be used

//#define DEBUG                   // uncomment to have verbose debug output
//#define SHOW_BLESTATECHANGE     // uncomment to have serial BLE connect/disconnect messages (see also #define SERIALOFF)
#define SERIALOFF               // switch off Serial interface during ULP for power saving

/* ****************************** library includes and compiler addons ********************************** */

#ifdef RFD
	#include <RFduinoBLE.h>
#else RFD
	#include <SimbleeBLE.h>
	#include <ota_bootloader.h>
#endif RFD

#include <SPI.h>
#include <Stream.h>
#include <Memory.h>
#include <itoa.h>

#define  str(x)   xstr(x) // double level of indirection required to get gcc
#define  xstr(x)  #x      // to apply the stringizing operator correctly

#ifdef __arm__				// should use uinstd.h to define sbrk but Due causes a conflict
	extern "C" char* sbrk(int incr);
#else  // __arm__
	extern char *__brkval;
#endif  // __arm__

/* ****************************** #tags definitions ********************************** */

/* *** BLE and central seetings *** */

#define UARTDELAY 2000						// delay after switching serial interface on/off

#define DXQUEUESIZE (9*12)					// 12 h of queue (remember and send the last x hours when reconnected via BLE)

#define LB_NAME   "xbridgeM"				// dont change "xbridge", space for 1 char left to indicate different versions
#define LB_ADVERT "rfduino"					// dont change "rfduino", length of device name and advertisement <=15!!!

#define LB_VERSION "V0.1"					// program version
#define LB_MINOR_VERSION ".01"				// indicates minor version
#define LB_DATETIME "180406_1100"			// date_time

#define SPIKE_HEIGHT 40						// minimum delta to be a spike

#ifdef RFD
	#define MAX_VOLTAGE 3600				// adjust voltage measurement to have a wider rrange
	#define MIN_VOLTAGE 2300				// minimum voltage where BLE will work properly (@bertrooode)
#else RFD
	#define MAX_VOLTAGE 3800				// maximum voltage measurement for Simblee with LiPo 
	#define MIN_VOLTAGE 2600				// minimum voltage where BLE and NFC will work properly 
#endif RFD

/* *** hardware SPI pins seetings *** */

#define PIN_SPI_SCK   4
#define PIN_SPI_MOSI  5
#define PIN_SPI_MISO  3
#define PIN_SPI_SS    6
#define PIN_IRQ       2
const int SS_PIN = SS;				// Slave Select pin, uses standard wiring from variant.h
									// const int IRQ_PIN = 2;  // IRQ/DIN pin used for wake-up pulse

/* *** NFC section seetings *** */
#define ALL_BYTES 0x1007
#define IDN_DATA 0x2001
#define SYSTEM_INFORMATION_DATA 0x2002
#define BATTERY_DATA 0x2005

/* CR95HF Commands */
#define IDN               0x01			// identification number of CR95HF
#define SELECT_PROTOCOL   0x02			// select protocol
#define POLL              0x03			// poll
#define SENDRECEIVE       0x04			// send and receive data (most commonly used)
#define READ              0x05			// read values from registers internal to CR95HF
#define WRITE             0x06			// write values to registers internal to CR95HF
#define ECHO              0x55

// send receive commands for ISO/IEC 15693 protocol
#define INVENTORY					0x01	// receives information about tags in range
#define STAY_QUIET					0x02	// selected unit will not send back a response
#define READ_BLOCK					0x20	// read single block of memory from RF tag
#define WRITE_BLOCK					0x21	// write single block to memory of RF tag
#define LOCK_BLOCK					0x22	// permanently locks a block of memory on RF tag
#define READ_BLOCKS					0x23	// reads multiple blocks of memory from RF tag
#define WRITE_BLOCKS				0x24	// writes multiple blocks of memory to RF tag
#define SELECT						0x25	// used to select a specific tag for communication via the uid
#define RESET_TO_READY				0x26	// resets RF tag to ready state
#define WRITE_AFI					0x27	// writes application family identifier to RF tag
#define LOCK_AFI					0x28	// permanently locks application family identifier
#define WRITE_DSFID					0x29	// writes data storage format identifier to RF tag
#define LOCK_DSFID					0x2A	// permanentlylocks data storage format identifier
#define GET_SYSTEM_INFORMATION		0x2B	// gets information from RF tag that includes memory
											// block size in bytes and number of memory blocks
#define GET_BLOCKS_SECURITY_STATUS  0x2C

#define BLEBUFLEN 80                // BLE input buffer
											
#define DEXBRIDGE_PROTO_LEVEL	(0x01)		// defines the xBridge protocol functional level.  Sent in each packet as the last byte.
											
#define COMMAND_MAXLEN			80			//define the maximum command string length for USB commands.

#define SNLEN		20				// length of decoded sensor number
#define SSBYTE		20              // length of sensorStatusByte message

#define MAXSTACK		0x20003FFF			// stack begin at RFduino
#define STACKWARNLEVEL	300					// maximum stack size (1k on RFduino)
#define FKTSTACK		20					// entries for calling history
#define FNMLEN			20

#define FILL8H (8*12)     // fillup 8 h - the length of the 8 hour Libre sensor FRAM history over all

#define NFCTIMEOUT 500

#define NFCDEVICEID 20      // length of NFC device ID (12)

#define MAXBLELEN 20

#define TIMEOF8HINMS ((long)(8L*12L*5L*60L*1000L))

/* ****************************** data structures definitions ********************************** */

//structure of a USB/BLE command
typedef struct    __attribute__((packed))
{
	unsigned char commandBuffer[COMMAND_MAXLEN];
	unsigned char nCurReadPos;
} t_command_buff;
 
typedef struct  __attribute__((packed))
{
	unsigned long raw;
	unsigned long ms;
} Dexcom_packet;

typedef struct  __attribute__((packed))
{
	volatile unsigned char read;
	volatile unsigned char write;
	Dexcom_packet buffer[DXQUEUESIZE];
} Dexcom_fifo;


typedef struct 	 __attribute__((packed))	// structure of a raw record we will send in a xbridge2 packet
{
	unsigned char size;						// size of the packet.
	unsigned char cmd_code;					// code for this data packet.  Always 00 for a Dexcom data packet.
	unsigned long raw;						// "raw" BGL value. ??? use unfiltered NFC readings here?
	unsigned long filtered;					// "filtered" BGL value
	unsigned char dex_battery;				// battery value
	unsigned char my_battery;				// xBridge battery value
	unsigned long dex_src_id;				// raw TXID of the Dexcom Transmitter
	unsigned long delay;					// delay of raw reading to the current runtime
	unsigned char function;					// Byte representing the xBridge code funcitonality.  01 = this level.
} nRawRecord;

typedef struct  __attribute__((packed))
{
	uint8_t resultCode;
	uint8_t deviceID[13];
	uint8_t romCRC[2];
} IDNDataType;

typedef struct  __attribute__((packed))
{
	uint8_t uid[8];
	uint8_t resultCode;
	uint8_t responseFlags;
	uint8_t infoFlags;
	uint8_t errorCode;
	char *sensorSN;
  String sSN;
} SystemInformationDataType;

typedef struct  __attribute__((packed))
{
	bool sensorDataOK;
	char *sensorSN;
	byte   sensorStatusByte;
	char sensorStatusString[SSBYTE];
	byte  nextTrend;
	byte  nextHistory;
	uint16_t minutesSinceStart;
	uint16_t minutesHistoryOffset;
	uint16_t trend[16];
	float trendTemperature[16];
	uint16_t history[32];
	float historyTemperature[32];
  String sSN;
} SensorDataDataType;

typedef struct  __attribute__((packed)) 
{
	uint8_t uid[8];                  // FreeStyle Libre serial number
	uint8_t oldUid[8];
	uint8_t nextTrend;               // number of next trend block to read
	uint8_t oldNextTrend;
	uint8_t nextHistory;             // number of next history block to read
	uint8_t oldNextHistory;
	uint16_t minutesSinceStart;      // minutes since start of sensor
	uint16_t oldMinutesSinceStart;
	byte fram[344];                  // buffer for Freestyle Libre FRAM data
	byte resultCodes[43];            // result codes from nfc read single block command
} Sensor;

typedef struct  __attribute__((packed))
{
	long voltage;
	int voltagePercent;
	double temperatureC;
	double temperatureF;
	double rssi;
} SoCDataType;

/* ****************************** variables definitions ********************************** */

uint16_t crc16table[256] =
{ 0, 4489, 8978, 12955, 17956, 22445, 25910, 29887, 35912, 40385,
44890, 48851, 51820, 56293, 59774, 63735, 4225, 264, 13203, 8730,
22181, 18220, 30135, 25662, 40137, 36160, 49115, 44626, 56045, 52068,
63999, 59510, 8450, 12427, 528, 5017, 26406, 30383, 17460, 21949,
44362, 48323, 36440, 40913, 60270, 64231, 51324, 55797, 12675, 8202,
4753, 792, 30631, 26158, 21685, 17724, 48587, 44098, 40665, 36688,
64495, 60006, 55549, 51572, 16900, 21389, 24854, 28831, 1056, 5545,
10034, 14011, 52812, 57285, 60766, 64727, 34920, 39393, 43898, 47859,
21125, 17164, 29079, 24606, 5281, 1320, 14259, 9786, 57037, 53060,
64991, 60502, 39145, 35168, 48123, 43634, 25350, 29327, 16404, 20893,
9506, 13483, 1584, 6073, 61262, 65223, 52316, 56789, 43370, 47331,
35448, 39921, 29575, 25102, 20629, 16668, 13731, 9258, 5809, 1848,
65487, 60998, 56541, 52564, 47595, 43106, 39673, 35696, 33800, 38273,
42778, 46739, 49708, 54181, 57662, 61623, 2112, 6601, 11090, 15067,
20068, 24557, 28022, 31999, 38025, 34048, 47003, 42514, 53933, 49956,
61887, 57398, 6337, 2376, 15315, 10842, 24293, 20332, 32247, 27774,
42250, 46211, 34328, 38801, 58158, 62119, 49212, 53685, 10562, 14539,
2640, 7129, 28518, 32495, 19572, 24061, 46475, 41986, 38553, 34576,
62383, 57894, 53437, 49460, 14787, 10314, 6865, 2904, 32743, 28270,
23797, 19836, 50700, 55173, 58654, 62615, 32808, 37281, 41786, 45747,
19012, 23501, 26966, 30943, 3168, 7657, 12146, 16123, 54925, 50948,
62879, 58390, 37033, 33056, 46011, 41522, 23237, 19276, 31191, 26718,
7393, 3432, 16371, 11898, 59150, 63111, 50204, 54677, 41258, 45219,
33336, 37809, 27462, 31439, 18516, 23005, 11618, 15595, 3696, 8185,
63375, 58886, 54429, 50452, 45483, 40994, 37561, 33584, 31687, 27214,
22741, 18780, 15843, 11370, 7921, 3960
};

char timestring[20];

int batteryPcnt = 0;                // battery capacity in %
unsigned long loop_cnt = 0;         // count the 5 mins loops
int ble_answer;                     // state counter, char from BLE reeived?

unsigned char bleBuf[BLEBUFLEN];
int bleBufRi = 0;                   // BLE read and write index
int bleBufWi = 0;
static boolean BTconnected = false;

unsigned long startMinutesSinceStart = 0;
unsigned long startMillisSinceStart = 0;
unsigned long minutesSinceProgramStart = 0;

uint16_t lastGlucose = 0;
uint16_t currentGlucose = 0;
boolean firstRun = 1;               // flag for spike filter operation

byte runPeriod = 1;                 // czas w minutach - init = 1 minute for first 10 loops
unsigned long time_loop_started = 0;

bool BatteryOK = false;

char TxBuffer[30];

byte NFCReady = 0;            // 0 - not initialized, 1 - initialized, no data, 2 - initialized, data OK

static boolean show_ble = 0;
static boolean got_ack = 0;               // indicates if we got an ack during the last do_services.
static unsigned long pkt_time = 0;        // time of last valid received glucose value
static unsigned long last_ble_send_time;  // beautifying BLE debug output
static boolean crlf_printed = 0;          // beautifying BLE debug output

static t_command_buff command_buff;
static Dexcom_fifo Pkts;
static Dexcom_packet pPkt;

unsigned long dex_tx_id = 0xA5B1AE;     // TXID for xbridge2 protocol packets = "ABCDE"

static IDNDataType idnData;
static SystemInformationDataType systemInformationData;
static SensorDataDataType sensorData;
static char decodedSensorSN[15];

static byte sensorDataHeader[24];
static byte sensorDataBody[296];
static byte sensorDataFooter[24];

static byte resultBuffer[40];
static byte RXBuffer[400];           // receive buffer for shadow FRAM data
static byte dataBuffer[400];         // receive buffer for non shadow FRAM processing

static Sensor sensor;

static SoCDataType SoCData;            // store processor variables

boolean sreached = 0;

int minimumMemory = 8192;

char fktstack[FKTSTACK][FNMLEN];
unsigned int fktFree[FKTSTACK];
unsigned char fwind = 0;
unsigned char frind = 0;

/* ****************************** SETUP SECTION ********************************** */
void setup()
{

	#ifdef DEBUG
		show_ble = 1;
	#else
		show_ble = 0;
	#endif DEBUG

	Serial.begin(9600);							// init and open serial line	
	delay(UARTDELAY);							// time to settle, avoid serial ghost characters
	Serial.printf("\r\n============================================================================");

	#ifdef RFD
		print_state("LBridge starting (RFduino)");
	#else
		print_state("LBridge starting (Simblee)");
	#endif RFD
	Serial.print("\r\n\tBLE name: "); Serial.print(LB_NAME);
	Serial.print(", "); Serial.print(LB_VERSION); Serial.print(LB_MINOR_VERSION);
	Serial.print(" from "); Serial.print(LB_DATETIME);
	Serial.printf("\r\n\tRAM used: %d, Flash used: %d", ramUsed(), flashUsed());
	Serial.print("\r\n\tQueue size: "); Serial.print(DXQUEUESIZE);
	Serial.print(" ("); Serial.print(DXQUEUESIZE / 12); Serial.print(" h)");
	Serial.print("\r\n\tOptions:");
	Serial.printf("-spikeFilter%d", SPIKE_HEIGHT);
	Serial.print("-xbridge2");
	Serial.print("-shadowFRAM");
	Serial.print("-10cycles1min");
	#ifdef SERIALOFF
		Serial.print("-serialOff");
	#endif
	Serial.print("-WDT");

	getSoCData();
	printSoCData();

	runPeriod = 1;          // loop time is 1 min, only for system start
	print_state("system start with "); Serial.print(runPeriod); Serial.print(" min cycyle time");
	loop_cnt = 0;

	nfcInit();

	digitalWrite(PIN_SPI_SS, LOW);
	SPI.transfer(0x01);
	digitalWrite(PIN_SPI_SS, HIGH);
	delay(1);

	initSensor();      // initialize the sensor data structure for FRAM shadowing

	configWDT();              // wind up watch dog timer in case of RFduino freezing do a reset

	setupBluetoothConnection();

	// init xbridge queue
	Pkts.read = 0;
	Pkts.write = 0;
	// set timestamps in queue for inital backfilling
	initialFillupBackfillTimestamps();

	print_state("");
	Serial.printf("NFCReady = %d, BatOK = %d", NFCReady, BatteryOK);

	check_stack("#setup#");
	memAndStack("setup()");
	Serial.printf("\r\n============================================================================");
}

/* ****************************** MAIN LOOP SECTION ********************************** */
void loop()
{
	time_loop_started = mymillis();

	check_stack("#loop#");

	Serial.printf("\r\n%s loop #%u--RSSI:%f--%umV/%d%%--SocTemp:%fC--(%s,%s%s/%s,", \
		BTconnected ? "=== BLE" : "=== ...", loop_cnt, SoCData.rssi, SoCData.voltage, SoCData.voltagePercent, SoCData.temperatureC, \
		LB_NAME, LB_VERSION, LB_MINOR_VERSION, LB_DATETIME);
	#ifdef RFD
		Serial.print("RFduino) ===");
	#else
		Serial.print("Simblee) ===");
	#endif

	restartWDT();
	
	memAndStack("loop()");				// show memory for debug	
	getSoCData();						// display current data from SoC hadrware

	loop_cnt++;

	if ((loop_cnt > 10)) 
	{
		runPeriod = 5;
		print_state("switch back to normal cycle time "); Serial.print(runPeriod); Serial.print(" min");
	}
	
	Serial.flush();				// @bertroode: wait for serial output in process
	delay(100);

	if (BatteryOK) 
	{
		readAllData();
		if (NFCReady == 2) 
		{
			fillupMissingReadings(0);
			dataTransferBLE();
		}
		else 
		{ 
			print_state("no sensor data to transmit"); 
		}
	}
	else	// calculate sleep time ajusted to runPeriod cycle
	{ 
		#ifdef RFD
			RFDuinoBLE.end();
		#else
			SimbleeBLE.end();
		#endif
		print_state("low Battery - go sleep, BLE disabled - all activities off to prevent deep battery discharge"); 
	}		
	
	unsigned long sleep_time = (60000 * runPeriod) - (mymillis() - time_loop_started) % 60000;

	
	if (sleep_time > 60000 * 10) // sanity check
	{		
		print_state(" *** error in sleep time calculationm (sleep ");		// in case of error set to 5 mins fix
		Serial.print(sleep_time);
		Serial.print(" ms) set to 5 min fix");
		sleep_time = 5 * 60000L;
	}

	restartBtStack(0);

	print_state("loop #"); 
	Serial.print(loop_cnt - 1); 
	Serial.print(" - end - NFCReady = ");
	Serial.print(NFCReady); 
	Serial.print(", sleep for "); 
	Serial.print(sleep_time / 1000); 
	Serial.print(" s");

	
	Serial.flush();		// @bertroode: wait for serial output in process
	delay(100);

	#ifdef SERIALOFF
		Serial.end();         // reduce power consumption
		delay(UARTDELAY);
	#endif SERIALOFF

	#ifdef RFD
		RFduino_ULPDelay(sleep_time);
	#else
		Simblee_ULPDelay(sleep_time);
	#endif

	#ifdef SERIALOFF
		Serial.begin(9600);
		delay(UARTDELAY);
	#endif SERIALOFF

	// @bertroode: reset CR95HF to be sure processing is stable /
	digitalWrite(PIN_SPI_SS, LOW);
	SPI.transfer(0x01);
	digitalWrite(PIN_SPI_SS, HIGH);
	delay(1);
  

}
