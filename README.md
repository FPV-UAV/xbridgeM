# xbridgeM

xbridgeM - Read the Freestyle Libre sensor and send the BG readings to phone application using the xBridge2 protocol over Bluetooth Low Energy (BLE).

This project is is a modification/extension of the Lbridge project from Keencave (https://github.com/keencave/LBridge). 
The code is running on the Simblee/RFDuino hardware platform - T-mini(Simblee) and T-I/T-II(RFduino).
Hardware details - in /Hardware folder. More about Devices - https://mtransmiter.pl and FB group T-Mini Users: https://www.facebook.com/groups/188288191764699


__Please note, that xbridgeM code is NOT maintained by Abbott. It is an experimental DIY project. You will built/buy your own individual hardware device. So you are responsible yourself for what you have. This is not a medical device. Dont make any medical decisions based on the results as they can be wrong!__

## xBridge2 protocol implementation:

### 1. Records structures

#### A. __Data Packet - Device to App.  Sends the Dexcom transmitter data, and the device battery volts.__
```
typedef struct 	 __attribute__((packed))
{
  unsigned char size;           // size of the packet.
  unsigned char cmd_code;       // code for this data packet.  Always 00 for a Dexcom data packet.
  unsigned long raw;            // "raw" BGL value
  unsigned long filtered;       // "filtered" BGL value (now - same as "raw")
  unsigned char dex_battery;    // battery value <204..215>
  unsigned char my_battery;     // xBridge battery value (%)
  unsigned long dex_src_id;     // raw TXID of the Dexcom Transmitter
  unsigned long delay;          // delay of raw reading to the current runtime
  unsigned char function;       // Byte representing the xBridge code funcitonality.  01 = this level.
} nRawRecord;
```

#### B. __Request data packet - App to Device. Request for various data from sensor.
```
typedef struct __attribute__((__packed__)) 
{
  uint8_t   size;                 // size of the packet (12)
  uint8_t   cmd_code;             // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code;             // Subtype code. 0x00 for a request packet.
  uint8_t   requested_sub_code;   // 0x00 xbridge Data Packet, 0x01 status packet, 0x02  last 15 minutes readings part1, 
                                  // 0x03 last 15 minutes readings part2, 0x04 last 8 hours readings part1, 0x05 last 8 hours readings
                                  // part2, 0x06 last 8 hours readings part3, 0x07 last 8 hours readings part4
  uint32_t  dex_src_id;           // raw TXID of the requested Dexcom Transmitter
  uint32_t  timestamp;            // current timestamp
} _DataRequestPacket;
```

#### C. __Status data packet - Device to App. 
```
typedef struct __attribute__((__packed__)) 
{
  uint8_t   size;                 // size of the packet (19)
  uint8_t   cmd_code;             // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code;             // Subtype code. 0x01 for a status packet.
  uint8_t   sensorStatusByte;     // sensor status - description below
  char      sensorSN[11];         // ie. 0M00000RG3D
  uint16_t  minutesSinceStart;    // number of minutes since sensor activation
  uint8_t   temperatureC;         // Celsius temerature of the Device
  uint8_t   rssi;                 // BLE reception signal level [dBm]
} _StatusPacket;
```
Sensor status codes:
- 01 - not yest started
- 02 - starting (first 60 minutes after activation)
- 03 - ready (normal 14-days work)
- 04 - expired (just after end of 14-days period, still working for approx next 12h)
- 05 - shutdown - sensor stopped 
- 06 - failure - sensor failure during 14-days period
- any other value - unknown state

#### D. __Last 15-minutes period data part I (BG values every 1 minute)  - Device to App. 
```
typedef struct __attribute__((__packed__)) 
{
  uint8_t   size;       // size of the packet (19)
  uint8_t   cmd_code;   // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code;   // Subtype code. 0x02 for a last quarter hour packet part1.
  uint16_t  trend[8];   // BG readings in the current minute, previous minute, 2, 3, 4, 5, 6, 7 minutes ago
} _QuarterPacket1;
```

#### E. __Last 15-minutes period data part II (BG values every 1 minute)  - Device to App. 
```
typedef struct __attribute__((__packed__)) 
{
  uint8_t   size;       // size of the packet (19)
  uint8_t   cmd_code;   // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code;   // Subtype code. 0x03 for a last quarter hour packet part2.
  uint16_t  trend[8];   // BG readings 8, 9, 10, 11, 12, 13, 14, 15 minutes ago
} _QuarterPacket2;
```

#### F. __Last 8h period data part I (BG values every 15 minute)  - Device to App. 
```
typedef struct __attribute__((__packed__)) 
{
  uint8_t   size;       // size of the packet (19)
  uint8_t   cmd_code;   // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code;   // Subtype code. 0x04 for a last 8 hour, packet part1.
  uint16_t  history[8]; // BG readings 15, 30, 45, 60, 75, 90, 105, 120 minutes ago
} _HistoryPacket1;
```

#### G. __Last 8h period data part II (BG values every 15 minute)  - Device to App. 
```
typedef struct __attribute__((__packed__)) 
{
  uint8_t   size;       // size of the packet (19)
  uint8_t   cmd_code;   // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code;   // Subtype code. 0x05 for a last 8 hour, packet part1.
  uint16_t  history[8]; // BG readings 135, 150, 165, 180, 195, 210, 225, 240 minutes ago
} _HistoryPacket2;
```

#### H. __Last 8h period data part III (BG values every 15 minute)  - Device to App. 
```
typedef struct __attribute__((__packed__)) 
{
  uint8_t   size;       // size of the packet (19)
  uint8_t   cmd_code;   // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code;   // Subtype code. 0x06 for a last 8 hour, packet part1.
  uint16_t  history[8]; // BG readings 255, 270, 285, 300, 315, 330, 345, 360 minutes ago
} _HistoryPacket3;
```

#### I. __Last 8h period data part IV (BG values every 15 minute)  - Device to App. 
```
typedef struct __attribute__((__packed__)) 
{
  uint8_t   size;       // size of the packet (19)
  uint8_t   cmd_code;   // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code;   // Subtype code. 0x07 for a last 8 hour, packet part1.
  uint16_t  history[8]; // BG readings 375, 390, 405, 420, 435, 450, 465, 480 minutes ago
} _HistoryPacket4;
```

#### J. __TXID packet  - App to Device. Sends the TXID the App wants the Device to filter on. In response to a Data packet being wrong.
```
typedef struct __attribute__((__packed__)) 
{
  uint8_t len;            // Length of the packet.
  uint8_t cmd_code;       // Packet Type (01 means TXID packet).
  uint32_t  dex_src_id;   // Dexcom encoded TXID
} _txid_packet;
```

### 2. Communication flow.

#### A. Normal (automatic, non-interactive) work: 

After power-on, is init-phase: initialization of local variables, structures, hardware modules, etc.
After init phase - device start loop-phase. First 10 loops have period of 1 minute (due to easy setup/init in phone application - short time for user). Than loop period is changed to 5 minutes for all further work time.

Every loop (every 5 minutes - or 1 minute in first 10 cycles) Device performs as follow:

0. battery power level check (voltage) - if is below minimum - go to point 5 (as protection from deep discharge).
1. reading Libre sensor via NFC, than prepare received data and format it according to _DataPacket structure_ (point 1.A above). 
2. filling local copy of Libre sensor memory (shadowFram) with data received from sensor. Local/shadow copy can store up to 12h of data.
3. storing prepared message in local Queue. Queue working on first-in - first-out basis. 
4. if detect __connected to Application__ state - transmit all data from Queue via BLE. 
5. goto sleep state (power save) until end of loop period than start loop from point 0.

BLE Transmision is transmision-with-ACK. When Application receive each message - confirming it to Device by __ACK message__ - two byte __0x02 0xF0__. When within 10 seconds Device do not receive __ACK message__ - re-sent current message one time (and wait for ACK). If still ACK is not received within next 10 seconds - go sleep for end of loop period.

__Device BLE setup:__
- device name:      __xbridgeM__
- advertisement:    __rfduino__
- main UUID:                                        __0000ffe0-0000-1000-8000-00805f9b34fb__ (primary service)
- characteristic for transmission (Device send):    __0000ffe1-0000-1000-8000-00805f9b34fb__
- characteristic for reception (Device listen to):  __0000ffe2-0000-1000-8000-00805f9b34fb__
- advertisement Interval: __500 miliseconds__

#### B. Non-automatic (interactive) work.

Anytime Application need some current and/or historical data from Device (sensor) can initiate communication by sending request to Device. Request message is in the same format as "reply" message, but with empty data. 

As an extension - there is one special request - Application can sent simply __"R"__ letter to ask for current "on-demand" BG value. As a reply - Device immeediatelly read sensor and sending main __DataPacket__ (point 1.A above).


# ToDo:
- packets B-J (only A implemented so far)
- implementation of interactive work (point 2.B) - except "R" command - its work already.
- code cleaning 
- more tests and power-save optimalizations






