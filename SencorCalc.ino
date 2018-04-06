void decodeSensor()
{
	check_stack("#ds#");

	for (int i = 0; i < 24; i++) sensorDataHeader[i] = dataBuffer[i];
	for (int i = 24; i < 320; i++) sensorDataBody[i - 24] = dataBuffer[i];
	for (int i = 320; i < 344; i++) sensorDataFooter[i - 320] = dataBuffer[i];
	decodeSensorHeader();
	decodeSensorBody();
	decodeSensorFooter();
	displaySensorData();
}

void apply_spike_filter(void)
{
	check_stack("#asf#");

	if (firstRun == 1) {
		firstRun = 0;
		lastGlucose = sensorData.trend[0];
	}
	else {
		lastGlucose = currentGlucose;
	}
	currentGlucose = sensorData.trend[0];

	if (((lastGlucose - currentGlucose) > SPIKE_HEIGHT) || ((currentGlucose - lastGlucose) > SPIKE_HEIGHT)) {
		print_state("spike detected: cG "); Serial.print(currentGlucose); Serial.print(", lG ");
		Serial.print(lastGlucose);
		currentGlucose = lastGlucose;
	}
}

void decodeSN(byte *data)
{
	byte uuid[8];
	char lookupTable[32] =
	{
		'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
		'A', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K', 'L',
		'M', 'N', 'P', 'Q', 'R', 'T', 'U', 'V', 'W', 'X',
		'Y', 'Z'
	};
	byte uuidShort[8];    // UUID from the sensor FRAM
	char binary[70];      // contains at most 8 x 8 digits in ASCII = 64
	char binS[10];        // binary byte in ASCII, 8 chars with leading ZEROs
	binS[0] = '\0';
	char bbyte[10];       // direct conversiopn result with itoa(), between 1 to 8 bytes
	int i;

	check_stack("#decSN#");

	for (int i = 2; i < 8; i++)
		uuidShort[i - 2] = data[i];
	uuidShort[6] = 0x00;
	uuidShort[7] = 0x00;

	// convert the raw ID to ASCII binary
	binary[0] = '\0';
	for (int i = 0; i < 8; i++)
	{
		itoa(uuidShort[i], bbyte, 2);
		int l = strlen(bbyte);
		if (l == 1) sprintf(binS, "%s%s", "0000000", bbyte);
		else if (l == 2) sprintf(binS, "%s%s", "000000", bbyte);
		else if (l == 3) sprintf(binS, "%s%s", "00000", bbyte);
		else if (l == 4) sprintf(binS, "%s%s", "0000", bbyte);
		else if (l == 5) sprintf(binS, "%s%s", "000", bbyte);
		else if (l == 6) sprintf(binS, "%s%s", "00", bbyte);
		else if (l == 7) sprintf(binS, "%s%s", "0", bbyte);
		else if (l == 8) strcpy(binS, bbyte);
		strcat(binary, binS);
	}
	strcpy(decodedSensorSN, "0");
	char pozS[5];
	for (i = 0; i < 10; i++)
	{
		for (int k = 0; k < 5; k++)
			pozS[k] = binary[(5 * i) + k];
		int value = (pozS[0] - '0') * 16 + (pozS[1] - '0') * 8 + (pozS[2] - '0') * 4 + (pozS[3] - '0') * 2 + (pozS[4] - '0') * 1;
		decodedSensorSN[i + 1] = lookupTable[value];
	}
	decodedSensorSN[i + 1] = '\0';        // append delemite
	return;
}

/*
* check the sensor status byte which indicates if the sensor is working or dead
*/
void decodeSensorHeader()
{
	check_stack("#dsh#");

	sensorData.sensorStatusByte = sensorDataHeader[4];
	//  print_state("decodeSensorHeader()"); 
	switch (sensorData.sensorStatusByte)
	{
	case 0x01:
		strcpy(sensorData.sensorStatusString, "not yet started");
		break;
	case 0x02:
		strcpy(sensorData.sensorStatusString, "starting");
		break;
	case 0x03:      // normal operation, 14 days and 12 h
		strcpy(sensorData.sensorStatusString, "READY");
		break;
	case 0x04:      // sensor operation is terminated, this status remains the next 12 h
		strcpy(sensorData.sensorStatusString, "EXPIRED");
		break;
	case 0x05:      // sensor operation is terminated, end status
		strcpy(sensorData.sensorStatusString, "SHUTDOWN");
		break;
	case 0x06:
		strcpy(sensorData.sensorStatusString, "failure");
		break;
	default:
		strcpy(sensorData.sensorStatusString, "unknown state");
		break;
	}
	Serial.printf(" - Sensor status: %s", sensorData.sensorStatusString);
}

/*
* work on the Libre sensor body data
* extract current BG value
*/
void decodeSensorBody()
{
	int j;
	//  print_state("decodeSensorBody() - ");
	byte pomiar[6];

	check_stack("#dsb#");

	// calculate block indices for trend/history
	sensorData.nextTrend = sensorDataBody[2];
	sensorData.nextHistory = sensorDataBody[3];

	// calculate sensor minutes eplapsed
	byte minut[2];
	minut[0] = sensorDataBody[293];
	minut[1] = sensorDataBody[292];
	sensorData.minutesSinceStart = minut[0] * 256 + minut[1];

	// store sensor livetime at program start
	if (startMinutesSinceStart == 0) {
		startMinutesSinceStart = sensorData.minutesSinceStart;
		startMillisSinceStart = mymillis();
		print_state(" *** set initial minutesSinceStart to ");
		Serial.printf("%d min, %d ms", startMinutesSinceStart, startMillisSinceStart);
	}
	// count the minutes from program start using sensor minutes counter
#ifdef SIM_MINUTES_UPCOUNT
	// simulate upcounting minutes using a dead sensor
	sensorData.minutesSinceStart += (loop_cnt - 1)*runPeriod;
#endif
	minutesSinceProgramStart = sensorData.minutesSinceStart - startMinutesSinceStart;
	// calculate minute distance to most recent history entry
	sensorData.minutesHistoryOffset = (sensorData.minutesSinceStart - 3) % 15 + 3;
	// copy 16 trend data
	int index = 0;
	for (int i = 0; i < 16; i++)
	{
		index = 4 + (sensorData.nextTrend - 1 - i) * 6;
		if (index < 4) index = index + 96;
		for (int k = index; k < index + 6; k++)
			pomiar[k - index] = sensorDataBody[k];
		// 13 Bits for raw BG reading are valid with the Libre sensor
		// meanwhile also hardcoded in xDrip+ (manual scan via NFC and blucon)
		sensorData.trend[i] = ((pomiar[1] << 8) & 0x1F00) + pomiar[0];
		sensorData.trendTemperature[i] = decode_temperature(&pomiar[3]);

		// print rawdata
		if (false) {
			Serial.printf("\r\nb #%d -> trend:[%x](%d)\t", i, sensorData.trend[i], sensorData.trend[i]);
			Serial.print("[");
			for (j = 0; j < 6; j++) {
				if (pomiar[j] <= 0x0f)
					Serial.print("0");
				Serial.print(pomiar[j], HEX);
				Serial.print(" ");
			}
			Serial.print("]");
			Serial.printf(" flg1: %b, temp: %f, flg: %b", pomiar[2], decode_temperature(&pomiar[3]), pomiar[5]);
		}
	}

	// copy 32 history data
	index = 0;
	for (int i = 0; i < 32; i++)
	{
		index = 100 + (sensorData.nextHistory - 1 - i) * 6;
		if (index < 100) index = index + 192;
		byte pomiar[6];
		for (int k = index; k < index + 6; k++) pomiar[k - index] = sensorDataBody[k];
		sensorData.history[i] = ((pomiar[1] << 8) & 0x0F00) + pomiar[0];
	}
}

void decodeSensorFooter()
{
	check_stack("#dsf#");
}

/*
* calculate the estimated sensor temperature using Ettores formula derived from the TI data sheet
* has to be proofed, experimental!!!
*/
float decode_temperature(byte *buf)
{
	float currentTemp;
	float NTC;
	float Temp;

	check_stack("#dtemp#");

	currentTemp = ((buf[1] * 256) + buf[0]) & 0x3FFF;
	NTC = currentTemp * 11.44;
	Temp = 22.124 * log(309443.81 / NTC);

	return(Temp);
}

void displaySensorData()
{
	check_stack("#dsd#");

	if (!sensorData.sensorDataOK)
		print_state("Sensor data error");
	else 
	{
		    print_state("");
		    Serial.printf("BG raw reading: %d, therm. temp est.: %f", sensorData.trend[0], sensorData.trendTemperature[0]);
	}
}
