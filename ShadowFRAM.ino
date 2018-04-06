/*
* shadow FRAM processing - init routine for selective nfc reading
*/
void initSensor()
{

	check_stack("#initsenor#");

	sensor.nextTrend = 0;
	sensor.oldNextTrend = 0;
	sensor.nextHistory = 0;
	sensor.oldNextHistory = 0;
	sensor.minutesSinceStart = 0;
	sensor.oldMinutesSinceStart = 0;
	for (int i = 0; i < sizeof(sensor.uid); i++) 
	{
		sensor.uid[i] = 0;
		sensor.oldUid[i] = 0;
	}
	for (int i = 0; i < sizeof(sensor.fram); i++) 
	{
		sensor.fram[i] = 0;
	}
	for (int i = 0; i < sizeof(sensor.resultCodes); i++) 
	{
		sensor.resultCodes[i] = 0;
	}
}

void initialFillupBackfillTimestamps(void)
{
	unsigned long sim_time_ms;
	unsigned char store;

	check_stack("#ifubft#");

	// fillup queue to transfer the last 8 for test direct after system start
	print_state("prepare 8 h backfilling - current runtime = ");
	Serial.printf("%d min", mymillis() / 60000L);
	store = sensorData.sensorStatusByte;
	sensorData.sensorStatusByte = 0x03; // needed for using the put_reading2queue() function without valid sensor data
	for (int i = 0; i < FILL8H; i++) {
		// create timestamps 0, 5, 10, 15, ... min relative to current time
		sim_time_ms = mymillis() - (long)(FILL8H - i) * 300000L;
		put_reading2queue(sim_time_ms, 0, 0);
	}
	sensorData.sensorStatusByte = store;
	// set to end of initial queue
	minutesSinceProgramStart = (mymillis() / (60000));
}

/*
* put the current BG to the send queue
* glucose of 0 values indicates a missed reading which has to be filled from the trend/history buffer
* of the Libre sensor before sending (backfilling)
*/
int put_reading2queue(unsigned long current_time_ms, int glucose, int debug)
{
	check_stack("#prtoq#");

	Pkts.buffer[Pkts.write].raw = scale_bg(glucose);
	Pkts.buffer[Pkts.write].ms = current_time_ms;
	pkt_time = current_time_ms;

	if (debug) 
	{
		print_state("");
		Serial.printf("got glucose(%f) at %d min, stored at %d, inc. write to ", scale_bg(glucose) / 1000.0, current_time_ms / 60000, Pkts.write);
	}
	// so increment write position for next round...
	if (++Pkts.write >= DXQUEUESIZE) Pkts.write = 0;
	if (debug) Serial.print(Pkts.write); Serial.print(" - ");
	if (Pkts.read == Pkts.write) 
	{
		if (debug) print_state("queue overflow, incrementing read overwriting oldest entry");
		if (++Pkts.read >= DXQUEUESIZE) Pkts.read = 0; //overflow in ringbuffer, overwriting oldest entry, thus move read one up
	}
}


int scale_bg(int glucose)
{
	check_stack("#sbg#");
	return((glucose / 8.5f)*1000.0f);
}

/*
* search BG queue for missing entries (0), if within the last 8 h fill the values from the Libre sensor FRAM
*/
void fillupMissingReadings(boolean debug)
{
	int i, j;
	int nulls = 0;
	int difftime;
	unsigned long mytime;

	check_stack("#fmr#");

	// ??
	minutesSinceProgramStart = (mymillis() / (60000));
	mytime = minutesSinceProgramStart;
#ifdef DEBUG
	print_state("fillupMissingReadings() - current time: ");
	Serial.printf("%d (%d/%d), sensorData.minutesHistoryOffset: %d", mytime, mytime / 15, mytime % 15, sensorData.minutesHistoryOffset);
#endif DEBUG
	// check complete queue for missing BG readings (0 entries)
	for (int i = 0; i < DXQUEUESIZE; i++) {
		// get time of queue entry
		unsigned long timeOfMissingBG_sec = (Pkts.buffer[i].ms + 500) / 1000;
		// if there is a missing BG reading and a valid time
		if ((Pkts.buffer[i].raw == 0) && (Pkts.buffer[i].ms != 0)) {
			if (debug) {
				print_state("");
				Serial.printf("%d - timeOfMissingBG %d/%d ", i, timeOfMissingBG_sec / 60, mytime);
			}
			nulls++;
			// if the missing value is not more than 15 min old search in trend array
			if ((mymillis() - Pkts.buffer[i].ms) / 1000 <= (15 * 60)) {
				if (debug)
					Serial.print(" - trend array");
				for (j = 0; j < 16; j++) {
					unsigned long timeGoBackInTrend_sec = (minutesSinceProgramStart - j) * 60;
					// trend array begins at current time - counting in minutes
					difftime = timeGoBackInTrend_sec - timeOfMissingBG_sec;
					difftime = abs(difftime);
					// find nearest entry
					if (difftime < 31) {
						Pkts.buffer[i].raw = scale_bg(sensorData.trend[j]);
						if (debug)
							Serial.printf(" - %d delta %d s, replacing with %d", i, difftime, (Pkts.buffer[i].raw + 500) / 1000);
						break;
					}
				}
			}
			else {
				if (debug)
					Serial.print(" - history array");
				for (j = 0; j < 32; j++) {
					unsigned long timeToHistoryEntry_sec = (minutesSinceProgramStart - (sensorData.minutesHistoryOffset + 15 * j)) * 60;
					difftime = timeToHistoryEntry_sec - timeOfMissingBG_sec;
					difftime = abs(difftime);
					if (difftime < 451) {
						Pkts.buffer[i].raw = scale_bg(sensorData.history[j]);
						if (debug)
							Serial.printf("(%d/%d min) - delta %d s, set BG %d", i, timeToHistoryEntry_sec / 60, difftime, (Pkts.buffer[i].raw + 500) / 1000);
						break;
					}
				}
				// not found or out of range (diiftime = 537 ...)
				if (Pkts.buffer[i].raw == 0) {
					if (debug)
						print_state(" *** not found, set to histroy index 31");
					Pkts.buffer[i].raw = scale_bg(sensorData.history[31]);
				}
			}
		}
	}
	if (nulls > 0) {
		print_state("");
		Serial.printf("found %d reading(s) for backfilling, stored in queue ...", nulls);
	}
}