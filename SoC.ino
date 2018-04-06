void getSoCData()
{
	check_stack("#gsocd#");

	analogReference(VBG);
	analogSelection(VDD_1_3_PS);
	#ifdef RFD
		uint16_t sensorValue1 = readVDD();
		int sensorValue = analogRead(1);
		SoCData.voltage = sensorValue * (360 / 1023.0) * 10;
		SoCData.temperatureC = RFduino_temperature(CELSIUS);
		SoCData.temperatureF = RFduino_temperature(FAHRENHEIT);
	#else
		uint16_t sensorValue = readVDD();
		SoCData.voltage = sensorValue * (380 / 1023.0) * 10;
		SoCData.temperatureC = Simblee_temperature(CELSIUS);
		SoCData.temperatureF = Simblee_temperature(FAHRENHEIT);
	#endif
	SoCData.voltagePercent = map(SoCData.voltage, MIN_VOLTAGE, MAX_VOLTAGE, 0, 100);
	if (SoCData.voltage < MIN_VOLTAGE)
		BatteryOK = false;
	else
		BatteryOK = true;
}

void printSoCData(void)
{
	check_stack("#prtSoCDt#");

	print_state("SoC Data:");
	Serial.printf("\r\n\tU[mv]: %d (%d%%)", SoCData.voltage, SoCData.voltagePercent);
	Serial.printf("\r\n\tSoC Temp[C]: %f Temp[F]: %f", SoCData.temperatureC, SoCData.temperatureF);
	Serial.printf("\r\n\tRSSI: %f", SoCData.rssi);
}

uint16_t readVDD()				// simple low-level subroutine to get the VDD supply voltage
{
	uint16_t value;

	check_stack("#rvdd#");
	
	NRF_ADC->CONFIG = 0x0000001A;						// Compare VDD against VBG(1.2v reference) at 10 bit resolution
	NRF_ADC->ENABLE = 0x00000001;						// Enable the ADC
	NRF_ADC->TASKS_START = 1;							// Start a conversion
	while (NRF_ADC->BUSY == 1) {}						// Busy Wait for completion
	delay(1);											// needed if used on RFduino for some reason.
	value = (uint16_t)NRF_ADC->RESULT;					// Read the ADC Result
	NRF_ADC->TASKS_STOP = 1;							// Turn off ADC
	NRF_ADC->ENABLE = 0;								// Disable the ADC
	return(value);
}
