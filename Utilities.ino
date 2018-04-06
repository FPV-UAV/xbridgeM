
void print_state(char *str)				// print timestamp and description of current status/action
{
	Serial.print(timeStamp());
	Serial.print(str);
}

unsigned long mymillis(void)
{
	return(millis() + TIMEOF8HINMS);    // next round value above 8 h buffer depth in ms
}

char *timeStamp()
{
	unsigned long ms = mymillis();
	unsigned long val;
	int i;

	check_stack("#timStmp#");

	timestring[0] = '\0';
	sprintf(timestring, "\r\n[%03d][%03d][%03d] ", ms / 60000, (ms / 1000) % 60, ms % 1000);
	return timestring;
}

bool checkCRC16(void *bytes, byte type)
{
	int number_of_bytes_to_read = 0;
	int offset = 0;

	check_stack("#ccrc16#");

	if (type == 0)
	{
		number_of_bytes_to_read = 24;
		offset = 0;
	}
	else if (type == 1)
	{
		number_of_bytes_to_read = 296;
		offset = 24;
	}
	else if (type == 2)
	{
		number_of_bytes_to_read = 24;
		offset = 320;
	}

	byte *data = (byte*)bytes;
	uint16_t x = data[0 + offset] | (data[1 + offset] << 8);
	uint16_t crc16calculated = computeCRC16(bytes, type);
	if (crc16calculated == x) return true;
	else return false;
}

uint16_t computeCRC16(void *bytes, byte type)
{
	int number_of_bytes_to_read = 0;
	int offset = 0;

	check_stack("#ccrc16#");

	if (type == 0)
	{
		number_of_bytes_to_read = 24;
		offset = 0;
	}
	else if (type == 1)
	{
		number_of_bytes_to_read = 296;
		offset = 24;
	}
	else if (type == 2)
	{
		number_of_bytes_to_read = 24;
		offset = 320;
	}
	byte *data = (byte*)bytes;
	uint16_t crc = 0xffff;
	for (int i = offset + 2; i < number_of_bytes_to_read + offset; ++i)           // first two bytes = crc16 included in data
	{
		crc = (uint16_t)((crc >> 8) ^ crc16table[(crc ^ data[i]) & 0xff]);
	}
	uint16_t reverseCrc = 0;
	for (int i = 0; i < 16; i++)
	{
		reverseCrc = (uint16_t)((uint16_t)(reverseCrc << 1) | (uint16_t)(crc & 1));
		crc >>= 1;
	}
	return reverseCrc;
}

