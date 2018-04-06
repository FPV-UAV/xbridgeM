// reference: http://forum.arduino.cc/index.php?topic=226880.0
// topic: Stack address > 2k for ATMEGA328p?
// https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory



void memAndStack(char *func)
{
	print_state("");
	Serial.printf("(%s) - free RAM = %d, min. RAM avail log: %d", func, freeMemory(), minimumMemory);
	int v;
	Serial.print(", Stack Size = ");		// stack memory of RFduino begins at 0x20003FFF	
	Serial.print((MAXSTACK - (int)&v));
}

/*
* stack size check for ARM Cortec M0 processors (RFduino)
* according to the Memory.h file from the RFduino library the RAM layout is like this
* ram:
* 20000000 - 20001FFF -> stack RAM storage
* 20002000 - 20003FFF -> application RAM storage
* where we use the application RAM storage
*/
void check_stack(char *fkt)
{
	unsigned int fm = freeMemory();

	if (fm < minimumMemory) minimumMemory = fm;
			
	if (fm < STACKWARNLEVEL)			// check if enough memory is left for proper operation between heap top and stack bottom
	{
		if (!sreached) 
		{
			sreached = 1;
			Serial.printf("\r\n *** %s max stack size reached: %d left", fkt, fm); // print fnction call history			
			while (frind != fwind) 
			{
				Serial.printf("\r\n->%s (%d)", fktstack[frind], fktFree[frind]);
				if (++frind >= FKTSTACK) frind = 0;					
			}
			Serial.printf("\r\n *** ->%s max stack size reached: %d left", fkt, fm);
		}
	}
	
	if (strlen(fkt) < FNMLEN) strcpy(&fktstack[fwind][0], fkt); // store function call history		
	else 
	{
		memcpy(&fktstack[fwind][0], fkt, FNMLEN - 1);
		fktstack[fwind][FNMLEN - 1] = '\0';
	}
	fktFree[fwind] = fm;
	if (++fwind >= FKTSTACK) fwind = 0;
	
	if (fwind == frind)								// queue overflow, put read index one ahead
	{
		if (++frind >= FKTSTACK)
			frind = 0;
	}
}


int freeMemory() 
{
	char top;
	#ifdef __arm__
		return &top - reinterpret_cast<char*>(sbrk(0));
	#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
		return &top - __brkval;
	#else  // __arm__
		return __brkval ? &top - __brkval : &top - __malloc_heap_start;
	#endif  // __arm__
}
