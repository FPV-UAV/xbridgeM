void setupBluetoothConnection()
{
	check_stack("#sblec#");

	print_state("configure and start BLE stack");
	#ifdef RFD
		RFduinoBLE.deviceName = LB_NAME;
		Serial.print(" - xbridge/wixel device");
		RFduinoBLE.advertisementData = LB_ADVERT;
		RFduinoBLE.customUUID = "0000ffe0-0000-1000-8000-00805f9b34fb";
		RFduinoBLE.advertisementInterval = MILLISECONDS(500);
		RFduinoBLE.txPowerLevel = 4;
		RFduinoBLE.begin();
	#else /* RFD */
		SimbleeBLE.deviceName = LB_NAME;
		Serial.print(" - setting xbridge/wixel device");
		SimbleeBLE.advertisementData = LB_ADVERT;
		SimbleeBLE.customUUID = "0000ffe0-0000-1000-8000-00805f9b34fb";
		SimbleeBLE.advertisementInterval = MILLISECONDS(500);
		SimbleeBLE.txPowerLevel = 4;
		SimbleeBLE.begin();
	#endif /* RFD */
	Serial.print(" - done");
}

/*
* check if we are connected to BLE and send then the current queue entries
*/
void dataTransferBLE()
{
	check_stack("#dtble#");

	if (!BTconnected) 
	{
		print_state("");
		Serial.printf("dataTransferBLE(), wait 100 s for BLE connect ...");
	}
	for (int i = 0; i < 100; i++) {
		if (BTconnected) 
		{			
			sendToXdripViaBLE();			
			break;
		}
		else 
		{
			waitDoingServices(1000, 1);
		}
	}
	NFCReady = 1;
}

/*
* during any wait action look for incoming chars to be processed
*/
void waitDoingServices(unsigned long wait_time, unsigned char bProtocolServices)
{
	unsigned long start_wait = mymillis();
	check_stack("#wds#");
	while ((mymillis() - start_wait) < wait_time)
	{
		if (bProtocolServices) controlProtocolService();
		delay(20);
	}
}

/*
* poll for an flag change or incoming chars to be processed during wait times
*/
int waitDoingServicesInterruptible(unsigned long wait_time, volatile boolean *break_flag, unsigned char bProtocolServices)
{
	unsigned long start_wait = mymillis();
	check_stack("#wdsi#");
	while ((mymillis() - start_wait) < wait_time) 
	{
		if (bProtocolServices) controlProtocolService();
		if (*break_flag) return (1); 
		delay(20);
	}
	return (0);
}

/*
* process incoming BLE packets or USB/seriall chars
*/
int controlProtocolService() // ok this is where we check if there's anything happening incoming on the USB COM port or UART 0 port.
{
	static unsigned long cmd_to;
	
	int nRet = 1;
	int i;
	unsigned char b;

	check_stack("#cps#");
	
	if (command_buff.nCurReadPos > 0 && (mymillis() - cmd_to) > 2000) //if we have timed out waiting for a command, clear the command buffer and return.
	{
		print_state("got <-[");
		for (i = 0; i < command_buff.nCurReadPos; i++)
			if (command_buff.commandBuffer[0] >= ' ') {
				Serial.write(command_buff.commandBuffer[i]);
			}
			else {
				Serial.print(F("0x"));
				Serial.print(command_buff.commandBuffer[i], HEX);
				Serial.print(F(" "));
			}
			Serial.print(F("]"));			
			init_command_buff(); // clear command buffer if there was anything
			return (nRet);
	}
	
	while ((Serial.available() || bleCharAvailable()) && command_buff.nCurReadPos < COMMAND_MAXLEN) //while we have something in either buffer,
	{

		if (ble_answer == 0) ble_answer = 1;
		
		if (((mymillis() - last_ble_send_time) > 3000) && !crlf_printed) // BLE receive which is not 3 s after last sending?
		{
			if (show_ble) Serial.print(F("\r\n"));
			crlf_printed = 1;
		}

		if (bleCharAvailable()) b = bleBufRead();

		command_buff.commandBuffer[command_buff.nCurReadPos] = b;
		command_buff.nCurReadPos++;
		cmd_to = mymillis();
		// if it is the end for the byte string, we need to process the command
		// valid data packet or "OK" received?
		if (command_buff.nCurReadPos == command_buff.commandBuffer[0] \
			|| (command_buff.commandBuffer[0] == 'O' && command_buff.commandBuffer[1] == 'K') \
			|| (command_buff.nCurReadPos == 1 && toupper(command_buff.commandBuffer[0]) == 'R'))
		{
			// ok we got the end of a command;
			if (command_buff.nCurReadPos) 
			{				
				nRet = doCommand(); // do the command				
				init_command_buff(); //re-initialise the command buffer for the next one.				
				if (!nRet) return (nRet); // break out if we got a breaking command					
			}
		}

		if (crlf_printed) 
		{
			crlf_printed = 0;
			last_ble_send_time = mymillis();
		}
		// otherwise, if the command is not up to the maximum length, add the character to the buffer.
	}

	if (ble_answer == 1) ble_answer = 2; //    Serial.print("-2-");

	if (command_buff.nCurReadPos) init_command_buff(); //re-initialise the command buffer for the next one.

	return (nRet);
}

void init_command_buff()
{	
	check_stack("#icb#");
	memset(command_buff.commandBuffer, 0, COMMAND_MAXLEN);
	command_buff.nCurReadPos = 0;
	return;
}

/*
* decode incoming serial/USB or BLE data commands
* commands are: R
*/
int doCommand(void)
{
	check_stack("#doCmd#");
	
	if (command_buff.commandBuffer[1] == 0x01 && command_buff.commandBuffer[0] == 0x06)		// TXID packet?
	{
		memcpy(&dex_tx_id, &command_buff.commandBuffer[2], sizeof(dex_tx_id)); // send back the TXID we think we got in response		
		return (0);
	}
	
	if (command_buff.commandBuffer[0] == 0x02 && command_buff.commandBuffer[1] == 0xF0 && !got_ack) // ACK packet?
	{
		got_ack = 1;
		init_command_buff();
		return (0);
	}
	
	if (toupper(command_buff.commandBuffer[0]) == 'R') 
	{
		print_state("R command - dop an extra read");
		#ifdef DEBUG
			Serial.println("R-command received.");
		#endif  
		readAllData();
		fillupMissingReadings(0);
		dataTransferBLE();
	}
	// we don't respond to unrecognised commands.
	return (1);
}

boolean bleCharAvailable(void)
{
	check_stack("#bca#");

	if (bleBufRi == bleBufWi)
		return (0);
	else
		return (1);
}

// read one char form BLE incoming buffer
unsigned char bleBufRead(void)
{
	unsigned char ret = 0xFF;

	check_stack("#bbr#");

	if (bleBufRi != bleBufWi) 
	{
		ret = bleBuf[bleBufRi++];
		if (show_ble) 
		{
			Serial.print(ret, HEX); Serial.print(" ");
		}
		if (bleBufRi >= BLEBUFLEN)
			bleBufRi = 0;
	}
	return (ret);
}

/*
* send the BG reading to xDrip+
*/
void sendToXdripViaBLE(void)
{
	check_stack("#stxvble#");
	boolean resend_pkt = 0;
	int more_than_one = 0;
	// if we are connected via BLE, the queue contains BG readings to be send and we have enough time left
	while ((Pkts.read != Pkts.write) && BTconnected && ((mymillis() - pkt_time) < ((DXQUEUESIZE + 1) * 5000))) 
	{
		got_ack = 0;
		unsigned long got_ack_time;
		
		if (Pkts.buffer[Pkts.read].raw) // dont send a 0 entry
		{			
			if (more_than_one++) waitDoingServices(100, 1); // waiting time before next packet will be sended				
			got_ack_time = mymillis();
			print_state("");
			Serial.printf("try sending [%d BG / -%d min / %d] - ", Pkts.buffer[Pkts.read].raw / 1000, (mymillis() - Pkts.buffer[Pkts.read].ms) / 60000, Pkts.read);	
			pPkt = Pkts.buffer[Pkts.read];
			sendBgViaBLE();
			
			int j;
			for (j = 0; j < 10; j++) // wait 10 s for ack
			{
				waitDoingServicesInterruptible(1000, &got_ack, 1);
				if (!BTconnected) 
				{
					print_state("BLE connection lost during 10 s wait for ack, go to sleep");
					break;
				}
			}
		}		
		else		// when it was 0 increment read counter
		{
			print_state("do not send a raw value containing 0, increment read pointer instead, simulate ACK");
			got_ack = 1;
		}

		if (got_ack) 
		{
			Serial.printf("ACK / %d ms", mymillis() - got_ack_time);
			if (++Pkts.read >= DXQUEUESIZE) Pkts.read = 0;     //increment read position since we got an ack for the last package
			resend_pkt = 0;
		}
		else 
		{
			if (!resend_pkt) 
			{
				print_state("no ACK received, try again");
				resend_pkt = 1;
				
				restartBtStack(1); // restart BT stack in case of BT transfer breaks after some blocks sended				
				if (BTconnected) delay(5000); // wait additional 5 s for DexCollectionService to sttle
			}
			else 
			{
				print_state("no ACK received, try again next wakeup");
				resend_pkt = 0;
				break;
			}
		}
	} 
}

/*
* send the passed (emulated) Dexom packet to xDrip+ via the xbridge2 protocol
*/
int sendBgViaBLE()  
{
	nRawRecord msg;
	unsigned char msgbuf[80];
	check_stack("#sbgvb#");

	if (!BTconnected) 
	{
		print_state("no BT connection, skip");
		return 0;
	}

	// prepare the message
	// calculate struct size by hand, structs seems to be long aligned (adding 0x00 0x20 to gaps)
	// better change to ___attribute____ packed
	msg.size = sizeof(msg.size) + sizeof(msg.cmd_code) + sizeof(msg.raw) + sizeof(msg.filtered)
		+ sizeof(msg.dex_battery) + sizeof(msg.my_battery) + sizeof(msg.dex_src_id)
		+ sizeof(msg.delay) + sizeof(msg.function);
	msg.cmd_code = 0x00;
	msg.raw = pPkt.raw;
	msg.filtered = pPkt.raw;
	msg.dex_battery = 215;  // simulate good dexcom transmitter battery
	msg.my_battery = SoCData.voltagePercent;
	msg.dex_src_id = dex_tx_id;
	msg.delay = mymillis() - pPkt.ms;
	msg.function = DEXBRIDGE_PROTO_LEVEL; // basic functionality, data packet (with ack), TXID packet, beacon packet (also TXID ack).
	*(unsigned char *)&msgbuf[0] = msg.size;
	*(unsigned char *)&msgbuf[1] = msg.cmd_code;
	*(unsigned long *)&msgbuf[2] = msg.raw;
	*(unsigned long *)&msgbuf[6] = msg.filtered;
	*(unsigned char *)&msgbuf[10] = msg.dex_battery;
	*(unsigned char *)&msgbuf[11] = msg.my_battery;
	*(unsigned long *)&msgbuf[12] = msg.dex_src_id;
	*(unsigned long *)&msgbuf[16] = msg.delay;
	*(unsigned char *)&msgbuf[20] = msg.function;	
	send_data((unsigned char *)&msgbuf, 20);	// send the message via BLE
	return 1;
}

void restartBtStack(boolean waitForBT)
{
	// if we loosed the BLE connection during transfer or whatever reason restart the BLE stack here
	if (!BTconnected)
	{
		print_state("restarting BT stack, stop BLE");
		// check for endless loop
		#ifdef RFD    
			while (!RFduinoBLE.radioActive) {};
			while (RFduinoBLE.radioActive) {};
			RFduinoBLE.end();
		#else /* RFD */
			while (!SimbleeBLE.radioActive) {};
			while (SimbleeBLE.radioActive) {};
			SimbleeBLE.end();
		#endif /* RFD */
		setupBluetoothConnection();
		
		if (waitForBT) // wait for max 40 s for BLE reconnect
		{
			Serial.printf(" - waiting up to 100 s for reconnect");
			waitDoingServicesInterruptible(100000, &BTconnected, 1);   // @bertroode changed to 100
		}
	}
}

/*
* send a raw data packet via BLE
*/
void send_data(unsigned char *msg, unsigned char len)
{
	unsigned char i = 0;
	check_stack("#sd#");

	last_ble_send_time = mymillis();
	crlf_printed = 0;

	#ifdef RFD
		RFduinoBLE.send((char *)msg, len);
	#else RFD
		SimbleeBLE.send((char *)msg, len);
	#endif RFD

	delay(100);

	if (show_ble) 
	{
		print_state("sending: <");
		for (i = 0; i < len; i++) Serial.printf("%x ", msg[i]);
		Serial.print(">");
		print_state("response: ");
	}
}

#ifdef RFD
	void RFduinoBLE_onReceive(char *data, int len)
	{
		int i;
		check_stack("#onr#");
		for (i = 0; i < len; i++) bleBufWrite(data[i]);
	}

	void RFduinoBLE_onDisconnect()
	{
		check_stack("#odis#");
		BTconnected = false;
		#ifdef SHOW_BLESTATECHANGE
			print_state("+++++++++++++ BLE lost");
		#endif
	}

	void RFduinoBLE_onConnect()
	{
		check_stack("#onc#");
		BTconnected = true;
		#ifdef SHOW_BLESTATECHANGE
			print_state("+++++++++++++ BLE conn");
		#endif
	}

	void RFduinoBLE_onRSSI(int rssi)
	{
		check_stack("#onRSSI#");
		SoCData.rssi = rssi;
	}

	void RFduinoBLE_onAdvertisement(bool start) 
	{
	}

	bool BLEconnected()
	{
	}
#else /* RFD */
	void SimbleeBLE_onReceive(char *data, int len)
	{
		int i;
		check_stack("#orc#");
		for (i = 0; i < len; i++) bleBufWrite(data[i]);
	}

	void SimbleeBLE_onDisconnect()
	{
		check_stack("#odis#");
		BTconnected = false;
		#ifdef SHOW_BLESTATECHANGE
			print_state("+++++++++++++ BLE lost");
		#endif
	}

	void SimbleeBLE_onConnect()
	{
		check_stack("#ocon#");
		BTconnected = true;
		#ifdef SHOW_BLESTATECHANGE
			print_state("+++++++++++++ BLE conn");
		#endif
	}

	void SimbleeBLE_onRSSI(int rssi)
	{
		check_stack("#orssi#");
		SoCData.rssi = rssi;
	}

	void SimbleeBLE_onAdvertisement(bool start) 
	{
	}

	bool BLEconnected()
	{
	}
#endif /* RFD */

void bleBufWrite(unsigned char c)
{
	check_stack("#bbw#");
	bleBuf[bleBufWi++] = c;
	if (bleBufWi >= BLEBUFLEN) bleBufWi = 0;
}