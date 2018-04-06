void configWDT()
{
	print_state("configure watchdog timer");
	NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) | (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);
	NRF_WDT->CRV = 32768 * 60 * 11;         // 11 minut
	NRF_WDT->RREN |= WDT_RREN_RR0_Msk;
	NRF_WDT->TASKS_START = 1;
}

void restartWDT()
{	
	print_state("restart watchdog timer");
	NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

