/*
 * lpwan_manager.c
 *
 *  Created on: Jun 29, 2017
 *      Author: waseemh
 */

#include "../resource managers_header/lpwan_manager.h"
#include "em_wdog.h"
			/*
			 * Shared variables
			 */
	static osjob_t 			init_job;
	static osjob_t			app_job;
	static bool				joined_lora=false;
	static bool				last_tx_complete=true;
	static uint8_t			gps_state=0;
	static uint8_t 			node_id=0;
	static uint16_t 		battery_info=180;  // 14-bit number, this should be implemented in the future
	static uint16_t 		tbr_serial_id=0;
#ifdef USE_RADIO
	nav_data_t	 			running_tstamp;
	nav_data_t	 			ref_tstamp;
	int						diff_in_tstamp;
#endif
	char					temp_buf[256];
			/*
			 * LMIC callbacks
			 */

	// provide application router ID (8 bytes, LSBF)
	void os_getArtEui (u1_t* buf) {
	    memcpy(buf, APPEUI, 8);
	}

	// provide device ID (8 bytes, LSBF)
	void os_getDevEui (u1_t* buf) {
	    memcpy(buf, DEVEUI, 8);
	    node_id=read_switch() & 0x3f;	//which is MSByte -1!!!
		buf[1]=node_id;
	}

	// provide device key (16 bytes)
	void os_getDevKey (u1_t* buf) {
	    memcpy(buf, DEVKEY, 16);
	}

			/*
			 * private functions
			 */
	static	void lora_tx_function (void) {
#ifdef USE_LORA_ACK
		if(LMIC_setTxData2(2,lora_buffer,lora_msg_length,1)==0){ //Ack => blocking behavior....
			return;
		}
		else{
		 	debug_str((const u1_t*)"Tx function failed on length");
		 	//onEvent(0);
		}
#else
		if(!LMIC_setTxData2(2,lora_buffer,lora_msg_length,0)){
			return;
		}
		else{
		 	debug_str((const u1_t*)"Tx function failed on length");
		 	//onEvent(0);
		}
#endif
		return;
	}
	static void setup_channel (void){
		int 		channel=4;
			/*
			 * Channel settings:
			 * 868.5 MHz
			 * Spreading factor=7
			 * ADR=off
			 * 14dBm power @ 1% duty cycle
			 */
		LMIC_setupBand(BAND_AUX,14,100);
		LMIC_setupChannel(4,868500000,DR_RANGE_MAP(DR_SF12,DR_SF7),BAND_AUX);
		for(int i=0; i<9; i++) {
		  if(i != channel) {
			LMIC_disableChannel(i);
		  }
		}
		LMIC_setDrTxpow(DR_SF7, 14);
		LMIC_setAdrMode(false);
		return;
	}
	static void init_funct (osjob_t* j) {
	    // reset MAC state
	    LMIC_reset();
	    // start joining
	    LMIC_startJoining();
		return;
	}

	static void app_funct (osjob_t* j) {
		time_manager_cmd_t		time_manager_cmd=basic_sync;
		static uint32_t prev_gps_tstamp = 0;
		static uint16_t gps_send_intvl = 60*10;  // 60 * number of minutes. Every time it's more, gps packet is sent with header.


				//add 10secs
		ref_tstamp.gps_timestamp+=BASIC_SYNCH_SECONDS;
				//get synch command type from time manager
		time_manager_cmd=time_manager_get_cmd();
				//update application manager
		app_manager_tbr_synch_msg(time_manager_cmd,ref_tstamp,running_tstamp,diff_in_tstamp);
					//update Timestamps
			gps_state=0;
			gps_poll_nav_status();
			while(gps_state<15){
				running_tstamp=gps_get_nav_data();
				if (running_tstamp.valid==true){
					break;
				}
					gps_state++;
			}
			running_tstamp.gps_timestamp=time_manager_unixTimestamp(running_tstamp.year,running_tstamp.month,running_tstamp.day,
																running_tstamp.hour,running_tstamp.min,running_tstamp.sec);

			diff_in_tstamp= (int)(ref_tstamp.gps_timestamp-running_tstamp.gps_timestamp);
			sprintf(temp_buf,"LoRA_join_flag=%d\tTime Diff:Ref=%ld\tCur=%ld\tdiff=%d\tmin=%d\tsec=%d\tnano=%ld\ttAcc=%ld\tGPS_fix=%2x\tgps_state=%d\n",joined_lora,(time_t)ref_tstamp.gps_timestamp,(time_t)running_tstamp.gps_timestamp,diff_in_tstamp,running_tstamp.min,running_tstamp.sec,running_tstamp.nano,running_tstamp.tAcc,running_tstamp.fix,gps_state);
			debug_str(temp_buf);
			if(time_manager_cmd==advance_sync && joined_lora==true && last_tx_complete==true){
				// if tbr_serial_id hasn't been set yet, set tbr_serial_id
				if(tbr_serial_id == 0){
					tbr_serial_id = app_manager_get_tbr_serial_id();
				}
				if ((running_tstamp.gps_timestamp - prev_gps_tstamp) < gps_send_intvl){
					// Clear header flag before getting buffer
					lora_buffer[1]=0;
					lora_msg_length=app_manager_get_lora_buffer(lora_buffer);
					lora_buffer[0]=(uint8_t)(tbr_serial_id>>6);
					lora_buffer[1]=(uint8_t)(tbr_serial_id<<2);  // Header 00b and TBR Serial ID
				} else {
					prev_gps_tstamp = running_tstamp.gps_timestamp;

					// latitude, longitude and PDOP needs to be scaled
					uint32_t long_gps = running_tstamp.longitude / 100;  // Remove 2 decimal digits
					uint32_t lat_gps = running_tstamp.latitude / 100; 	 // Remove 2 decimal digits
					uint16_t pDOP = running_tstamp.pDOP / 10;  			 // Remove 1 decimal digit

					// Round decimal 1 up if difference equal or larger than 5
					if((uint8_t)((running_tstamp.longitude - long_gps*100) >= 5)){long_gps = long_gps + 1;}
					if((uint8_t)((running_tstamp.latitude - lat_gps*100) >= 5)){lat_gps = lat_gps + 1;}
					if((uint8_t)((running_tstamp.pDOP - pDOP*10) >= 5)){pDOP = pDOP + 1;}

					// number of satellites enforced as 5-bit number, and pDOP enforced as 7-bit number
					uint8_t numSV = running_tstamp.numSV;
					if(numSV>31){numSV=31;}  // enforce 5-bit number
					if(pDOP>127){pDOP=127;}  // enforce 7-bit number

					// Clear header flag before getting buffer
					lora_buffer[1]=1;
					lora_msg_length=app_manager_get_lora_buffer(lora_buffer);
					lora_buffer[0]=(uint8_t)(tbr_serial_id>>6);
					lora_buffer[1]=(uint8_t)((tbr_serial_id<<2) | 0x01);  // Header 01b flag and TBR serial id

					// If there are no tbr messsages, send gps only message with gps timestamp
					if (lora_msg_length == 0){
						lora_buffer[2] = (uint8_t)(running_tstamp.gps_timestamp>>24);
						lora_buffer[3] = (uint8_t)(running_tstamp.gps_timestamp>>16);
						lora_buffer[4] = (uint8_t)(running_tstamp.gps_timestamp>>8);
						lora_buffer[5] = (uint8_t)(running_tstamp.gps_timestamp>>0);
						lora_msg_length = 16; // header (6 bytes) + gps data (10 bytes)
					}

					// Fill gps data into buffer
					lora_buffer[6]=(uint8_t)(battery_info>>6);
					lora_buffer[7]=(uint8_t)((battery_info<<2) | ((long_gps>>24) & 0x03));
					lora_buffer[8]=(uint8_t)(long_gps>>16);
					lora_buffer[9]=(uint8_t)(long_gps>>8);
					lora_buffer[10]=(uint8_t)(long_gps>>0);
					lora_buffer[11]=(uint8_t)((pDOP<<1) | ((lat_gps>>24) & 0x01));
					lora_buffer[12]=(uint8_t)(lat_gps>>16);
					lora_buffer[13]=(uint8_t)(lat_gps>>8);
					lora_buffer[14]=(uint8_t)(lat_gps>>0);
					lora_buffer[15]=(uint8_t)((running_tstamp.fix<<5) | (numSV));
				}
				if(lora_msg_length>0){
					lora_tx_function();
					GPIO_PinOutSet(LED_GPS_RADIO_PORT, LED_RADIO);
					last_tx_complete=false;
				}
		}
		WDOGn_Feed(WDOG);
		os_clearCallback(&app_job);
	return;
	}
			/*
			 * public functions
			 */

	void lpwan_init(void){
			//Setup and initialize WDOG
		WDOG_Init_TypeDef	wdog_init=WDOG_INIT_DEFAULT;
		wdog_init.perSel=wdogPeriod_16k;
		wdog_init.em2Run=true;
		wdog_init.em3Run=true;
		CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);
		RMU_ResetControl(rmuResetBU, rmuResetModeClear);
		display_clear();
			//Timing accuracy issue
		uint32_t t_acc_limit=1000;
		node_id=read_switch() & 0x3f;
		if(node_id>=32){
			t_acc_limit=1000;
		}else{
			t_acc_limit=2000000000;  //nsec which is default...
		}
		sprintf(temp_buf,"Node ID=%2x\nInit. successful\nResolving GPS timestamp",node_id);
		display_put_string(3,3,temp_buf,font_medium);
		gps_poll_nav_status();
		  while(1){
			  ref_tstamp=gps_get_nav_data();
			  ref_tstamp.gps_timestamp=time_manager_unixTimestamp(ref_tstamp.year,ref_tstamp.month,ref_tstamp.day,
					  	  	  	  	  	  	  	  	  	  	  	  	 ref_tstamp.hour,ref_tstamp.min,ref_tstamp.sec);
			  if(ref_tstamp.valid==false){
				  gps_state++;
				  if(gps_state>15){
					  debug_str("GPS double polling scenario\n");
					  delay_ms(7);
					  gps_poll_nav_status();
					  gps_state=0;
				  }
			  }
			  else {
				  if(ref_tstamp.fix==0x03 && ref_tstamp.gps_timestamp%10==0 && ref_tstamp.tAcc<=t_acc_limit){
				  //if(ref_tstamp.fix==0x03 && ref_tstamp.gps_timestamp%10==0){
					  break;
				  }
				  else {
						gps_poll_nav_status();
				  }
			  }
		  }
		time_manager_init();
		os_init();
		WDOGn_Init(WDOG,&wdog_init);
		debug_str((const u1_t*)"\t\tRadio Version. OS and sync initialized. Waiting for join to finish...\n");
		display_clear();
		sprintf(temp_buf,"Node ID=%2x\nRunning Loop\nNOT JOINED...\n",node_id);
		display_put_string(3,3,temp_buf,font_medium);
		display_update();
		os_setCallback(&init_job, init_funct);
		os_runloop();
	}

	void onEvent (ev_t ev) {

		switch(ev) {
		  case EV_JOINING:
			  debug_str((const u1_t*)"\tEV_JOINING\n");
			  break;
		  case EV_JOINED:
			  debug_str((const u1_t*)"\tEV_JOINED\n");
			  os_clearCallback(&init_job);
			  setup_channel();						//setup channel....
			  joined_lora=true;
			  display_clear();
			  sprintf(temp_buf,"Node ID=%2x\nRunning Loop\nJOINED LoRa...\n",node_id);
			  display_put_string(3,3,temp_buf,font_medium);
			  display_update();
			  break;
		  case EV_TXCOMPLETE:
#ifdef USE_LORA_ACK
		  if(LMIC.txrxFlags & TXRX_ACK){
			  debug_str((const u1_t*)"\tEV_TXCOMPLETE\n");
			  os_setCallback(&app_job, app_funct);
		  }
		  else{
			  debug_str((const u1_t*)"\nNo ACK RXCVD retrying...\n");
			  lora_tx_function();	//retry logic. NOT tested.
		  }
#else
		  debug_str((const u1_t*)"\tEV_TXCOMPLETE\n");
		  GPIO_PinOutClear(LED_GPS_RADIO_PORT, LED_RADIO);
		  last_tx_complete=true;
#endif
			  break;
		  case EV_JOIN_FAILED:
			  debug_str((const u1_t*)"\tEV_JOIN_FAILED\n");
			  break;
		  case EV_RXCOMPLETE:
			  debug_str((const u1_t*)"\tEV_RXCOMPLETE\n");
			  break;
		  case EV_SCAN_TIMEOUT:
			  debug_str((const u1_t*)"\tEV_SCAN_TIMEOUT\n");
			  break;
		  case EV_LINK_DEAD:
			  debug_str((const u1_t*)"\tEV_LINK_DEAD\n");
			  break;
		  case EV_LINK_ALIVE:
			  debug_str((const u1_t*)"\tEV_LINK_ALIVE\n");
			  break;
		  default:
			  debug_str((const u1_t*)"\tEV_DEFAULT\n");
			  break;
		}
	}

	void debug_function(void){
		os_setCallback(&app_job, app_funct);
	}
