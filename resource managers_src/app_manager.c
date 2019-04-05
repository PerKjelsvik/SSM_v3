/*
 * app_manager.c
 *
 *  Created on: May 2, 2017
 *      Author: waseemh
 */


#include "../resource managers_header/app_manager.h"

		/*
		 * private variables
		 */
static			FATFS 				FatFs;
static 			int					tbr_lora_length=0;
static 			uint8_t				tbr_lora_buf[ARRAY_MESSAGE_SIZE];
static			char				tbr_sd_card_buf[2*ARRAY_MESSAGE_SIZE];	//tags with NAV data appended

		/*
		 * public variables
		 */
static 			unsigned char  		rs232_tx_buf[512];
		/*
		 * external variables
		 */
extern 			uint32_t			cum_temp_counter;
extern 			uint32_t			cum_detections_counter;
		/*
		 * private functions
		 */
bool log_file_sys_setup(uint16_t year,uint8_t month,uint8_t day, char buf[]){

	char 			filename[32]="00000000";
	bool			ret_flag=false;
	FIL  			f_pointer;
	FRESULT 		f_ret;

	/*
	 * FR_NOT_READY issue not solved with delay. Maybe send a command and wait...
	 * BUT only using f_open and not f_mount in this function works  :-)
	 * shutdown only once in init. NOT in this function
	 * Current Scenario: Current is 26mA on average but on write, goes to 46mA
	 * which is normal and as per logic. Function is almost done.
	 * Also works with remove and reinsert card
	 */
	f_mount(&FatFs,"", 1);

	sprintf(filename,"%4d%d%d.txt",year,month,day);
	f_ret = f_open(&f_pointer, filename, FA_WRITE | FA_OPEN_APPEND);
	if(f_ret==FR_OK){
    	f_ret = f_puts(buf,&f_pointer);
    	f_close(&f_pointer);
    	if(f_ret>0){
    		ret_flag=true;
    	}
    	else{
    		ret_flag=false;
    	}
    }
    else{
    	ret_flag=false;
		sprintf((char *)rs232_tx_buf,"f_open=%1d and name=%s\n",f_ret,filename);
		debug_str((unsigned char *)rs232_tx_buf);
		f_close(&f_pointer);
    }
    return ret_flag;
}

bool tbr_cmd_update_rgb_led(tbr_cmd_t tbr_cmd, time_t timestamp){
	bool 	ret_flag=false;

	ret_flag=tbr_send_cmd(tbr_cmd,timestamp);
	if(ret_flag){
		set_status_led(false,true);
	}
	else{
		set_status_led(true,false);
	}
	delay_ms(6);
	set_status_led(false,false);
	return ret_flag;
}
int nmea0183_checksum(char *nmea_data)
{
    int crc = 0;
    int i;

    // the first $ sign and the last two bytes of original CRC + the * sign
    for (i = 1; i < strlen(nmea_data) - 2; i ++) {
        crc ^= nmea_data[i];
    }

    return crc;
}
void append_gps_status(char *tbr_msg_buf, int tbr_msg_count, nav_data_t nav_data){
			//GP variables
	int	 		outer_loop_var=0;
	int	 		inner_loop_var=0;
	int 		tbr_msg_buf_offset=0;
	int 		tbr_sd_card_buf_offset=0;
	char		temp_single_tbr_msg_buf[60];
	char		temp_single_appended_msg_buf[100];
			//variables for latitude, long and DOP calculations
	float		latitude=0.0;
	float		longitude=0.0;
	int32_t		lat_deg=0;
	int32_t		long_deg=0;
	float		lat_min=0.0;
	float		long_min=0.0;
	float		p_dop=0.0;
	int 		fixType=3;
	uint8_t		checksum=0x3f;
	char		lat_dir='N';	//1=North and 0=South
	char		long_dir='E';	//1=East and 0=West

			//calculate navigation variables in degrees and minutes
		//Latitude
	latitude=(float)nav_data.latitude;
	latitude=(float)(latitude/10000000);
	if(latitude<0.0){
		latitude=(float)-1.0*latitude;
		lat_dir='S';		//South
	}
	else{
		lat_dir='N';		//North
	}
	lat_deg=(int32_t)(latitude);
	lat_min=(float)(latitude-lat_deg);
	lat_min=(float)lat_min*60;
		//Longitude
	longitude=(float)nav_data.longitude;
	longitude=(float)(longitude/10000000);
	if(longitude<0.0){
		longitude=(float)-1*longitude;
		long_dir='W';		//West
	}
	else{
		long_dir='E';		//East
	}
	long_deg=(int32_t)(longitude);
	long_min=(float)(longitude-long_deg);
	long_min=(float)long_min*60;

		//DOP for position
	p_dop=(float)(nav_data.pDOP/100);
			//now decimate messages and append navigation data
	clear_buffer(tbr_sd_card_buf, ARRAY_MESSAGE_SIZE);
	tbr_msg_buf_offset=0;
	tbr_sd_card_buf_offset=0;
	for(outer_loop_var=0;outer_loop_var<tbr_msg_count;outer_loop_var++){
		clear_buffer(temp_single_tbr_msg_buf, 60);
		clear_buffer(temp_single_appended_msg_buf, 100);
			//extract single TBR message
		for(inner_loop_var=0;inner_loop_var<60;inner_loop_var++){
			if(tbr_msg_buf[tbr_msg_buf_offset+inner_loop_var]=='\n'){
				tbr_msg_buf_offset+=inner_loop_var+1;	//\n dropped here
				break;
			}
			temp_single_tbr_msg_buf[inner_loop_var]=tbr_msg_buf[tbr_msg_buf_offset+inner_loop_var];
		}
		if(strlen(temp_single_tbr_msg_buf)>30){	//valid messages only
				//append NAV data
			sprintf(temp_single_appended_msg_buf,"%s,%02d%02.3f,%c,%03d%02.3f,%c,%01d,%02d,%01.1f*\n",temp_single_tbr_msg_buf,lat_deg,lat_min,lat_dir,long_deg,long_min,long_dir,fixType,nav_data.numSV,p_dop);
			checksum=nmea0183_checksum(temp_single_appended_msg_buf);
			sprintf(temp_single_appended_msg_buf,"%s,%02d%02.3f,%c,%03d%02.3f,%c,%01d,%02d,%01.1f*%02d\n",temp_single_tbr_msg_buf,lat_deg,lat_min,lat_dir,long_deg,long_min,long_dir,fixType,nav_data.numSV,p_dop,checksum);
				//put appended message in SD card buffer
			for(inner_loop_var=0;inner_loop_var<100;inner_loop_var++){
				tbr_sd_card_buf[inner_loop_var+tbr_sd_card_buf_offset]=temp_single_appended_msg_buf[inner_loop_var];
				if(temp_single_appended_msg_buf[inner_loop_var]=='\n'){
					tbr_sd_card_buf_offset+=inner_loop_var+1;	//\n addedd already
					break;
				}
			}
			//debug_str(temp_single_appended_msg_buf);
		}
	}
}
		/*
		 * public functions
		 */
bool app_manager_init(void){
	  uint8_t					init_retry=0;
	  bool						temp_init_flag=false;

	  	  	  	  //basic initializations
	  rs232_init();
	  rs232_enable();
	  delay_init();
	  	  	  	  //Turn on init. LED (blue)
	  set_status_led(false,true);

#ifdef USE_GPS
	  init_retry=0;
	  do{
		  temp_init_flag= gps_init();
			 init_retry++;
			 if(init_retry>INIT_RETRIES){
					sprintf((char *)rs232_tx_buf,"GPS INIT FAILED\n");
					debug_str(rs232_tx_buf);
					set_status_led(true,false);
					return 0;
			 }
	  	  }while(!temp_init_flag);
	sprintf((char *)rs232_tx_buf,"GPS Init. DONE\n");
	debug_str(rs232_tx_buf);
#endif

#ifdef USE_SD_CARD
	 do{
		 temp_init_flag=sd_card_init();
			 init_retry++;
			 if(init_retry>INIT_RETRIES){
					sprintf((char *)rs232_tx_buf,"Init. SD Card Failed\n");
					debug_str((unsigned char *)rs232_tx_buf);
					set_status_led(true,false);
					return 0;
			 }
		 }while(!temp_init_flag);

	init_retry=0;
	 do{
		 temp_init_flag=(bool)f_mount(&FatFs,"", 0);
			 init_retry++;
			 if(init_retry>INIT_RETRIES){
					sprintf((char *)rs232_tx_buf,"Mounting SD Card Failed\n");
					debug_str((unsigned char *)rs232_tx_buf);
					set_status_led(true,false);
					return false;
			 }
		 }while(temp_init_flag);
		//New session...
	sprintf((char *)rs232_tx_buf,"\n\t\t\t****New session****\n");
	debug_file_sys_setup((char *)"debug",(char *)rs232_tx_buf);

#else
	debug_str((const char*)"\t\tNOT using SD Card\n");
#endif

#ifdef USE_TBR
	tbr_init();
	init_retry=0;
	 do{
		 temp_init_flag=	tbr_send_cmd(cmd_sn_req,(time_t) 0);
			 init_retry++;
			 if(init_retry>INIT_RETRIES){
					sprintf((char *)rs232_tx_buf,"TBR Missing....\n");
					debug_str((unsigned char *)rs232_tx_buf);
					break;
			 }
		 }while(!temp_init_flag);
#endif
	 	 	 //Turn off init. LED
	 set_status_led(false,false);
	 return true;
}

void app_manager_tbr_synch_msg(uint8_t  time_manager_cmd, nav_data_t ref_timestamp_local_function, nav_data_t running_tstamp_local_function,int diff){

	bool			temp_flag=false;
	int				tbr_msg_count=0;
	int				tbr_msg_length=0;
	char			tbr_msg_buf[ARRAY_MESSAGE_SIZE];

	static uint32_t	cum_tbr_msg_count=0;
	static bool		sent_adv_synch=false;

	if(time_manager_cmd==0){
		temp_flag=tbr_cmd_update_rgb_led(cmd_basic_sync,(time_t)ref_timestamp_local_function.gps_timestamp);
	}
	else if (time_manager_cmd==1){
		if(sent_adv_synch){
			temp_flag=tbr_cmd_update_rgb_led(cmd_basic_sync,(time_t)ref_timestamp_local_function.gps_timestamp);
		}else{
			  temp_flag=tbr_cmd_update_rgb_led(cmd_advance_sync,(time_t)ref_timestamp_local_function.gps_timestamp);
			  sent_adv_synch=true;
		}
#ifdef SD_CARD_ONLY
	  tbr_msg_count=tbr_recv_msg((char *)tbr_msg_buf,&tbr_msg_length);
	  if(tbr_msg_count>0){
		if(running_tstamp_local_function.valid==true){
			append_gps_status(tbr_msg_buf,tbr_msg_count, running_tstamp_local_function);
		}
		else{
			append_gps_status(tbr_msg_buf,tbr_msg_count, ref_timestamp_local_function);
		}
			//log file entry
		sprintf((char *)rs232_tx_buf,"TBR_Flag=%d\tRef_TS=%ld\tRunning_TS=%10d\tnano=%ld\ttAcc=%ld\tTBR message count=%d\tCumulative detections=%d\tCumulative temperature=%d\n",temp_flag,ref_timestamp_local_function.gps_timestamp,running_tstamp_local_function.gps_timestamp,running_tstamp_local_function.nano,running_tstamp_local_function.tAcc,tbr_msg_count,cum_detections_counter,cum_temp_counter);
		temp_flag=debug_file_sys_setup((char *)"debug",(char *)rs232_tx_buf);
			//store buffer on SD card
		temp_flag=log_file_sys_setup(ref_timestamp_local_function.year,ref_timestamp_local_function.month,ref_timestamp_local_function.day,tbr_sd_card_buf);
		if(temp_flag==false){
			sprintf((char *)rs232_tx_buf,"Wrt Flg=%1d Lngth=%3d write failed\n",temp_flag,tbr_msg_length);
			debug_str(rs232_tx_buf);
		}
	  }
#elif BOTH_RADIO_SD_CARD
	  tbr_msg_count=tbr_recv_msg_uint(tbr_lora_buf,&tbr_lora_length,tbr_msg_buf,&tbr_msg_length);
	  if(tbr_msg_count>0){
		if(running_tstamp_local_function.valid==true){
			append_gps_status(tbr_msg_buf,tbr_msg_count, running_tstamp_local_function);
		}
		else{
			append_gps_status(tbr_msg_buf,tbr_msg_count, ref_timestamp_local_function);
		}
				//log file entry
		cum_tbr_msg_count+=tbr_msg_count;
		sprintf((char *)rs232_tx_buf,"TBR_Flag=%d\tRef_TS=%ld\tRunning_TS=%10d\tnano=%d\tTBR_message_count=%d\tCumulative_detections=%d\tCumulative_temperature=%d\t\n",temp_flag,ref_timestamp_local_function.gps_timestamp,running_tstamp_local_function.gps_timestamp,running_tstamp_local_function.nano,cum_tbr_msg_count,cum_detections_counter,cum_temp_counter);
		temp_flag=debug_file_sys_setup((char *)"debug",(char *)rs232_tx_buf);
				//store buffer on SD card
		temp_flag=log_file_sys_setup(ref_timestamp_local_function.year,ref_timestamp_local_function.month,ref_timestamp_local_function.day,tbr_sd_card_buf);
		if(temp_flag==false){
			sprintf((char *)rs232_tx_buf,"Wrt Flg=%1d Lngth=%3d write failed\n",temp_flag,tbr_msg_length);
			debug_str(rs232_tx_buf);
		}
	  }
#elif RADIO_ONLY
		  tbr_msg_count=tbr_recv_msg_uint(tbr_lora_buf,&tbr_lora_length,tbr_msg_buf,&tbr_msg_length);
#endif

	  }
	  else{
		  ;
	  }

	return;
}

nav_data_t app_manager_get_nav_data(void){
	return gps_get_nav_data();
}
uint8_t	app_manager_get_lora_buffer(uint8_t	*lora_buffer){
	int				loop_var=0;
	uint8_t			lora_buf_length=0;
	uint8_t			offset=0;

	if(tbr_lora_length>0){
		if ((lora_buffer[1] & 0x03) == 0x01){offset=12-2;}
		for(loop_var=0;loop_var<tbr_lora_length;loop_var++){
			lora_buffer[loop_var+offset]=tbr_lora_buf[loop_var];
			//sprintf((char *)rs232_tx_buf,"lora_buffer[loop_var+offset (%d+%d)]=%1d\n", loop_var, offset, lora_buffer[loop_var+offset]);
			//debug_str(rs232_tx_buf);
		}
		lora_buf_length=(uint8_t)tbr_lora_length + offset;
		tbr_lora_length=0;
		return lora_buf_length;
	}
	else{
		return 0;
	}
}

bool debug_file_sys_setup(char *debug_name, char buf[]){

	char 			filename[32]="00000000";
	bool			ret_flag=false;
	FIL  			f_pointer;
	FRESULT 		f_ret;

	f_mount(&FatFs,"", 1);
	sprintf(filename,"%s.txt",debug_name);
	f_ret = f_open(&f_pointer, filename, FA_WRITE | FA_OPEN_APPEND);
	if(f_ret==FR_OK){
    	f_ret = f_puts(buf,&f_pointer);
    	f_close(&f_pointer);
    	if(f_ret>0){
    		ret_flag=true;
    	}
    	else{
    		ret_flag=false;
    	}
    }
    else{
    	ret_flag=false;
		sprintf((char *)rs232_tx_buf,"f_open=%1d and name=%s\n",f_ret,filename);
		debug_str((unsigned char *)rs232_tx_buf);
		f_close(&f_pointer);
    }
    return ret_flag;
}
