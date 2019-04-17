/*
 * ublox_gps.c
 *
 *  Created on: Apr 25, 2017
 *      Author: waseemh
 */
#include "../devices_header/ublox_gps.h"
	/*
	 *private variables
	 */

	/*
	 *public variables
	 */

	/*
	 *private functions
	 */
bool			send_cmd_rx_ack(uint8_t const *cmd,uint8_t size_cmd){
	int 		outer_loop_var=0;
	int 		inner_loop_var=0;
	int			retry=0;
	uint8_t		reply=0;
	bool 		ret_flag=false;
	bool 		break_flag=false;
	spi_cs_clear(gps);
	for(outer_loop_var=0;outer_loop_var<50;outer_loop_var++){
		for(inner_loop_var=0;inner_loop_var<size_cmd;inner_loop_var++){
			spi_read_write_byte(cmd[inner_loop_var]);
		}
		retry=0;
		while(1){
			reply=spi_read_byte();
			if(reply==0xB5){
				for(inner_loop_var=0;inner_loop_var<9;inner_loop_var++){
					reply=spi_read_byte();
					if(inner_loop_var==2){
						if(reply==0x01){
							ret_flag=true;
						}
						else{
							ret_flag=false;
						}
					}
					delay_ms(2);
				}
				break_flag=true;
				break;
			}
			else{retry++;}
			if(retry>RETRY){
				ret_flag=false;
				break;
			}
			delay_ms(2);
		}
		if(break_flag==true){break;}
	}
	if(outer_loop_var>=50){ret_flag=false;}
	spi_cs_set(gps);
	delay_ms(2);
	spi_cs_clear(gps);
	spi_read_write_byte(0xFF);
	return ret_flag;
}
uint16_t		fletcher16( uint8_t const *data, size_t size ) {
	 uint16_t crc_a = 0;
	 uint16_t crc_b = 0;
    if (size > 0) {
        do {
            crc_a += *data++;
            crc_b += crc_a;
        } while (--size);
        crc_a &= 0xff;
        crc_b &= 0xff;
    }
    return (crc_a | (crc_b << 8));
}
bool 			port_config(void){
	bool 					flag_spi=false;
	bool 					flag_usb=false;
	bool 					flag_ddc=false;
		//spi port
	flag_spi=send_cmd_rx_ack(cfg_prt_spi,(sizeof(cfg_prt_spi)/sizeof(uint8_t)));
		//usb poprt
	//spi port
	flag_usb=send_cmd_rx_ack(cfg_prt_usb,(sizeof(cfg_prt_usb)/sizeof(uint8_t)));
		//ddc port
	//spi port
	flag_ddc=send_cmd_rx_ack(cfg_prt_ddc,(sizeof(cfg_prt_ddc)/sizeof(uint8_t)));
	if(flag_spi==true && flag_usb==true && flag_ddc==true){
		return true;
	}
	else{
		return false;
	}
}
uint8_t 		receiver_nav_status(void){
	uint8_t 				reply=0;
	int 					inner_loop_var=0;
	int						outer_loop_var=0;
	int 					retry=0;
	uint8_t					fix=0;
	bool					flag_success=false;
	spi_cs_set(gps);
	delay_ms(2);
	spi_cs_clear(gps);
	spi_read_write_byte(0xFF);
	for(outer_loop_var=0;outer_loop_var<50;outer_loop_var++){
		for(inner_loop_var=0;inner_loop_var<(sizeof(nav_pvt_gps_data)/sizeof(uint8_t));inner_loop_var++){
			spi_read_write_byte(nav_pvt_gps_data[inner_loop_var]);
		}
		retry=0;
		while(1){
			reply=spi_read_byte();
			if(reply==0xB5){
				for(inner_loop_var=1;inner_loop_var<(88+1);inner_loop_var++){
					reply=spi_read_byte();
					if(inner_loop_var-6==11){
						fix=(reply<<4);
					}
					if(inner_loop_var-6==20){
						fix|=(0x0F & reply);
					}
					delay_ms(2);
				}
				flag_success=true;
				break;
			}
			else {retry++;}
			if(retry>RETRY){
				break;
			}
			delay_ms(2);
		}
		if(flag_success==true){break;}
	}
	if(outer_loop_var>=50){fix=0xFF;}
	spi_cs_set(gps);
	return fix;
}
bool			config_low_power(void){
	bool 					flag=false;

	flag=send_cmd_rx_ack(cfg_pm2_on_off,(sizeof(cfg_pm2_on_off)/sizeof(uint8_t)));
	return flag;
}
bool			enter_low_power(void){
	bool 					flag=false;

	flag=send_cmd_rx_ack(cfg_rxm_psm_mode,(sizeof(cfg_rxm_psm_mode)/sizeof(uint8_t)));
	return flag;
}
bool			disable_sbas(void){
	bool 					flag=false;
	flag=send_cmd_rx_ack(cfg_sbas_disable,(sizeof(cfg_sbas_disable)/sizeof(uint8_t)));
	return flag;
}

bool			poll_psm(void){
	bool 					flag=false;
	uint8_t 				reply=0;
	int 					inner_loop_var=0;
	int						outer_loop_var=0;
	int 					retry=0;
	bool					flag_success=false;
	////////////////////////////////////////
	  const unsigned char  		rs232_tx_buf[64];
	  ////////////////////////////////////////
	spi_cs_set(gps);
	delay_ms(2);
	spi_cs_clear(gps);
	spi_read_write_byte(0xFF);
	for(outer_loop_var=0;outer_loop_var<50;outer_loop_var++){
		for(inner_loop_var=0;inner_loop_var<(sizeof(cfg_rxm_poll_psm)/sizeof(uint8_t));inner_loop_var++){
			spi_read_write_byte(cfg_rxm_poll_psm[inner_loop_var]);
		}
		retry=0;
		while(1){
			reply=spi_read_byte();
			if(reply==0xB5){
				for(inner_loop_var=0;inner_loop_var<9;inner_loop_var++){
					reply=spi_read_byte();
					sprintf((char *)rs232_tx_buf,"Char=%2x\n",reply);
					rs232_transmit_string(rs232_tx_buf,8);
					delay_ms(5);
					if(inner_loop_var==6){
						if(reply==0x01){
							flag=true;
						}
						else{
							flag=false;
						}
					}
					delay_ms(2);
				}
				flag_success=true;
				break;
			}
			else {retry++;}
			if(retry>RETRY){
				break;
			}
			delay_ms(2);
		}
		if(flag_success==true){break;}
	}
	if(outer_loop_var>=50){flag=false;}
	spi_cs_set(gps);
	return flag;
}
nav_data_t		parse_message(uint8_t data[]){
	nav_data_t 		nav_data;
	uint32_t		gps_timestamp=0;
	uint32_t		tAcc=0;	//added later on
	uint32_t		nano=0;	//added later on
	uint16_t		year=0;
	uint32_t		longitude=0;
	uint32_t		latitude=0;
	uint32_t		height=0;
	uint8_t			offset=6;
	uint16_t		pdop=0;

		//extract GPS TimeStamp
	gps_timestamp|=(data[offset+3]<<24);
	gps_timestamp|=(data[offset+2]<<16);
	gps_timestamp|=(data[offset+1]<<8);
	gps_timestamp|=data[offset+0];
	nav_data.gps_timestamp=(uint32_t)(gps_timestamp);
		//extract date
	year=data[offset+5];
	year=year<<8;
	year|=data[offset+4];
	nav_data.year=year;
	nav_data.month=data[offset+6];
	nav_data.day=data[offset+7];
		//extract time
	nav_data.hour=data[offset+8];
	nav_data.min=data[offset+9];
	nav_data.sec=data[offset+10];
		//extract timing accuracy
	nav_data.t_flags=data[offset+11];		//added later on
	tAcc|=(data[offset+15]<<24);			//added later on
	tAcc|=(data[offset+14]<<16);			//added later on
	tAcc|=(data[offset+13]<<8);				//added later on
	tAcc|=data[offset+12];					//added later on
	nav_data.tAcc=(uint32_t)(tAcc);			//added later on
	nano|=(data[offset+19]<<24);			//added later on
	nano|=(data[offset+18]<<16);			//added later on
	nano|=(data[offset+17]<<8);				//added later on
	nano|=data[offset+16];					//added later on
	nav_data.nano=(uint32_t)(nano);			//added later on
	nav_data.fix=data[offset+20];			//added later later later on
	nav_data.numSV=data[offset+23];			//added later on
	pdop|=(data[offset+77]<<8);
	pdop|=data[offset+76];
	nav_data.pDOP=(uint16_t)(pdop);			//added later later on
		//extract longitude
	longitude=0;
	longitude|=(data[offset+27]<<24);
	longitude|=(data[offset+26]<<16);
	longitude|=(data[offset+25]<<8);
	longitude|=data[offset+24];
	nav_data.longitude=(uint32_t)(longitude);
		//extract latitude
	latitude=0;
	latitude|=(data[offset+31]<<24);
	latitude|=(data[offset+30]<<16);
	latitude|=(data[offset+29]<<8);
	latitude|=(data[offset+28]);
	nav_data.latitude=(uint32_t)(latitude);
		//extract latitude
	height=0;

	height|=(data[offset+39]<<24);
	height|=(data[offset+38]<<16);
	height|=(data[offset+37]<<8);
	height|=data[offset+36];
	nav_data.height=(uint32_t)(height);
		//status of gps
	if(data[offset+20]==0x01 || data[offset+20]==0x02 || data[offset+20]==0x03
	|| data[offset+20]==0x04 || data[offset+20]==0x05){
		nav_data.valid=true;
	}
	else{
		nav_data.valid=false;
	}
	return nav_data;
}

	/*
	* public functions
	*/
bool 			gps_init(void){
	uint8_t 				reply=0;
	bool					flag_port=false;
	bool					flag_init=false;
	bool					flag_config_lp=false;
	bool					flag_sbas=false;
	bool					flag_enter_lp=false;
	bool					flag_select_gps=false;
	bool					flag_config_tp=false;
	const unsigned char  	rs232_tx_buf[64];
	nav_data_t				start_fix;

	spi_init();
	//GPIO_PinModeSet(PWR_EN_PORT,GPS_PWR_EN,gpioModePushPull,0);
	GPIO_PinModeSet(GPS_SIG_PORT, GPS_INT, gpioModePushPull,0);
	//gps_on();
					//configure ports
	flag_port=port_config();
	flag_select_gps=send_cmd_rx_ack(cfg_gnss,(sizeof(cfg_gnss)/sizeof(uint8_t)));
	flag_config_tp=send_cmd_rx_ack(cfg_tp5,(sizeof(cfg_tp5)/sizeof(uint8_t)));
	reply=0x00;
					//wait for fix
	gps_poll_nav_status();
	while(1){
		start_fix=gps_get_nav_data();
		sprintf((char *)rs232_tx_buf,"GPS Nav Status=%d\n",start_fix.fix);
		rs232_transmit_string(rs232_tx_buf,strlen((char *)rs232_tx_buf));
		if(!start_fix.valid){
			reply++;
			if(reply>15){
				gps_poll_nav_status();
				reply=0;
				delay_ms(9);
			}
		}
		if(start_fix.fix==3){
			flag_init=true;
			break;
		}
		delay_ms(9);
	}
					//configure and enter low power
	//flag_config_lp=config_low_power();
	flag_sbas=disable_sbas();
	/*flag_enter_lp=enter_low_power();
					//grid search delay....
	for(loop_var=0;loop_var<25;loop_var++){
		 delay_ms(10);
	 }
					//poll PSM and exit if OK
	 for(loop_var=0;loop_var<60;loop_var++){
		 enter_low_power();
		 if(poll_psm()){break;}
		 delay_ms(9);
	 }
	if(flag_port==true && flag_init==true && flag_config_lp==true && flag_sbas && flag_enter_lp){
		return true;
	}
	else{
		return false;
	}*/
	if(flag_port==true && flag_sbas==true && flag_init==true && flag_config_tp==true && flag_select_gps==true){
		return true;
	}
	else{
		return false;
	}
}
void 	gps_poll_nav_status (void){
	int 					inner_loop_var=0;

		spi_cs_clear(gps);
		spi_read_write_byte(0xFF);
		for(inner_loop_var=0;inner_loop_var<(sizeof(nav_pvt_gps_data)/sizeof(uint8_t));inner_loop_var++){
			spi_read_write_byte(nav_pvt_gps_data[inner_loop_var]);
		}
		spi_cs_set(gps);
		delay_ms(2);

		return;
}

nav_data_t 		gps_get_nav_data (void){
	nav_data_t 				nav_data;
	uint8_t 				reply=0;
	int 					inner_loop_var=0;
	int 					retry=0;
	bool					flag_success=false;
	uint8_t					msg_buf[90];

		spi_cs_clear(gps);
		spi_read_write_byte(0xFF);
		retry=0;
		flag_success=true;
		while(spi_read_byte()!=0xB5){
			retry++;
			if(retry>255){
				flag_success=false;
				break;
			}
			delay_ms(0);
		}
		if(flag_success){
			msg_buf[0]=0xB5;
			for(inner_loop_var=1;inner_loop_var<(88+1);inner_loop_var++){
				reply=spi_read_byte();
				msg_buf[inner_loop_var]=reply;
			}
			nav_data=parse_message(msg_buf);
			if(nav_data.valid==false){
				nav_data.valid=false;
				nav_data.year=0;
				nav_data.month=0;
				nav_data.day=0;
				nav_data.hour=0;
				nav_data.min=0;
				nav_data.sec=0;
				nav_data.latitude=0;
				nav_data.longitude=0;
				nav_data.height=0;
			}
		}
		else {
			nav_data.valid=false;
			nav_data.year=0;
			nav_data.month=0;
			nav_data.day=0;
			nav_data.hour=0;
			nav_data.min=0;
			nav_data.sec=0;
			nav_data.latitude=0;
			nav_data.longitude=0;
			nav_data.height=0;
		}
		spi_cs_set(gps);
	return nav_data;
}
void 			gps_on(void){
	//GPIO_PinOutSet(PWR_EN_PORT,GPS_PWR_EN);
}

void 			gps_off(void){
	//GPIO_PinOutClear(PWR_EN_PORT,GPS_PWR_EN);
}

void 			gps_int_pin_toggle(void){
	GPIO_PinOutSet(GPS_SIG_PORT, GPS_INT);
	delay_ms(1);
	GPIO_PinOutClear(GPS_SIG_PORT, GPS_INT);
	delay_ms(1);
}
