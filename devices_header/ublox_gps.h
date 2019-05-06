/*
 * ublox_gps.h
 *
 *  Created on: Apr 25, 2017
 *      Author: waseemh
 */

#ifndef SRC_UBLOX_GPS_H_
#define SRC_UBLOX_GPS_H_

#include "../drivers_header/spi.h"
#include "../drivers_header/delay.h"
#include "../drivers_header/rs232.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ublox_msg.h"
/*
 * Macros
 */
#define 	RETRY	25
/*
 * structs and typedefs
 */
typedef struct {
	bool		valid;
	uint32_t	gps_timestamp;
	uint16_t	year;
	uint8_t		month;
	uint8_t		day;
	uint8_t		hour;
	uint8_t		min;
	uint8_t		sec;
	uint8_t		t_flags;	//added later on
	uint32_t	tAcc;		//added later on
	uint32_t	nano;		//added later on
	uint8_t		numSV;		//added later on
	uint16_t	pDOP;		//added later later on
	uint32_t	longitude;
	uint32_t	latitude;
	uint32_t	height;
	uint8_t		fix;		//added later later later on

}nav_data_t;
/*
 * public variables
 */
/*
 * private functions
 */
bool			send_cmd_rx_ack(uint8_t const *cmd,uint8_t size_cmd);
bool 			port_config(void);
bool			disbale_sbas(void);
bool			config_low_power(void);
bool			enter_low_power(void);
bool			poll_psm(void);
uint8_t 		receiver_nav_status(void);
uint16_t		fletcher16( uint8_t const *data, size_t bytes );
nav_data_t		parse_message(uint8_t data[]);
/*
 * public functions
 */
void 			gps_int_pin_toggle(void);
void 			gps_on(void);
void 			gps_off(void);
bool			gps_init(void);
bool			gps_status(void);
bool 			gps_low_power(void);
void 			gps_poll_nav_status (void);
nav_data_t 		gps_get_nav_data(void);
#endif /* SRC_UBLOX_GPS_H_ */
