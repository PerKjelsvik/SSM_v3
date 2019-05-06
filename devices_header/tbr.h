/*
 * tbr.h
 *
 *  Created on: Mar 29, 2017
 *      Author: waseemh
 */

#ifndef SRC_TBR_H_
#define SRC_TBR_H_

#include "../drivers_header/rs485.h"
#include "../drivers_header/delay.h"

#define ARRAY_MESSAGE_SIZE		1024//512
#define CMD_RX_TX_BUF_SIZE		1024//256
#define	CMD_TX_BUF_SIZE			16
#define TBR_SENSOR_MSG 			true
#define TBR_DETECION_MSG		false

typedef struct {
		//First delimiter
	uint16_t	tbrID;
		//Second delimiter
	uint8_t		timeDiff;
		//3rd delimiter
	uint16_t	millisec;
		//4th delimiter
	uint8_t		CodeType;
	uint16_t	Temperature;
		//5th delimiter
	uint32_t	CodeID;
	uint8_t		Noise;
		//6th delimiter
	uint16_t	CodeData;
	uint8_t		NoiseLP;
		//7th delimiter
	uint8_t		frequency;
	uint8_t		SNR;
}tbr_msesage_t;

	/*
	 * private functions
	 */
void 		array_add(char data);
void		clear_buffer(char *buf, uint16_t size);
bool 		array_is_empty(void);
bool 		array_is_full(void);
bool 		check_other_messages(char * cmd_buf);
bool 		get_and_compare(char *compare_string);
uint8_t	  	CalculateLuhn(time_t * time);
char 		array_remove(void);
int			parse_message_tbr(char *buffer);
void 		shift_Elements(char *arr, int size, int positions);
	/*
	 * public functions
	 */
void 		tbr_init(void);
void 		tbr_shutdown(void);
bool 		tbr_send_cmd(tbr_cmd_t tbr_cmd,time_t timestamp);
bool 		tbr_send_synch_msg(char *sync_msg);					//NOT used
uint8_t 	tbr_tag_count(void);
uint8_t		tbr_recv_msg(char *msg_buf, int *msg_length);
uint8_t 	tbr_recv_msg_uint(uint8_t *lora_msg_buf, int *lora_length,
							char *msg_buf, int *msg_length);
#endif /* SRC_TBR_H_ */
