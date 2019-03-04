/*
 * tbr.c
 *
 *  Created on: Mar 30, 2017
 *      Author: waseemh
 */


#include "../devices_header/tbr.h"

	/*
	 * private variables
	 */
static	char		array_msg[ARRAY_MESSAGE_SIZE];
static 	int			array_size=0;
static 	int			array_front=0;
static 	int			array_rear=0;
static 	bool		incomplete_ack_flag=false;
static 	uint8_t		tbr_backoff_delay=4;
	/*
	 * shared global varibales
	 */
uint32_t 			cum_detections_counter=0;
uint32_t 			cum_temp_counter=0;
	/*
	 * private functions
	 */
bool array_is_full(void){
	 if(array_size==ARRAY_MESSAGE_SIZE){
		 return true;
	 }
	 else{
		 return false;
	 }
 }
bool array_is_empty(void){
	 if(array_size==0){
		 return true;
	 }
	 else{
		 return false;
	 }
 }
void array_add(char data){
	 if(array_size<ARRAY_MESSAGE_SIZE){
		 array_msg[array_rear]=data;
		 array_rear++;
		 array_size++;
		 if(array_rear==ARRAY_MESSAGE_SIZE){array_rear=0;}
	 }
 }
char array_remove(void){
	char	temp_char;
	 if(array_size>0){
		 temp_char=array_msg[array_front];
		 array_front++;
		 array_size--;
		 if(array_front==ARRAY_MESSAGE_SIZE){array_front=0;}
		 return temp_char;
	 }
	 else{
		 return '0';
	 }
}
uint8_t  CalculateLuhn(time_t * time){

	int digit, counter;
	uint8_t timeStampArray[9];
	uint32_t luhn_sum = 0;

	time_t timestamp = *time / 10; /* Cut last digit */

	/* Make an array of digits of the time stamp */
	for (counter = 9; counter >= 1; counter--){
		digit = timestamp % 10;
		timestamp = timestamp / 10;
		timeStampArray[counter - 1] = digit;
	}

	/* Calculate luhn sum */
	for (int i = 0; i < 9; i++){

		if (i % 2 == 0){
		timeStampArray[i] += timeStampArray[i];
		}

		if (timeStampArray[i] >= 10){
			luhn_sum += timeStampArray[i] % 10 + 1;
	}

		else luhn_sum += timeStampArray[i];
	}

	return (luhn_sum * 9) % 10 + '0';
}

void shift_Elements(char *arr, int size, int positions){
        for (int i=size-1; i>=positions; i--){
            arr[i] = arr[i-positions];
    }
        return;
}

int		parse_message_tbr(char *buffer){
	int				loop_var=0;
	int				token_length=0;
	char			*token_str;
	char			ref_token[2]="\r";
	static 	char 	broken_msg_buf[64];
	static 	bool	last_broken_message_flag=false;
	static 	uint8_t	last_broken_message_size=0;

	uint32_t		inner_loop_var=0;
	uint32_t		outer_loop_var=0;
	uint8_t			n_complete_messages=0;
	uint8_t			n_converted_tokens=0;
	bool			found_partial_msg=false;

	if(last_broken_message_flag==true){
		shift_Elements(buffer, CMD_RX_TX_BUF_SIZE ,last_broken_message_size);
		for(loop_var=0;loop_var<last_broken_message_size;loop_var++){
			buffer[loop_var]=broken_msg_buf[loop_var];
		}
		last_broken_message_flag=false;
		last_broken_message_size=0;
	}
	n_complete_messages=0;
	found_partial_msg=false;
	for(outer_loop_var=0;outer_loop_var<strlen(buffer);outer_loop_var++){
		if(buffer[outer_loop_var]=='$' || buffer[outer_loop_var]=='a'){
			for(inner_loop_var=1;inner_loop_var<64;inner_loop_var++){		//64 is also danger!!!
				if(buffer[outer_loop_var+inner_loop_var]=='\r'){
					n_complete_messages++;
					outer_loop_var+=inner_loop_var;
					break;
				}
				if(inner_loop_var==63){
					found_partial_msg=true;
				}
			}
		}
		if(found_partial_msg==true){
			break;
		}
	}

	last_broken_message_size=0;
	last_broken_message_flag=false;
	clear_buffer(broken_msg_buf,64);
	if(outer_loop_var<strlen(buffer)){
		for(loop_var=outer_loop_var;loop_var<strlen(buffer);loop_var++){
			broken_msg_buf[loop_var-outer_loop_var]=buffer[loop_var];
		}
		last_broken_message_flag=true;
		last_broken_message_size=loop_var-outer_loop_var;
		 //debug_str("\t\t\t\tParse found and added half message...");
		 //debug_str(broken_msg_buf);
		 //debug_char('\n');
	}
	n_converted_tokens=0;

	if(n_complete_messages>0){
		token_str=strtok(buffer,ref_token);
		if(token_str==NULL){
			debug_str("Parse DANGER, complete messages >0 BUT no token found!!!!\n");
			return -1;
		}
		else{
			if(!array_is_full()){
				while(token_str!=NULL){
					if(token_str[0]=='$'){
						token_length=strlen(token_str);
						if(token_length>50){
							debug_str("\t\t\tParse DANGER, string length larger than 50!!!");
							debug_str(token_str);
							debug_char('\n');
						}
						for(loop_var=0;loop_var<token_length;loop_var++){
							array_add(token_str[loop_var]);
						}
						array_add('\n');
					}
					else if(token_str[0]=='a') {
						 incomplete_ack_flag=true;
					}
					else{
						 //debug_str("\t\t\t!!!Parse discarding unknown message type!!!");
						 //debug_str(token_str);
						 //debug_char('\n');
					}
					n_converted_tokens++;
					if(n_converted_tokens>=n_complete_messages){
						break;
					}
					token_str=strtok(NULL,ref_token);
				}
				return 1;
			}
			else{
				return -1;
			}
		}
	}
	else{
		return 0;
	}
}

bool check_other_messages(char * cmd_rx_tx_buf){
	return parse_message_tbr(cmd_rx_tx_buf);
}
void clear_buffer(char *buf, uint16_t size){
	int			loop_var=0;
	for(loop_var=0;loop_var<size;loop_var++){
		buf[loop_var]=0;
	}
	return;
}
char            reusable_buffer[128];
bool get_and_compare(char *compare_string){
	char 			*cmd_compare_str;
	char 			cmd_rx_tx_buf[CMD_RX_TX_BUF_SIZE];
	int				loop_var=0;
	char			temp_char='0';
	bool			ret_flag=false;

	clear_buffer(cmd_rx_tx_buf,CMD_RX_TX_BUF_SIZE);
	tbr_backoff_delay=8;
	delay_ms(tbr_backoff_delay);												//response time from TBR

	for(loop_var=0;loop_var<FIFO_TBR_RX_DATA_SIZE;loop_var++){
		temp_char=rs485_recieve_char();
		if(temp_char=='@'){break;}
		cmd_rx_tx_buf[loop_var]=temp_char;
	}
	cmd_compare_str=strstr(cmd_rx_tx_buf,(const char *)compare_string);
	if(cmd_compare_str!=NULL){
		ret_flag=true;
	}
	 else{
		ret_flag=false;
	 }
	incomplete_ack_flag=false;
	check_other_messages(cmd_rx_tx_buf);
	 /*if(ret_flag==false){
		 if(incomplete_ack_flag==true){
			 ret_flag=true;
		 }
	 }*/
	return ret_flag;
}

uint8_t convert_single_tbr_msg_into_uint(char *single_msg, uint8_t *dst_buf, uint8_t offset, uint32_t *first_timestamp){
	char			*temp_ptr;
	char			ref_token[2]=",";
	char			*token;
	bool			message_type=TBR_DETECION_MSG;
	tbr_msesage_t	tbr_message;
	int				diff_freq=0;
	uint8_t			detection_freq=0;
	uint8_t			code_type_unsigned=0;
  uint8_t     buf_index=0;
	uint8_t 		protocol=0;
	bool				diff_flag=true;

		//$000xxx
	token = strtok(single_msg, ref_token);
		//timestamp
  token = strtok(NULL, ref_token);
  if(*first_timestamp > 0){
    tbr_message.timeDiff=(uint8_t)((uint32_t)strtoul(token,&temp_ptr,10) - *first_timestamp);
  }else
  {
    *first_timestamp=(uint32_t)strtoul(token,&temp_ptr,10);
		diff_flag=false;
  }
		//TBR Sensor or millisec
	token = strtok(NULL, ref_token);
	if(strncmp(token,(char *)"TBR Sensor",5)==0){
		message_type=TBR_SENSOR_MSG;
	}
	else
	{
		tbr_message.millisec=(uint16_t)strtoul(token,&temp_ptr,10);
	}
		//Codetype or Temperature
	token = strtok(NULL, ref_token);
	if(message_type==TBR_DETECION_MSG){
		if((strncmp(token,(char *)"R256",4)==0) || (strncmp(token,(char *)"r256",4)==0))
			tbr_message.CodeType=00;
		else if((strncmp(token,(char *)"R04K",4)==0) || (strncmp(token,(char *)"r04k",4)==0) || (strncmp(token,(char *)"R04k",4)==0))
			tbr_message.CodeType=01;
		else if((strncmp(token,(char *)"R64K",4)==0) || (strncmp(token,(char *)"r64k",4)==0) || (strncmp(token,(char *)"R64k",4)==0))
			tbr_message.CodeType=02;
		else if((strncmp(token,(char *)"S256",4)==0) || (strncmp(token,(char *)"s256",4)==0))
			tbr_message.CodeType=03;
		else if((strncmp(token,(char *)"R01M",4)==0) || (strncmp(token,(char *)"r01m",4)==0) || (strncmp(token,(char *)"R01m",4)==0))
			tbr_message.CodeType=04;
		else if((strncmp(token,(char *)"S64K",4)==0) || (strncmp(token,(char *)"s64k",4)==0) || (strncmp(token,(char *)"S64k",4)==0))
			tbr_message.CodeType=05;
		else if((strncmp(token,(char *)"H256",4)==0) || (strncmp(token,(char *)"h256",4)==0))
			tbr_message.CodeType=06;
		else if((strncmp(token,(char *)"D256",4)==0) || (strncmp(token,(char *)"d256",4)==0))
			tbr_message.CodeType=07;
		else
			tbr_message.CodeType=0xFE;
		protocol=tbr_message.CodeType;
	}
	else{
		tbr_message.Temperature=(uint16_t)strtoul(token,&temp_ptr,10);
	}
		//CodeID or Noise
	token = strtok(NULL, ref_token);
	if(message_type==TBR_DETECION_MSG){
		tbr_message.CodeID=(uint32_t)strtoul(token,&temp_ptr,10);
	}
	else{
		tbr_message.Noise=(uint8_t)strtoul(token,&temp_ptr,10);
	}
		//CodeData or NoiseLP
	token = strtok(NULL, ref_token);
	if(message_type==TBR_DETECION_MSG){
		tbr_message.CodeData=(uint16_t)strtoul(token,&temp_ptr,10);
	}
	else{
		tbr_message.NoiseLP=(uint8_t)strtoul(token,&temp_ptr,10);
	}
		//SNR or Frequency
	token = strtok(NULL, ref_token);
	if(message_type==TBR_DETECION_MSG){
		tbr_message.SNR=(uint8_t)strtoul(token,&temp_ptr,10);
	}
	else{
		tbr_message.frequency=(uint8_t)strtoul(token,&temp_ptr,10);
	}
		//change code type as per frequency
	token = strtok(NULL, ref_token);
	if(message_type==TBR_DETECION_MSG){
		detection_freq=(uint8_t)strtoul(token,&temp_ptr,10);
		diff_freq=detection_freq-69;
		if(detection_freq>69){
			code_type_unsigned=tbr_message.CodeType+(16*diff_freq);
			tbr_message.CodeType=code_type_unsigned;
		}
		else if(detection_freq<69){
			code_type_unsigned=(uint8_t)(tbr_message.CodeType+(16*(diff_freq-1)));
			tbr_message.CodeType=code_type_unsigned;
		}
		else {		//in case of 69 KHz do not change code type
			;
		}
	}
			//fill the lora buffer
  if(diff_flag){
    dst_buf[offset+0]=(uint8_t)tbr_message.timeDiff;
  }else{
    dst_buf[offset+0]=(uint8_t)(*first_timestamp>>24);
    dst_buf[offset+1]=(uint8_t)(*first_timestamp>>16);
    dst_buf[offset+2]=(uint8_t)(*first_timestamp>>8);
    dst_buf[offset+3]=(uint8_t)(*first_timestamp>>0);
    buf_index=3;
  }
	if(message_type==TBR_DETECION_MSG){
		dst_buf[offset+buf_index+1]=(uint8_t)tbr_message.CodeType;
		buf_index+=1;
		if(protocol == 0){  //R256
      dst_buf[offset+buf_index+1]=(uint8_t)(tbr_message.CodeID);
      buf_index+=1;
    }else if((protocol == 1) || (protocol == 2)){  //R04K || R64K
      dst_buf[offset+buf_index+1]=(uint8_t)(tbr_message.CodeID>>8);
      dst_buf[offset+buf_index+2]=(uint8_t)(tbr_message.CodeID>>0);
      buf_index+=2;
    }else if(protocol == 3){  //S256
      dst_buf[offset+buf_index+1]=(uint8_t)tbr_message.CodeID;
      dst_buf[offset+buf_index+2]=(uint8_t)tbr_message.CodeData;
      buf_index+=2;
    }else if(protocol == 4){  //R01M
      dst_buf[offset+buf_index+1]=(uint8_t)(tbr_message.CodeID>>16);
      dst_buf[offset+buf_index+2]=(uint8_t)(tbr_message.CodeID>>8);
      dst_buf[offset+buf_index+3]=(uint8_t)(tbr_message.CodeID>>0);
      buf_index+=3;
    }else if(protocol == 5){  //S64K
      dst_buf[offset+buf_index+1]=(uint8_t)(tbr_message.CodeID>>8);
      dst_buf[offset+buf_index+2]=(uint8_t)(tbr_message.CodeID>>0);
      dst_buf[offset+buf_index+3]=(uint8_t)tbr_message.CodeData;
      buf_index+=3;
    }else if((protocol == 6) || (protocol == 7)){  //HS256 || DS256
      dst_buf[offset+buf_index+1]=(uint8_t)tbr_message.CodeID;
      dst_buf[offset+buf_index+2]=(uint8_t)(tbr_message.CodeData>>8);
      dst_buf[offset+buf_index+3]=(uint8_t)(tbr_message.CodeData>>0);
      buf_index+=3;
    }else {  //should not happen
      ;  /* Do nothing, packet will be discarded at receiving end */
    }
		uint8_t temp_1=((tbr_message.SNR & 0x3F)<<2);
		uint8_t temp_2=((tbr_message.millisec>>8));
		dst_buf[offset+buf_index+1]=temp_1 | temp_2;
		dst_buf[offset+buf_index+2]=(uint8_t)(tbr_message.millisec>>0);
    buf_index+=2;
		cum_detections_counter++;
	}else{
		dst_buf[offset+buf_index+1]=(uint8_t)0xFF;
		dst_buf[offset+buf_index+2]=(uint8_t)(tbr_message.Temperature>>8);
		dst_buf[offset+buf_index+3]=(uint8_t)(tbr_message.Temperature>>0);
		dst_buf[offset+buf_index+4]=(uint8_t)tbr_message.Noise;
		dst_buf[offset+buf_index+5]=(uint8_t)tbr_message.NoiseLP;
		dst_buf[offset+buf_index+6]=(uint8_t)tbr_message.frequency;
		buf_index+=6;
		cum_temp_counter++;
	}
	return offset+buf_index+1;		//fixed offset=message size - 1.....*/
}
uint8_t convert_tbr_msgs_to_uint(char *src_buf, uint8_t *dst_buf, uint8_t msg_count){
	uint16_t 		inner_loop_var=0;
	uint16_t		outer_loop_var=0;
	uint16_t		offset_src_buf=0;
	uint16_t		offset_dst_buf=0;
	char 			single_msg[50];
	char			*temp_ptr;
  uint32_t first_timestamp=0;

	clear_buffer(single_msg, 50);
		//extract and convert SN to uint8_t
	for(outer_loop_var=0;outer_loop_var<10;outer_loop_var++){
		if(src_buf[outer_loop_var+1]==','){break;}
		single_msg[outer_loop_var]=src_buf[outer_loop_var+1];	//+1 to ignore $
	}
	dst_buf[0]=(uint8_t)strtoul(single_msg,&temp_ptr,10);
	dst_buf[0]=0x80;
	offset_dst_buf=1;
		//now convert rest of the messages into uint8_t (7 bytes per message => TimeStamp(4)+milli_sec(2)+tagID(1))
	offset_src_buf=0;
	if(msg_count>17){
		//sprintf(reusable_buffer, "\tMessage length =%d. NO LoRA parse\n",msg_count);
		//debug_str(reusable_buffer);
		//return 0;
		msg_count=10;
	}
	for(outer_loop_var=0;outer_loop_var<msg_count;outer_loop_var++){
		clear_buffer(single_msg, 50);
		for(inner_loop_var=0;inner_loop_var<strlen(src_buf);inner_loop_var++){
			if(src_buf[offset_src_buf+inner_loop_var]=='\n'){
				offset_src_buf+=inner_loop_var+1;
				break;
			}
			single_msg[inner_loop_var]=src_buf[offset_src_buf+inner_loop_var];
		}
		if((strstr(single_msg,(const char*)"ack")==NULL) && strlen(single_msg)>30){
			offset_dst_buf=convert_single_tbr_msg_into_uint(single_msg,dst_buf,offset_dst_buf,&first_timestamp);
		}
	}

	return offset_dst_buf;
}
	/*
	 * public functions
	 */
void tbr_init(void){
	//GPIO_PinModeSet(PWR_EN_PORT, RS485_12V_PWR_EN, gpioModePushPull, 0);
	rs485_init();
	rs485_enable();
	clear_buffer(array_msg,ARRAY_MESSAGE_SIZE);
	array_size=0;
	array_front=0;
	array_rear=0;
	return;
}

void tbr_shutdown(void){
	//GPIO_PinOutClear(PWR_EN_PORT, RS485_12V_PWR_EN);
	return;
}

bool tbr_send_cmd(tbr_cmd_t tbr_cmd,time_t timestamp){
	char 			cmd_tx_buf[CMD_TX_BUF_SIZE];
	bool			ret_flag=false;
	time_t			my_timestamp;
	uint8_t			luhn;
	int				temp_var=0;

	if( tbr_cmd==cmd_sn_req){
		sprintf((char *)cmd_tx_buf,"?\n");
		rs485_transmit_string_new(cmd_tx_buf,1);
		ret_flag=get_and_compare((char *)"SN=");
	}
	else if(tbr_cmd==cmd_basic_sync){
		//sprintf((char *)cmd_tx_buf,"(+)\n");
		cmd_tx_buf[0]='(';
		cmd_tx_buf[1]='+';
		cmd_tx_buf[2]=')';
		cmd_tx_buf[3]='\r';
		rs485_transmit_string_new(cmd_tx_buf,4);
		ret_flag=get_and_compare((char *)"ack01\r");			//changed from 01
		/*if (!ret_flag){
			ret_flag=get_and_compare((char *)"ack01\r");		//changed from 01
		}
		if (!ret_flag){
			ret_flag=get_and_compare((char *)"ack01\r");		//changed from 01
		}*/
		/*if(ret_flag){
			debug_str("\t\tTBR ACK received\n");
		}
		else {
			debug_str("\t\tTBR ACK NOT received\n");
		}*/

	}
	else if(tbr_cmd==cmd_advance_sync){
		my_timestamp=timestamp;
		luhn=CalculateLuhn(&my_timestamp);
		//sprintf((char *)cmd_tx_buf,"(+)%ld\n",my_timestamp);
		cmd_tx_buf[0]='(';											cmd_tx_buf[1]='+'; 											cmd_tx_buf[2]=')';
		cmd_tx_buf[3]=((int)(timestamp/1000000000))%10 +  '0';		cmd_tx_buf[4]=((int)(timestamp/100000000))%10 + '0'; 		cmd_tx_buf[5]=((int)(timestamp/10000000))%10 + '0';
		cmd_tx_buf[6]=((int)(timestamp/1000000))%10 + '0';			cmd_tx_buf[7]=((int)(timestamp/100000))%10 + '0'; 			cmd_tx_buf[8]=((int)(timestamp/10000))%10 + '0';
		cmd_tx_buf[9]=((int)(timestamp/1000))%10 + '0';				cmd_tx_buf[10]=((int)(timestamp/100))%10 + '0'; 			cmd_tx_buf[11]=((int)(timestamp/10))%10 + '0';
		cmd_tx_buf[12]=luhn;										cmd_tx_buf[13]='\r';

		//temp_var=strlen((const char *)cmd_tx_buf);
		//cmd_tx_buf[temp_var-2]=luhn;						//change last digit of TimeStamp
		rs485_transmit_string_new(cmd_tx_buf,14);
		ret_flag=get_and_compare((char *)"ack01\rack02\r");		//changed from 02
		/*if (!ret_flag){
			ret_flag=get_and_compare((char *)"ack01\rack02\r");		//changed from 02
		}
		if (!ret_flag){
			ret_flag=get_and_compare((char *)"ack01\rack02\r");		//changed from 02
		}*/
		/*if(ret_flag){
			debug_str("\t\tTBR ACK received\n");
		}
		else {
			debug_str("\t\tTBR ACK NOT received\n");
		}*/

	}
	else{
		ret_flag=false;
	}
	return ret_flag;
}
uint8_t tbr_recv_msg(char *msg_buf, int *msg_length){
	int 			msg_count=0;
	int 			loop_var=0;
	char			temp_char='0';

	clear_buffer(msg_buf, ARRAY_MESSAGE_SIZE);
	while(!array_is_empty()){
		temp_char=array_remove();
		if(temp_char=='$'){
			msg_count++;
		}
		msg_buf[loop_var]=temp_char;
		loop_var++;
	}
	*msg_length=loop_var;
	return msg_count;
}

uint8_t tbr_recv_msg_uint(uint8_t *lora_msg_buf, int *lora_length, char *msg_buf, int *msg_length)
{
	int 			loop_var=0;
	int 			msg_count=0;
	uint8_t			lora_buf_length=0;
	char			temp_char='0';
	char			temp_char_last='0';
	bool			first_dollar_found=false;

		//SD card msg_buffer
	clear_buffer(msg_buf, ARRAY_MESSAGE_SIZE);

	loop_var=0;
	temp_char_last=0;
	first_dollar_found=false;
	while(!array_is_empty()){
		temp_char=array_remove();
		if(temp_char=='$'){
			msg_count++;
			if(first_dollar_found==true){
				if(temp_char_last!='\n'){
					msg_buf[loop_var]='\n';
					loop_var++;
				}
			}
			first_dollar_found=true;
		}
		msg_buf[loop_var]=temp_char;
		loop_var++;
		temp_char_last=temp_char;
		//debug_char(temp_char);
	}
	//debug_char('\n');
	msg_buf[loop_var]='\n';		//safety of convert_tbr_msgs_to_uint function
	loop_var++;
	*msg_length=loop_var;
		//LoRa buffer
	lora_buf_length=convert_tbr_msgs_to_uint(msg_buf,lora_msg_buf,(uint8_t)msg_count);
	if(lora_buf_length<=2){lora_buf_length=0;}	//exclude Serial Number byte....
	*lora_length=lora_buf_length;
	return msg_count;
}
