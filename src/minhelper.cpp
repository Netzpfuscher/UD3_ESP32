/*
 * UD3 ESP32 interface
 *
 * Copyright (c) 2019 Jens Kerrinnes
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include "minhelper.h"

#include "network.h"

struct min_context min_ctx;
uint8_t wd_reset_msg[2] = {0x00,0x00};

void min_init(void){
    min_init_context(&min_ctx, 0);
}

void min_tx_start(uint8_t port)
{
  return;
}

void min_tx_finished(uint8_t port)
{
  return;
}

uint16_t min_tx_space(uint8_t port)
{
  // Ignore 'port' because we have just one context. But in a bigger application
  // with multiple ports we could make an array indexed by port to select the serial
  // port we need to use.
  //uint16_t n = Serial1.availableForWrite();

  return 512;
}
uint32_t min_rx_space(uint8_t port)
{
  // Ignore 'port' because we have just one context. But in a bigger application
  // with multiple ports we could make an array indexed by port to select the serial
  // port we need to use.
  uint32_t n = SERIAL_RX_BUFF - Serial1.available();

  return n;
}

// Send a character on the designated port.
void min_tx_byte(uint8_t port, uint8_t byte)
{
  // Ignore 'port' because we have just one context.
  Serial1.write(&byte, 1U);  
}

// Tell MIN the current time in milliseconds.
uint32_t min_time_ms(void)
{
  return millis();
  //return micros()/3;
}

void min_run(void){

    // put your main code here, to run repeatedly:
    uint32_t serial_bytes_available=0;
    char buf[1024];
    size_t buf_len;

    serial_bytes_available = Serial1.available();
      // Read some bytes from the USB serial port..
    if(serial_bytes_available > 0) {
      buf_len = Serial1.readBytes(buf,serial_bytes_available>sizeof(buf)?sizeof(buf):serial_bytes_available);
    }
    else {
      buf_len = 0;
    }

    min_poll(&min_ctx, (uint8_t *)buf, buf_len);
    
}

void process_command(uint8_t *min_payload, uint8_t len_payload){
  len_payload--;
  switch(*min_payload++){
    case COMMAND_IP:
      Serial.print("Received IP: ");
      Serial.write(min_payload,len_payload);
      Serial.print("\r\n");
    break;
    case COMMAND_GW:
      Serial.print("Received GW: ");
      Serial.write(min_payload,len_payload);
      Serial.print("\r\n");
    break;
    case COMMAND_MAC:
      Serial.print("Received MAC: ");
      Serial.write(min_payload,len_payload);
      Serial.print("\r\n");
    break;
    case COMMAND_SSID:
      Serial.print("Received SSID: ");
      Serial.write(min_payload,len_payload);
      Serial.print("\r\n");
      memcpy(ssid,min_payload,len_payload);
      ssid[len_payload]='\0';
    break;
    case COMMAND_PASSWD:
      Serial.print("Received PASSWD: ");
      Serial.write(min_payload,len_payload);
      Serial.print("\r\n");
      memcpy(password,min_payload,len_payload);
      password[len_payload]='\0';
      if(strcmp(ssid,"NULL")!=0 && strcmp(password,"NULL")!=0){
        connectToWiFi(ssid,password);
      }
    break;

  }

}

uint8_t assemble_if_command(uint8_t cmd, uint8_t interface,char *str, uint8_t *buf){
    uint8_t len=0;
    *buf = cmd;
    buf++;
    *buf = interface;
    buf++;
    len=strlen(str);
    memcpy(buf,str,len);
    return len+2;
}


void min_application_handler(uint8_t min_id, uint8_t *min_payload, uint8_t len_payload, uint8_t port)
{
  if(min_id < 10){
    sendTelnetData(min_payload,len_payload,min_id);
    return;
  }
  else if(min_id >=20 && min_id<30){
    for(uint16_t w=0;w<len_payload;w++){
      if(min_payload[w]=='o') Serial.println("Rec_Start");
    }
    sendMIDIData(min_payload,len_payload,min_id- 20);
    
    return;
  }
  switch(min_id){
    case MIN_ID_RESET:
      
    break;
    case MIN_ID_COMMAND:
      process_command(min_payload,len_payload);
    break;
  }

}

void wd_reset(void){
  static uint32_t last_time;
  if((millis()-last_time) > CONNECTION_HEARTBEAT){
    min_send_frame(&min_ctx,MIN_ID_WD,wd_reset_msg,0);
    last_time = millis();
  }  
}