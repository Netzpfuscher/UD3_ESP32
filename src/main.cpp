#include <Arduino.h>
#include <stdarg.h>
#include <WiFi.h>
#include <string.h>
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
#define ETH_PHY_POWER 12
#include <ETH.h>

#include "min.h"
/*
// Debug printing callback
void min_debug_print(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 256, fmt, args);
    va_end (args);
    Serial.print(buf);
}
*/

#include "mincon.h"

#define SERIAL_RX_BUFF 4096


char ssid[32];
char password[32];



struct min_context min_ctx;

#define MAX_SRV_CLIENTS 3
#define BUFFER_BYTES 128

WiFiServer telnet_server(23);
WiFiServer midi_server(123);
WiFiClient telnet_serverClients[MAX_SRV_CLIENTS];
WiFiClient midi_serverClients[MAX_SRV_CLIENTS];

uint32_t last_sent;

boolean connected = false;


extern xQueueHandle id0Qhandle;



static bool eth_connected = false;

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

void sendTelnetData(uint8_t *ptr, uint32_t len, uint8_t socket){
  if(socket<MAX_SRV_CLIENTS){
    if (telnet_serverClients[socket] && telnet_serverClients[socket].connected()){
        telnet_serverClients[socket].write(ptr,len);

    }
  }

}
bool stop=false;

void sendMIDIData(uint8_t *ptr, uint32_t len, uint8_t socket){

  for(uint8_t i=0;i<MAX_SRV_CLIENTS;i++){
    if(socket<MAX_SRV_CLIENTS){
      if (midi_serverClients[i] && midi_serverClients[i].connected()){
        midi_serverClients[i].write(ptr,len);
      }
    }
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

void WiFiEvent(WiFiEvent_t event){
  if(SYSTEM_EVENT_ETH_GOT_IP || SYSTEM_EVENT_STA_GOT_IP){
    
  }
  uint8_t buf[60];
  uint8_t temp[4];
  uint8_t len;
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      //When connected set 
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());  
      //initializes the UDP state
      //This initializes the transfer buffer
      
      connected = true;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      connected = false;
      break;
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      Serial.print(", GW: ");
      Serial.print(ETH.gatewayIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      len = assemble_if_command(COMMAND_IP,ETH_INFO_ETH,(char*)ETH.localIP().toString().c_str(),buf);
      min_send_frame(&min_ctx,MIN_ID_COMMAND,buf,len);
      len = assemble_if_command(COMMAND_GW,ETH_INFO_ETH,(char*)ETH.gatewayIP().toString().c_str(),buf);
      min_send_frame(&min_ctx,MIN_ID_COMMAND,buf,len);
      if(ETH.linkSpeed()==10){
        len = assemble_if_command(COMMAND_INFO,ETH_INFO_ETH,"10 Mbps",buf);
      }else{
        len = assemble_if_command(COMMAND_INFO,ETH_INFO_ETH,"100 Mbps",buf);
      }
      min_send_frame(&min_ctx,MIN_ID_COMMAND,buf,len);
      temp[0] = 0x02;
      temp[1] = 0x00;
      len = assemble_if_command(COMMAND_ETH_STATE,ETH_INFO_ETH,(char*)temp,buf);
      min_send_frame(&min_ctx,MIN_ID_COMMAND,buf,len);
      eth_connected = true;
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      temp[0] = 0x01;
      temp[1] = 0x00;
      len = assemble_if_command(COMMAND_ETH_STATE,ETH_INFO_ETH,(char*)temp,buf);
      min_send_frame(&min_ctx,MIN_ID_COMMAND,buf,len);
      eth_connected = false;
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    case SYSTEM_EVENT_AP_START:
      len = assemble_if_command(COMMAND_IP,ETH_INFO_WIFI,(char*)WiFi.softAPIP().toString().c_str(),buf);
      min_send_frame(&min_ctx,MIN_ID_COMMAND,buf,len);
      len = assemble_if_command(COMMAND_GW,ETH_INFO_WIFI,(char*)WiFi.gatewayIP().toString().c_str(),buf);
      min_send_frame(&min_ctx,MIN_ID_COMMAND,buf,len);
      break;
    default:
      break;
    }
}

void connectToWiFi(const char * ssid, const char * pwd){
  static bool event_registered=false;
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  //WiFi.disconnect(true);
  WiFi.softAPdisconnect();
  //register event handler
  if(!event_registered){
    WiFi.onEvent(WiFiEvent);
    event_registered=true;
  }
  
  //Initiate connection
  //WiFi.begin(ssid, pwd);
  WiFi.softAP(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

#define CONNECTION_TIMEOUT      150     //ms
#define CONNECTION_HEARTBEAT    25      //ms

uint32_t reset;
void min_wd_reset(void){
  //reset=min_time_ms();
}

void min_wd(void){
    static uint32_t last=0;
    /*
    if((millis()-reset)>CONNECTION_TIMEOUT){
            xSemaphoreTake(min_block,portMAX_DELAY);
            min_transport_reset(&min_ctx,true);
            xSemaphoreGive(min_block); 
        }*/
        
        if((millis()-last) > CONNECTION_HEARTBEAT){
            last = millis();
            uint8_t byte=0;

            min_send_frame(&min_ctx,MIN_ID_RESET,&byte,1);
 
        }
    
}


void telnet_run(){
  uint8_t buf[BUFFER_BYTES];
  uint8_t i;  
  static bool old_state_1[MAX_SRV_CLIENTS];
  static bool old_state_2[MAX_SRV_CLIENTS];
  if (telnet_server.hasClient()){
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      //find free/disconnected spot
      if (!telnet_serverClients[i] || !telnet_serverClients[i].connected()){
        if(telnet_serverClients[i]){
          telnet_serverClients[i].stop();
          } 
        telnet_serverClients[i] = telnet_server.available();
        char temp[30];
        temp[1]=SOCKET_CONNECTED;
        temp[0]=i;
        strcpy(&temp[2],telnet_serverClients[i].remoteIP().toString().c_str());
        min_queue_frame(&min_ctx,MIN_ID_SOCKET,(uint8_t*)temp,telnet_serverClients[i].remoteIP().toString().length()+3);
        if (!telnet_serverClients[i]) Serial.println("available broken");
        Serial.print("New client: ");
        Serial.print(i); Serial.print(' ');
        Serial.println(telnet_serverClients[i].remoteIP());
        break;
      }
    }
    if (i >= MAX_SRV_CLIENTS) {
      //no free/disconnected spot so reject
      telnet_server.available().stop();
    }
  }
    //check clients for data
  for(i = 0; i < MAX_SRV_CLIENTS; i++){
    if((!telnet_serverClients[i]&&old_state_1[i]) || (!telnet_serverClients[i].connected()&&old_state_2[i])){
      uint8_t temp[3];
      temp[2]='\0';
      temp[1]=SOCKET_DISCONNECTED;
      temp[0]=i;
      min_queue_frame(&min_ctx,MIN_ID_SOCKET,temp,sizeof(temp));
      Serial.print("Disconected: ");
      Serial.print(i); Serial.print(' ');
      Serial.println(telnet_serverClients[i].remoteIP());
    }
    old_state_1[i]=telnet_serverClients[i];
    old_state_2[i]=telnet_serverClients[i].connected();

    if (telnet_serverClients[i] && telnet_serverClients[i].connected()){
      int bytes_in_buffer = telnet_serverClients[i].available();
      if(bytes_in_buffer){
        //get data from the telnet client and push it to the UART
        uint8_t len = bytes_in_buffer>sizeof(buf)?sizeof(buf):bytes_in_buffer;
        telnet_serverClients[i].readBytes(buf,len);

        //min_send_frame(&min_ctx, i, buf, len);
        
        
        while(min_queue_frame(&min_ctx, i, buf, len)==false){
          min_run();
          vTaskDelay(1);
        }

      }
    } else {
      if (telnet_serverClients[i]) {
        telnet_serverClients[i].stop();
      }
    }
  }
}



void midi_run(){
  uint8_t i;
  uint8_t buf[BUFFER_BYTES];
  //check if there are any new clients
  if (midi_server.hasClient()){
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      //find free/disconnected spot
      if (!midi_serverClients[i] || !midi_serverClients[i].connected()){
        if(midi_serverClients[i]) midi_serverClients[i].stop();
        midi_serverClients[i] = midi_server.available();
        break;
      }
    }
    if (i >= MAX_SRV_CLIENTS) {
      //no free/disconnected spot so reject
      midi_server.available().stop();
    }
  }

  //check clients for data
  for(i = 0; i < MAX_SRV_CLIENTS; i++){
    if (midi_serverClients[i] && midi_serverClients[i].connected()){
      int bytes_in_buffer = midi_serverClients[i].available();
      if(bytes_in_buffer){
        //get data from the midi client and push it to the UART
        uint8_t len = bytes_in_buffer>sizeof(buf)?sizeof(buf):bytes_in_buffer;
        midi_serverClients[i].readBytes(buf,len);

        //min_send_frame(&min_ctx, MIN_ID_MIDI + i, buf, len);
        
        while(min_queue_frame(&min_ctx, MIN_ID_MIDI + i, buf, len)==false){
          min_run();
          vTaskDelay(1);
        }

      }
    }
    else {
      if (midi_serverClients[i]) {
        midi_serverClients[i].stop();
      }
    }
  }
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



void min_application_handler(uint8_t min_id, uint8_t *min_payload, uint8_t len_payload, uint8_t port)
{
  // In this simple example application we just echo the frame back when we get one, with the MIN ID
  // one more than the incoming frame.
  //
  // We ignore the port because we have one context, but we could use it to index an array of
  // contexts in a bigger application.
  //Serial.write(min_payload,len_payload);
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
      min_wd_reset();
    break;
    case MIN_ID_COMMAND:
      process_command(min_payload,len_payload);
    break;
  }
  // The frame echoed back doesn't go through the transport protocol: it's send back directly
  // as a datagram (and could be lost if there were noise on the serial line).
  //min_send_frame(&min_ctx, min_id, min_payload, len_payload);
}







void setup() {
  // put your setup code here, to run once:
  Serial.begin(512000);
  Serial1.begin(2000000,SERIAL_8N1,36,4);
  Serial1.setRxBufferSize(SERIAL_RX_BUFF);
  strcpy(ssid,"NULL");
  strcpy(password,"NULL");
  while(!Serial) {
    ; // Wait for serial port
  }
  Serial.print("\033[2J\033[1;1H");
  connectToWiFi(ssid, password);
  
  telnet_server.begin();
  telnet_server.setNoDelay(true);
  midi_server.begin();
  midi_server.setNoDelay(true);

  ETH.begin();

  min_init_context(&min_ctx, 0);

}

void loop() {

  min_run();
  telnet_run();
  midi_run();


}


