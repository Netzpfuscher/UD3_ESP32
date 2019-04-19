#include "network.h"
#include "min.h"
#include "minhelper.h"
#include "ESPUI.h"

#define MAX_SRV_CLIENTS 3
#define BUFFER_BYTES 128

WiFiServer telnet_server(23);
WiFiServer midi_server(123);
WiFiClient telnet_serverClients[MAX_SRV_CLIENTS];
WiFiClient midi_serverClients[MAX_SRV_CLIENTS];

bool connected = false;
bool eth_connected = false;

extern bool bootloader_mode = false;

char ssid[32];
char password[32];

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
      ESPUI.print("ETH state:", String(ETH.linkSpeed()) + "Mbps");
      ESPUI.print("ETH IP:", ETH.localIP().toString());
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      temp[0] = 0x01;
      temp[1] = 0x00;
      len = assemble_if_command(COMMAND_ETH_STATE,ETH_INFO_ETH,(char*)temp,buf);
      min_send_frame(&min_ctx,MIN_ID_COMMAND,buf,len);
      eth_connected = false;
      ESPUI.print("ETH state:", "disconnected");
      ESPUI.print("ETH IP:", "disconnected");
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

void sendTelnetData(uint8_t *ptr, uint32_t len, uint8_t socket){
  if(socket<MAX_SRV_CLIENTS){
    if (telnet_serverClients[socket] && telnet_serverClients[socket].connected()){
        telnet_serverClients[socket].write(ptr,len);

    }
  }

}

void sendMIDIData(uint8_t *ptr, uint32_t len, uint8_t socket){

  for(uint8_t i=0;i<MAX_SRV_CLIENTS;i++){
    if(socket<MAX_SRV_CLIENTS){
      if (midi_serverClients[i] && midi_serverClients[i].connected()){
        midi_serverClients[i].write(ptr,len);
      }
    }
  }
}

void connectToWiFi(const char * ssid, const char * pwd){
  static bool event_registered=false;
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.softAPdisconnect();
  //register event handler
  if(!event_registered){
    WiFi.onEvent(WiFiEvent);
    event_registered=true;
  }
  
  //Initiate connection
  WiFi.softAP(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

void telnet_run(void){
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



void midi_run(void){
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

void network_init(void){
    strcpy(ssid,"NULL");
    strcpy(password,"NULLNULL");
    connectToWiFi(ssid, password);
    telnet_server.begin();
    telnet_server.setNoDelay(true);
    midi_server.begin();
    midi_server.setNoDelay(true);
    ETH.begin();
}

void bootloader_run(void){
  uint8_t buf[BUFFER_BYTES];
  static uint8_t last_socket=0;
  uint8_t i;

  // put your main code here, to run repeatedly:
  uint32_t serial_bytes_available=0;
  size_t buf_len;



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
        if (!telnet_serverClients[i]) Serial.println("available broken");
        last_socket = i;
        Serial.print("New bootloader client: ");
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
        last_socket = i;
        Serial1.write(buf, len);
      }
    } else {
      if (telnet_serverClients[i]) {
        telnet_serverClients[i].stop();
      }
    }
  }

  serial_bytes_available = Serial1.available();
  // Read some bytes from the USB serial port..
  if(serial_bytes_available > 0) {
    buf_len = Serial1.readBytes(buf,serial_bytes_available>sizeof(buf)?sizeof(buf):serial_bytes_available);
    sendTelnetData(buf,buf_len,last_socket);
  }

}