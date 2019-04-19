#include <Arduino.h>
#include <string.h>

#include "websrv.h"
#include "network.h"
#include "minhelper.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(512000);
  Serial1.begin(2000000,SERIAL_8N1,36,4);
  Serial1.setRxBufferSize(SERIAL_RX_BUFF);
  
  while(!Serial) {
    ; // Wait for serial port
  }
  Serial.print("\033[2J\033[1;1H");
    
  network_init();
  min_init();
  webserver_init();

}


void loop() {
  if(!bootloader_mode){
    min_run();
    telnet_run();
    midi_run();
    wd_reset();
    
  }else{
    bootloader_run();
  }

  webserver_update();

}


