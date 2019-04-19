#include "websrv.h"
#include "network.h"
#include <ESPUI.h>

AsyncWebServer server(80);

void switchTransparent(Control sender, int value) {
  switch (value) {
    case S_ACTIVE:
      bootloader_mode = true;
      break;
    case S_INACTIVE:
      bootloader_mode = false;
      break;
  }
}

void callBaudrate(Control sender, int type) { 
  Serial.print("New baudrate: ");
  Serial.println(sender.value); 
  Serial1.updateBaudRate(sender.value.toInt());
}

void webserver_init(void){
  ESPUI.label("ETH state:", COLOR_WETASPHALT, "disconnected");
  ESPUI.label("ETH IP:", COLOR_WETASPHALT, "disconnected");
  ESPUI.switcher("UART Transparent", false, &switchTransparent, COLOR_EMERALD);
  ESPUI.text("Baudrate:", &callBaudrate, COLOR_EMERALD, "2000000");
  ESPUI.begin("UD3 Status");
}

void webserver_update(void){
  static uint32_t last_time;
  if((millis()-last_time) > 1000){
    //ESPUI.print("Dropped frames :", String(min_ctx.transport_fifo.dropped_frames));
    //ESPUI.print("Millis:", String(millis()));

    last_time = millis();
  }  
}