/**
 * Example code for using a microchip mrf24j40 module to receive only
 * packets using plain 802.15.4
 * Requirements: 3 pins for spi, 3 pins for reset, chip select and interrupt
 * notifications
 * This example file is considered to be in the public domain
 * Originally written by Karl Palsson, karlp@tweak.net.au, March 2011
 */
#include <SPI.h>
#include "mrf24j.h"
#include <Wire.h>

int slaveAddress = 10;

const int pin_reset = 6;
const int pin_cs = 9; // default CS pin on ATmega8/168/328
const int pin_interrupt = 2; // default interrupt pin on ATmega8/168/328

Mrf24j mrf(pin_reset, pin_cs, pin_interrupt);
int rssi;
int lqi;
int paquet;

void requestEvent () {
  Wire.write (rssi);
  Wire.write( lqi);
  Wire.write(paquet);
} // end of receiveEvent


void setup() {
  Serial.begin(9600);
  Wire.begin(slaveAddress); 
  Wire.onRequest (requestEvent);
  
  mrf.reset();
  mrf.init();
  Serial.println("Init");
  
  mrf.set_pan(0x1234);
  // This is _our_ address
  mrf.address16_write(0x6003); 
  // uncomment if you want to receive any packet on this channel
  mrf.set_promiscuous(true);
  // uncomment if you want to enable PA/LNA external control
  //mrf.set_palna(true);
  
  // uncomment if you want to buffer all PHY Payload
  //mrf.set_bufferPHY(true);
  attachInterrupt(digitalPinToInterrupt(2), interrupt_routine, CHANGE); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
  interrupts();
}

void interrupt_routine() {
  mrf.interrupt_handler(); // mrf24 object interrupt routine
}

volatile bool received = false;

void loop() {
    mrf.check_flags(&handle_rx, &handle_tx);
    if (received) {
      received = false;
    }
    delay(100);
}

void handle_rx() {
  received = true;
  String mensaje = "";
  for (int i = 0; i < mrf.rx_datalength(); i++) {
    paquet = mrf.get_rxinfo()->rx_data[i];
    mensaje += (char)paquet;
  }
  if (mensaje.indexOf("AD") >= 0) {
    Serial.println(mensaje); // filtrar datos válidos
  }
  lqi = mrf.get_rxinfo()->lqi;
  rssi = mrf.get_rxinfo()->rssi;
}

void handle_tx() {
    // code to transmit, nothing to do
}


