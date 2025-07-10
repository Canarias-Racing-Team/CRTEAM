#include <SPI.h>
#include <mrf24j.h>
#include <ArduinoJson.h>

Mrf24j mrf(6, 9, 2);  // reset, CS, interrupt

void setup() {
  Serial.begin(9600);
  mrf.reset();
  mrf.init();
  //mrf.set_channel(12);
  mrf.set_pan(0x1234);        // PAN ID compartido
  mrf.address16_write(0x6002); // dirección propia (AD1 o AD2)
  mrf.set_promiscuous(true);
  //mrf.set_bufferPHY(false);
  Serial.println("Emisor AD1 listo");
}


void loop() {
  // Simulación de valores de acelerómetro
  int x = analogRead(A0);
  int y = analogRead(A1);
  int z = analogRead(A2);
  // Armar mensaje tipo: AD1,x:123,y:456,z:789
  String mensaje = "AD1,"; // marcamos para hacer el json del receptor easy
  mensaje += "x:" + String(x) + ",";
  mensaje += "y:" + String(y) + ",";
  mensaje += "z:" + String(z);
  
  //String mensaje = "hola mundo";
  // Enviar al receptor 
  mrf.send16(0x6003, (char*)mensaje.c_str());
  Serial.println(mensaje);
  delay(950); 
}

