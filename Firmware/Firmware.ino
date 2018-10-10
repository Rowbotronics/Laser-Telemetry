#include <OneWire.h>

OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)

volatile byte pulse_count;
unsigned int rpm;
unsigned long last_time;

byte laser_pin = 6;
byte door_pin = 7;
byte temperature_pin = 10;
byte fan1_pin = 2;

byte test_sensor[8] = {0x28, 0x85, 0xA3, 0x45, 0x92, 0x3, 0x2, 0x8D};
byte mirror1_temp[8] = {0x28, 0x2E, 0xC0, 0x45, 0x92, 0x3, 0x2, 0x76};
byte mirror2_temp[8] = {0x28, 0x2E, 0xC0, 0x45, 0x92, 0x3, 0x2, 0x76};
byte mirror3_temp[8] = {0x28, 0x2E, 0xC0, 0x45, 0x92, 0x3, 0x2, 0x76};
byte temp_sensors[8][3] = {mirror1_temp, mirror2_temp, mirror3_temp};


void rpm_int()
 {
      pulse_count++;
 }


void SearchTempSensors() {
  byte addr[8];
  if ( !ds.search(addr)) {
  //    Serial.println("No more addresses.");
  //    Serial.println();
  ds.reset_search();
  delay(250);
  return;
  }
  
  Serial.print("New Temp Sensor: ");
  for( byte i = 0; i < 8; i++) {
    Serial.write(" 0x");
    Serial.print(addr[i], HEX);
    Serial.print(",");
  }
  Serial.println();
}

float ReadTemperature(byte* addr) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  float celsius, fahrenheit;
  
  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  //  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  byte cfg = (data[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  //// default is 12 bit resolution, 750 ms conversion time
  celsius = (float)raw / 16.0;
  return celsius;
}

unsigned int ReadRPM() {
//    delay(1000); // wait for data
   detachInterrupt(0); // disable interrupts
   rpm = 30*1000/(millis() - last_time)*pulse_count; // with 2 pulses per revolution
   last_time = millis();
   pulse_count = 0;

   //Restart the interrupt processing
   attachInterrupt(0, rpm_int, FALLING);
   return rpm;
}

void setup()
 {
   Serial.begin(9600);
   
   pinMode(fan1_pin, INPUT_PULLUP); // set pullup resistor on pin 2
   pinMode(temperature_pin, INPUT_PULLUP);
   pinMode(laser_pin, INPUT_PULLUP);
   pinMode(door_pin, INPUT_PULLUP);
   attachInterrupt(0, rpm_int, FALLING); // interrupt 0 on pin 2

   pulse_count = 0;
   rpm = 0;
   last_time = 0;

   SearchTempSensors();
 }

 void loop() {
   //Print out result to serial
   Serial.println("{");
   
   Serial.print("\"mirror1\": ");
//   Serial.println(ReadTemperature(test_sensor));
   Serial.print(ReadTemperature(mirror1_temp));
   Serial.println(", ");
   Serial.print("\"mirror2\": ");
   Serial.print(ReadTemperature(mirror2_temp));
   Serial.println(", ");
   Serial.print("\"mirror3\": ");
   Serial.print(ReadTemperature(mirror3_temp));
   Serial.println(", ");
   Serial.print("\"fan1\": ");
   Serial.print(ReadRPM());
   Serial.println(", ");
   Serial.print("\"door\": ");
   Serial.print(digitalRead(door_pin));
   Serial.println(", ");
   Serial.print("\"laser\": ");
   Serial.print(digitalRead(laser_pin));
   Serial.println(", ");
   Serial.println("}");
   Serial.println();
   delay(500);
 
}
