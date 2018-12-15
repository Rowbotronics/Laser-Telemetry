#include <OneWire.h>

#define DANIEL
//#define LILLY
//#define TEST

OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)

volatile float pulse_count[5] = {0};
long pulse_count_duration[5] = {0};
byte pulse_bank = 0;
//unsigned long last_time;

byte laser_pin = 6;
byte door_pin = 7;
byte temperature_pin = 10;
byte fan1_pin = 2;

//Red Laser (Daniel)
#ifdef DANIEL
byte mirror1_addr[8] = {0x28, 0x96, 0x95, 0x45, 0x92, 0xB, 0x2, 0xE1};
byte mirror2_addr[8] = {0x28, 0x85, 0xA3, 0x45, 0x92, 0x3, 0x2, 0x8D};
byte mirror3_addr[8] = {0x28, 0x1C, 0x89, 0x45, 0x92, 0x3, 0x2, 0xEF};
#endif

//Blue Laser (Lilly)
#ifdef LILLY
byte mirror1_addr[8] = {0x28, 0x6F, 0xA0, 0x45, 0x92, 0x7, 0x2, 0x1E};
byte mirror2_addr[8] = {0x28, 0x26, 0x88, 0x45, 0x92, 0x3, 0x2, 0x0};
byte mirror3_addr[8] = {0x28, 0xCD, 0x68, 0x45, 0x92, 0xB, 0x2, 0x7D};
#endif

//Test setup
#ifdef TEST
byte mirror1_addr[8] = {0x28, 0x79, 0xB1, 0x45, 0x92, 0xC, 0x2, 0x65};
byte mirror2_addr[8] = {0x28, 0xB7, 0x9B, 0x45, 0x92, 0xB, 0x2, 0xC2};
byte mirror3_addr[8] = {0x28, 0x79, 0xB1, 0x45, 0x92, 0xC, 0x2, 0x65};
#endif

byte* temp_sensors[3] = {mirror1_addr, mirror2_addr, mirror3_addr};
float temperatures[3] = {0};

void rpm_int()
 {
      pulse_count[pulse_bank]++;
 }


void SearchTempSensors() {
  while(1) {
    byte addr[8];
    if ( !ds.search(addr)) {
//      Serial.println("No more addresses.");
//      Serial.println();
      ds.reset_search();
      delay(250);
      return;
    }
    
    Serial.print("Temp Sensor: ");
    for( byte i = 0; i < 8; i++) {
      Serial.write(" 0x");
      Serial.print(addr[i], HEX);
      Serial.print(",");
    }
    Serial.println();
  }
}
void UpdateTemperatures() {
  static long read_start = 0;
  static bool waiting = false;
  bool toggle_wait = false;
  bool waited_long_enough = (millis() - read_start) > 1000;
  
  for(byte i=0; i<3; i++)
  { 
    byte present = 0;
    byte type_s;
    byte data[12];
    float celsius, fahrenheit;
    byte* addr = temp_sensors[i];
//    byte* addr = mirror1_addr;
    
//    Serial.print("\naddr:");
//    for(int b=0; b<8; b++) {
//      Serial.write(" 0x");
//      Serial.print(addr[b], HEX);
//      Serial.print(",");
//    }
//    Serial.println();
//    
    
    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
//        return;
    }

    if (!waiting) { 
      ds.reset();
      ds.select(addr);
      ds.write(0x44, 1);        // start conversion, with parasite power on at the end
      read_start = millis();
      toggle_wait = true;
    }

    if (waiting && waited_long_enough) {
      present = ds.reset();
      ds.select(addr);    
      ds.write(0xBE);         // Read Scratchpad
    
      for (byte j = 0; j < 9; j++) {           // we need 9 bytes
        data[j] = ds.read();
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
      temperatures[i] = celsius;
      toggle_wait = true;
    }
  }
  if (toggle_wait) waiting=!waiting;
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

long ReadRPM() {
   static long last_rpm_read = 0;
   static unsigned long rpm = 0;
//   Serial.println("ReadRPM");
   long current_time = millis();
   if (current_time - last_rpm_read > 1000) {
     detachInterrupt(0); // disable interrupts
     long total_pulse_count = pulse_count[0] + pulse_count[1] + pulse_count[2] + pulse_count[3] + pulse_count[4];
     pulse_count_duration[pulse_bank] = current_time - last_rpm_read;
     last_rpm_read = current_time;
     long total_duration = pulse_count_duration[0] + pulse_count_duration[1] + pulse_count_duration[2] + pulse_count_duration[3] + pulse_count_duration[4];   
     rpm = (30000*total_pulse_count)/total_duration; // with 2 pulses per revolution
//     Serial.print("pulse bank: ");
//     Serial.println(pulse_bank);
//
//     for(int bank=0;bank<5;bank++) {
//       Serial.print("bank: ");
//       Serial.println(bank);
//  
//       Serial.print("pulse_count: ");
//       Serial.println(pulse_count[bank]);
//       Serial.print("duration: ");
//       Serial.println(pulse_count_duration[bank]);
//     }
//     Serial.print("total_pulse_count: ");
//     Serial.println(total_pulse_count);
//     Serial.print("total_duration: ");
//     Serial.println(total_duration);
//  
     pulse_bank = ++pulse_bank % 5;

     pulse_count[pulse_bank] = 0;
     pulse_count_duration[pulse_bank] = 0;
  
     //Restart the interrupt processing
     attachInterrupt(0, rpm_int, FALLING);
//     Serial.print("rpm: ");
//     Serial.println(rpm);
   }

   return rpm;
}

void setup()
 {
   Serial.begin(115200);
   
   pinMode(fan1_pin, INPUT_PULLUP); // set pullup resistor on pin 2
   pinMode(temperature_pin, INPUT);
   pinMode(laser_pin, INPUT_PULLUP);
   pinMode(door_pin, INPUT_PULLUP);
   attachInterrupt(0, rpm_int, FALLING); // interrupt 0 on pin 2
   
   SearchTempSensors();
 }

 void loop() {
  static long last_print = 0;
  
  UpdateTemperatures();

  long current_time = millis();
//  ReadRPM();
//  if(false)
  if(current_time - last_print > 100)
  {
     //Print out result to serial 
     Serial.println("{");  
     Serial.print("\"mirror1\": ");
     Serial.print(temperatures[0]);
     Serial.println(", ");
     Serial.print("\"mirror2\": ");
     Serial.print(temperatures[1]);
     Serial.println(", ");
     Serial.print("\"mirror3\": ");
     Serial.print(temperatures[2]);
     Serial.println(", ");
     Serial.print("\"fan1\": ");
     Serial.print(ReadRPM());
     Serial.println(", ");
     Serial.print("\"door\": ");
     Serial.print(!digitalRead(door_pin));
     Serial.println(", ");
     Serial.print("\"laser\": ");
     Serial.print(!digitalRead(laser_pin));
     Serial.println(", ");
     Serial.println("}");
     Serial.println();

     last_print = current_time;
  }
}
