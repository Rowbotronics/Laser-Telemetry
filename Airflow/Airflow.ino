volatile byte pulse_count;
unsigned int rpm;
unsigned long last_time;

void rpm_int()
 {
      pulse_count++;
 }

void setup()
 {
   Serial.begin(9600);
   
   pinMode(2, INPUT_PULLUP); // set pullup resistor on pin 2
   attachInterrupt(0, rpm_int, FALLING); // interrupt 0 on pin 2

   pulse_count = 0;
   rpm = 0;
   last_time = 0;
 }

 void loop()
 {
   delay(1000); // wait for data
   detachInterrupt(0); // disable interrupts
   rpm = 30*1000/(millis() - last_time)*pulse_count; // with 2 pulses per revolution
   last_time = millis();
   pulse_count = 0;

   //Print out result to serial
   Serial.print("RPM=");
   Serial.println(rpm);

   //Restart the interrupt processing
   attachInterrupt(0, rpm_int, FALLING);
  }
