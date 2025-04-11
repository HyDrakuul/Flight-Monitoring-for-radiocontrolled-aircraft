int sensorPin = 4;              // 220 or 1k resistor connected to this pin
long result = 0;
void setup()                    // run once, when the sketch starts
{
   Serial.begin(9600);
   Serial.println("start");      // a personal quirk 
}
void loop()                     // run over and over again
{
  Serial.println(RCtime(sensorPin));
   Serial.println( 1-((((RCtime(sensorPin))*pow(10,-6)/1e6)-163.33e-12)/3.33e-11));
   delay(1000);

}

long RCtime(int sensPin){
   long result = 0;
   pinMode(sensPin, OUTPUT);       // make pin OUTPUT
   digitalWrite(sensPin, HIGH);    // make pin HIGH to discharge capacitor - study the schematic
   delay(1);                       // wait a  ms to make sure cap is discharged
   unsigned long StartTime =micros();
   pinMode(sensPin, INPUT);        // turn pin into an input and time till pin goes low
   digitalWrite(sensPin, LOW);     // turn pullups off - or it won't work
   while(digitalRead(sensPin)){    // wait for pin to go low
      if(micros()-StartTime > 300000) break;
   }

   return micros()-StartTime;                   // report results   
}