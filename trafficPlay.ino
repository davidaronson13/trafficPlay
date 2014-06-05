/*
Traffic Play version 0.1
this is now just getting speed from one radar



*/
//set a variable to tell if we are awake
bool awake = false;

//first thing is to get the solar panel voltage
int batMonPin = A0;    // input pin for the voltage divider
int batVal = 0;       // variable for the A/D value
float pinBatVoltage = 0; // variable to hold the calculated voltage
float batteryVoltage = 12;
const float ratio = 2.65;
const int batHighThreshold = 11.7;
const int batLowThreshold = 11.5;


const int Radar_A_PinEN = 2;
const int Radar_A_PinOUT = A1;
const int Radar_B_PinEN = 3;
const int Radar_B_PinOUT = A2;
const int Radar_C_PinEN = 4;
const int Radar_C_PinOUT = A3;
const int Radar_D_PinEN = 5;
const int Radar_D_PinOUT = A4;

const int radarReadDuration = 1000;

int enabled = 0;  //sensor detection flag
int b_enabled = 0; 
int c_enabled = 0; 
int d_enabled = 0; 

int current_state = 0;
int b_current_state = 0;
int c_current_state = 0;
int d_current_state = 0;

int state = 0;    //momentary state value
int b_state = 0; 
int c_state = 0; 
int d_state = 0; 

int a_count = 0;    //state change count
int b_count = 0; 
int c_count = 0; 
int d_count = 0; 

long currentMilli = 0;
long b_currentMilli = 0;
long c_currentMilli = 0;
long d_currentMilli = 0;

int a_speed=0;
int b_speed=0;
int c_speed=0;
int d_speed=0;


const int greenPin = 6;
const int yellowPin = 7;
const int redPin = 8;

const int hornA = 9;
bool moveA = false;

bool playGame = false;

//#define INVERTERSWITCH 31  

// Define the number of samples to keep track of.  The higher the number,
// the more the readings will be smoothed, but the slower the output will
// respond to the input.  Using a constant rather than a normal variable lets
// use this value to determine the size of the readings array.
const int numReadings = 25;

int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

void setup()
{
  Serial.begin(9600);
  pinMode(batMonPin, INPUT);
  
  pinMode(greenPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(hornA, OUTPUT);
  digitalWrite(greenPin, HIGH);
   digitalWrite(yellowPin, HIGH);
    digitalWrite(redPin, HIGH);

 // enable();
  currentMilli = millis();
  
  // initialize all the readings to 0: 
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0; 

}

void loop()
{
  //let's get the  battery voltage to see if we should sleep or wake
  readVoltages();
  if (!awake){
    delay(1000 );
    return;
  }
    
  //now we wait to turn on the green light until something moves
  
  getMotion();
  
  if(playGame){
    //green
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    delay(10000);
    
    //yellow
     digitalWrite(greenPin, HIGH);
    digitalWrite(yellowPin, LOW);
    delay(10000);
    
    //red
    digitalWrite(redPin, LOW);
    digitalWrite(yellowPin, HIGH);
     getMotion();
     if (moveA){
       digitalWrite(hornA, HIGH);
       delay(250);
       digitalWrite(hornA, LOW);
       moveA = false;
     }
    delay(10000);
  }
  
}
void readVoltages()
{
   batVal = analogRead(batMonPin);    // read the voltage on the divider 
     //average the amp sensor
  // subtract the last reading:
  total= total - readings[index];         
  // read from the sensor:  
  readings[index] = batVal; 
  // add the reading to the total:
  total= total + readings[index];       
  // advance to the next position in the array:  
  index = index + 1;                    

  // if we're at the end of the array...
  if (index >= numReadings)              
    // ...wrap around to the beginning: 
    index = 0;                           

  // calculate the average:
  average = total / numReadings;         
  
 
  
   Serial.print("bat sensor read: ");
  Serial.println(batVal);  
    // read the analog in value:
//assign sensort value to average, then calc stuff
  batVal = average;
  
   Serial.print("bat sensor average: ");
  Serial.println(batVal);  
  
  pinBatVoltage = batVal * 0.00635;       //  Calculate the voltage on the A/D pin
                                    //  A reading of 1 for the A/D = 0.0048mV
                                    //  if we multiply the A/D reading by 0.00488 then 
                                    //  we get the voltage on the pin.  

  batteryVoltage =  pinBatVoltage * ratio;    //  Use the ratio calculated for the voltage divider// readVcc();// 
                                          //  to calculate the battery voltage
  Serial.print("avg battery voltage: ");
  Serial.println(batteryVoltage); 

 
 
 if(awake){
    if( batteryVoltage < batLowThreshold && awake) {
    
    goSleep();
    }
 }else{
  if( batteryVoltage > batHighThreshold && !awake){
    wakeUp();
  }
 } 
}

void wakeUp()
{
 
 //digitalWrite(INVERTERSWITCH, HIGH); 
 awake = true;
 Serial.println("waking up"); 
 enable();
}
void goSleep()
{
 
// digitalWrite(INVERTERSWITCH, LOW); 
 awake = false;
 Serial.println("going to sleep"); 
 disable();
}
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
 // result = result * .001; //convert millvolts to volts
  return result; // Vcc in millivolts
}
void enable()
{
  
  //To start the sensor reading, enable pin EN (pin 9) with HIGH signal
  //To make sure it's a clean HIGH signal, give small LOW signal before
  
  pinMode(Radar_A_PinOUT, INPUT);
  pinMode(Radar_A_PinEN, OUTPUT);
  digitalWrite(Radar_A_PinEN,LOW);
  delayMicroseconds(5);
  digitalWrite(Radar_A_PinEN, HIGH);
 
  
   pinMode(Radar_B_PinOUT, INPUT);
  pinMode(Radar_B_PinEN, OUTPUT);
  digitalWrite(Radar_B_PinEN,LOW);
  delayMicroseconds(5);
  digitalWrite(Radar_B_PinEN, HIGH);
  
  
   pinMode(Radar_C_PinOUT, INPUT);
  pinMode(Radar_C_PinEN, OUTPUT);
  digitalWrite(Radar_C_PinEN,LOW);
  delayMicroseconds(5);
  digitalWrite(Radar_C_PinEN, HIGH);
  
  
   pinMode(Radar_D_PinOUT, INPUT);
  pinMode(Radar_D_PinEN, OUTPUT);
  digitalWrite(Radar_D_PinEN,LOW);
  delayMicroseconds(5);
  digitalWrite(Radar_D_PinEN, HIGH);
  wait();
}

void disable()                        //function not used in this program
{
  pinMode(Radar_A_PinEN, OUTPUT);
  digitalWrite(Radar_A_PinEN, LOW);
  
    pinMode(Radar_B_PinEN, OUTPUT);
  digitalWrite(Radar_B_PinEN, LOW);
  
    pinMode(Radar_C_PinEN, OUTPUT);
  digitalWrite(Radar_C_PinEN, LOW);
  
    pinMode(Radar_D_PinEN, OUTPUT);
  digitalWrite(Radar_D_PinEN, LOW);
}


void wait()                            //waits for the sensor to return a state = 1
{
  while(digitalRead(Radar_A_PinEN) != 1)
  {
    digitalRead(Radar_A_PinEN);
   
  }
  current_state = 1;
  Serial.println(" Sensor A enabled!");
  
   while(digitalRead(Radar_B_PinEN) != 1)
  {
    digitalRead(Radar_A_PinEN);
   
  }
  b_current_state = 1;
  Serial.println(" Sensor B enabled!");
  
   while(digitalRead(Radar_C_PinEN) != 1)
  {
    digitalRead(Radar_C_PinEN);
   
  }
  c_current_state = 1;
  Serial.println(" Sensor C enabled!");
  
   while(digitalRead(Radar_D_PinEN) != 1)
  {
    digitalRead(Radar_D_PinEN);
   
  }
  d_current_state = 1;
  Serial.println(" Sensor D enabled!");
}
void getMotion(){
  
    if(digitalRead(Radar_A_PinOUT) != current_state)
    {
      a_count++;
      delay(1);
      current_state = -(current_state - 1);  //changes current_state from 0 to 1, and vice-versa
    }
    if(millis()-currentMilli > radarReadDuration)          //prints the "speed" every half a second
    {
      //Serial.print("A: ");
      //print_speed(a_speed, a_count);
      Serial.print("A Speed: ");
      Serial.print(a_count*2);
      Serial.println(" Changes/s");
      if(a_count > 0){
        playGame = true;
        moveA = true;
      }
      currentMilli = millis();
      a_count = 0;
    }
     if(digitalRead(Radar_B_PinOUT) != b_current_state)
    {
      b_count++;
      delay(1);
      b_current_state = -(b_current_state - 1);  //changes current_state from 0 to 1, and vice-versa
    }
    if(millis()-b_currentMilli > radarReadDuration)          //prints the "speed" every half a second
    {
      //Serial.print("B: ");
      //print_speed(b_speed, b_count);
       Serial.print("B Speed: ");
      Serial.print(b_count*2);
      Serial.println(" Changes/s");
      if(b_count > 0)playGame = true;
      b_currentMilli = millis();
      b_count = 0;
    }
     if(digitalRead(Radar_C_PinOUT) != c_current_state)
    {
      c_count++;
      delay(1);
      c_current_state = -(c_current_state - 1);  //changes current_state from 0 to 1, and vice-versa
    }
    if(millis()-c_currentMilli > radarReadDuration)          //prints the "speed" every half a second
    {
      //Serial.print("C: ");
      //print_speed(c_speed, c_count);
       Serial.print("C Speed: ");
      Serial.print(c_count*2);
      Serial.println(" Changes/s");
      if(c_count > 0)playGame = true;
      c_currentMilli = millis();
      c_count = 0;
    }
   if(digitalRead(Radar_D_PinOUT) != d_current_state)
    {
      d_count++;
      delay(1);
      d_current_state = -(d_current_state - 1);  //changes current_state from 0 to 1, and vice-versa
    }
    if(millis()-d_currentMilli > radarReadDuration)          //prints the "speed" every half a second
    {
      //Serial.print("D: ");
      //print_speed(d_speed, d_count);
       Serial.print("D Speed: ");
      Serial.print(d_count*2);
      Serial.println(" Changes/s");
      if(d_count > 0)playGame = true;
      d_currentMilli = millis();
      d_count = 0;
    }
  delay(radarReadDuration);
}


void print_speed(int sPeed, int count)
{
  sPeed = count*2;
  Serial.print("Speed: ");
  Serial.print(sPeed*2);
  Serial.println(" Changes/s");
  currentMilli = millis();
   count = 0;
}


