/*
Traffic Play version 0.1
this is now just getting speed from one radar



*/
//set a variable to tell if we are awake
bool awake = false;

//first thing is to get the solar panel voltage
int batMonPin = A8;    // input pin for the voltage divider
int batVal = 0;       // variable for the A/D value
float pinBatVoltage = 0; // variable to hold the calculated voltage
float batteryVoltage = 12;

int panelMonPin = A15;    // input pin for the voltage divider
int panelVal = 0;       // variable for the A/D value
float pinPanelVoltage = 0; // variable to hold the calculated voltage
float panelVoltage = 12;
float ratio = 2.6;  // Change this to match the MEASURED ration of the circuit, 12.2k R1 and 4.7k R2


const int Radar_A_PinEN = 9;
const int Radar_A_PinOUT = 13;
int enabled = 0;  //sensor detection flag
int current_state = 0;
int state = 0;    //momentary state value
int a_count = 0;    //state change count
long currentMilli = 0;

int a_speed=0;

#define INVERTERSWITCH 31  

void setup()
{
  Serial.begin(9600);
  pinMode(batMonPin, INPUT);
  pinMode(panelMonPin, INPUT);
  pinMode(INVERTERSWITCH, OUTPUT);  
  
  digitalWrite(INVERTERSWITCH, LOW);
 // enable();
  currentMilli = millis();
  
  // initialize all the readings to 0: 
 /* for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0; */

}

void loop()
{
  //let's get the solar and battery voltage to see if we should sleep or wake
  readVoltages();
  if (!awake){
    delay(1000 );
    return;
  }
    
  //now we wait to turn on the green light until something moves
  
  
  if(digitalRead(Radar_A_PinOUT) != current_state)
  {
    a_count++;
    delay(1);
    current_state = -(current_state - 1);  //changes current_state from 0 to 1, and vice-versa
  }

  if(millis()-currentMilli > 500)          //prints the "speed" every half a second
  {
    print_speed();
  }
delay(500);
}
void readVoltages()
{
 //  batVal = analogRead(batMonPin);    // read the voltage on the divider 
//   Serial.print("bat sensor read: ");
//  Serial.println(batVal);  
    // read the analog in value:

  
//  pinBatVoltage = batVal * 0.00488;       //  Calculate the voltage on the A/D pin
                                    //  A reading of 1 for the A/D = 0.0048mV
                                    //  if we multiply the A/D reading by 0.00488 then 
                                    //  we get the voltage on the pin.  

  batteryVoltage = readVcc();// pinBatVoltage * ratio;    //  Use the ratio calculated for the voltage divider
                                          //  to calculate the battery voltage
  Serial.print("battery voltage: ");
  Serial.println(batteryVoltage); 

  panelVal = analogRead(panelMonPin);    // read the voltage on the divider 
   Serial.print("panel sensor read: ");
  Serial.println(panelVal);  
    // read the analog in value:

  
  pinPanelVoltage = panelVal * 0.00488;       //  Calculate the voltage on the A/D pin
                                    //  A reading of 1 for the A/D = 0.0048mV
                                    //  if we multiply the A/D reading by 0.00488 then 
                                    //  we get the voltage on the pin.  

  panelVoltage = pinPanelVoltage * ratio;    //  Use the ratio calculated for the voltage divider
                                          //  to calculate the battery voltage
  Serial.print("panel voltage: ");
  Serial.println(panelVoltage);      
 
 if(awake){
    if(panelVoltage < 8 || batteryVoltage < 11){
    
    goSleep();
    }
 }else{
  if(panelVoltage > 11 && batteryVoltage > 12){
    wakeUp();
  }
 } 
}

void wakeUp()
{
 
 digitalWrite(INVERTERSWITCH, HIGH); 
 awake = true;
 Serial.println("waking up"); 
 enable();
}
void goSleep()
{
 
 digitalWrite(INVERTERSWITCH, LOW); 
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
  result = result * .001; //convert millvolts to volts
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
  wait();
}

void disable()                        //function not used in this program
{
  pinMode(Radar_A_PinEN, OUTPUT);
  digitalWrite(Radar_A_PinEN, LOW);
}

void wait()                            //waits for the sensor to return a state = 1
{
  while(digitalRead(Radar_A_PinOUT) != 1)
  {
    digitalRead(Radar_A_PinOUT);
  }
  current_state = 1;
  Serial.println("Sensor enabled!");
}

void print_speed()
{
  a_speed = a_count*2;
 /* Serial.print("Speed: ");
  Serial.print(a_speed*2);
  Serial.println(" Changes/s");*/
  currentMilli = millis();
  a_count = 0;
}
