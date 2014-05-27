/*
Traffic Play version 0.1
this is now just getting speed from one radar

next set up timer interupt to drive traffic lights

*/

const int PinEN = 9;
const int PinOUT = 7;
int enabled = 0;  //sensor detection flag
int current_state = 0;
int state = 0;    //momentary state value
int count = 0;    //state change count
long currentMilli = 0;


void setup()
{
  Serial.begin(9600);
  enable();
  currentMilli = millis();
}

void loop()
{
  
  if(digitalRead(PinOUT) != current_state)
  {
    count++;
    delay(1);
    current_state = -(current_state - 1);  //changes current_state from 0 to 1, and vice-versa
  }

  if(millis()-currentMilli > 500)          //prints the "speed" every half a second
  {
    print_speed();
  }

}

void enable()
{
  
  //To start the sensor reading, enable pin EN (pin 9) with HIGH signal
  //To make sure it's a clean HIGH signal, give small LOW signal before
  
  pinMode(PinOUT, INPUT);
  pinMode(PinEN, OUTPUT);
  digitalWrite(PinEN,LOW);
  delayMicroseconds(5);
  digitalWrite(PinEN, HIGH);
  wait();
}

void disable()                        //function not used in this program
{
  pinMode(PinEN, OUTPUT);
  digitalWrite(PinEN, LOW);
}

void wait()                            //waits for the sensor to return a state = 1
{
  while(digitalRead(PinOUT) != 1)
  {
    digitalRead(PinOUT);
  }
  current_state = 1;
  Serial.println("Sensor enabled!");
}

void print_speed()
{
  Serial.print("Speed: ");
  Serial.print(count*2);
  Serial.println(" Changes/s");
  currentMilli = millis();
  count = 0;
}
