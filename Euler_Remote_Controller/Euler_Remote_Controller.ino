
#include <SoftwareSerial.h>

SoftwareSerial s(5,6); //rx, tx

int Left_JoyStick_Throttle = 0;
int Left_JoyStick_Yaw = 0;
int Right_JoyStick_Pitch = 0;
int Right_JoyStick_Roll = 0;
int Incoming = 0;
int hazardPin = 4;


void setup() 
{
  s.begin(9600);
  Serial.begin(9600);
  
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(hazardPin, INPUT);
  digitalWrite(hazardPin, 1);
  while(!Serial);
  Serial.println("Enter any key to begin sending controller data.");
  while(!Serial.available()); //Wait for Keyboard Input to start.

  while(s.available() && s.read());
  while(Serial.available() && Serial.read());

  Serial.println("Initializing Hankshake with Flight Controller.");
  HandShake();
  Serial.println("Handshake Successful.");
}

void loop() 
{
  int val = digitalRead(hazardPin);
  Serial.println(val);
  if(!val)
    {
      hold();
    }
  if(s.available() > 0)
  {
    
    //Read in the serial acknowledgement signal, clear the serial buffer.
    Incoming = s.read(); 

    //Read in Joystick data from analog inputs.
    Left_JoyStick_Throttle = analogRead(0);
    Left_JoyStick_Yaw = analogRead(1);
    Right_JoyStick_Pitch = analogRead(2);
    Right_JoyStick_Roll = analogRead(3);

    //Map the values to fit in a sigle byte.
    Left_JoyStick_Throttle = map(Left_JoyStick_Throttle, 0, 1024, 0, 250); //Map the input to a value in the motor range.
    Left_JoyStick_Yaw = map(Left_JoyStick_Yaw, 0, 1024, 0, 250); //Map the input to a value in the motor range.
    Right_JoyStick_Pitch = map(Right_JoyStick_Pitch, 0, 1024, 0, 250); //Map the input to a value in the motor range.
    Right_JoyStick_Roll = map(Right_JoyStick_Roll, 0, 1024, 0, 250); //Map the input to a value in the motor range.

    //Prints for serial debugging.
    Serial.print("Throttle: ");
    Serial.println(Left_JoyStick_Throttle);

    Serial.print("Yaw: ");
    Serial.println(Left_JoyStick_Yaw);

    Serial.print("Pitch: ");
    Serial.println(Right_JoyStick_Pitch);

    Serial.print("Roll: ");
    Serial.println(Right_JoyStick_Roll);

    Serial.println(" ");

    //Send over bluetooth to flight controller.
    s.write(Left_JoyStick_Throttle);
    s.write(Left_JoyStick_Yaw);
    s.write(Right_JoyStick_Pitch);
    s.write(Right_JoyStick_Roll);

    delay(750);
  }
}

void HandShake() 
{
  while (s.available() <= 0) 
  {
    s.print('A');
    Serial.println('A');
    delay(300);
  }
}
void hold()
{
  Serial.println("Holding....");
  while(1){;}
}



