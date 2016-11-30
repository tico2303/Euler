


///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////

//#include <Arduino_FreeRTOS.h>
#include <I2Cdev.h>
#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro
#include "MPU_Final.h"
#include <Servo.h>
#include <SoftwareSerial.h>
//#include "pidTest.h"
#define YPR_DEBUG 0
#define MOTOR_DEBUG 0
#define PID_DEBUG 0
#define JOYSTICK_DEBUG 0



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 0.4;               //Gain setting for the roll P-controller (1.3)   //.4 So far robert likes .2 and .4 (found minor osillations at .8)
float pid_i_gain_roll = 0;              //Gain setting for the roll I-controller (0.3)    //0.01 recorded
float pid_d_gain_roll = 18.75;                //Gain setting for the roll D-controller (15)  //18.75
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 0;                 //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0;                 //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0;                 //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                    //Maximum output of the PID-controller (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int cal_int, start;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte highByte, lowByte;
int tempint;
int max_motor_val = 180;

//////////////////////////////////////////
//      get JoyStickValues Variables 
/////////////////////////////////////////
int serialCount;
int firstContact;
int valid;
int JoyStick_Complete;
int contact_count = 0;


float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

// offset are calculated by taking the joystick value range and subtracting half to finding center
//         ex: roll_setpoint range(0 - 60) so offset = 30
float roll_offset = 0;
float pitch_offset = 0;
float yaw_offset =0;
float throttle_offset=0;

Servo m1;
Servo m2;
Servo m3;
Servo m4;

int m1Pin=6;//front right 
int m2Pin=10;//back right
int m3Pin=5;//back left
int m4Pin=9;//front left 

char r;
int parse_count=0;
int ledPin=8;

SoftwareSerial ser(12,11);// bluetooth rx, tx

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                 Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{

  ser.begin(9600);// blueTooth
  pinMode(0, INPUT); // battery tester
  pinMode(ledPin, OUTPUT); // Low Battery Warning Light
  
  Serial.begin(115200);// debugging
  Wire.begin();//Start the I2C as master.
  

   /////////////////////////////////////////////////////////////////////
  //                 Wait for input from master
  //////////////////////////////////////////////////////////////////////
  
  Serial.println("*************** Waiting For HandShake With Controller ********************");
  while (ser.available() && ser.read()); // empty buffer
  while (!ser.available());                 // wait for data
  while (ser.available() && ser.read()); // empty buffer again
  

  //reflects min and max pulse width of the esc
  m1.attach(m1Pin);
  m2.attach(m2Pin);
  m3.attach(m3Pin);
  m4.attach(m4Pin);

  
  m1.write(0);
  m2.write(0);
  m3.write(0);
  m4.write(0);
  
  MPU_Init();
  calibrateJoyStick();

  throttle = 0;
  
  Serial.println("****** Initialization Complete. *******");
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  //Let's get the current gyro data.
   MPU_read();
   
   gyro_roll_input = roll;
   gyro_pitch_input = pitch;
   gyro_yaw_input = yaw;
   
  if(YPR_DEBUG)
  {
  Serial.print("Roll: " );
  Serial.print("  ");
  Serial.print(gyro_roll_input);
  Serial.print("  ");
  
  Serial.print("Pitch: " );
  Serial.print("  ");
  Serial.println(gyro_pitch_input);
  Serial.print("  ");

  
  Serial.print("Yaw: " );
  Serial.print("  ");
  Serial.println(gyro_yaw_input);
  Serial.println(" ");
  }
  
 

  //*******************************************************************
  //    PID inputs are known. So we can calculate the pid output.
  //*******************************************************************
  
  calculate_pid();

  
  //****************** 
  // Battery Voltage
  // 12.6V battery
  // 1023 highest analog reading
  // 1260/1023 = 1.2317
  // diode Compensation = 65
  //******************  
                              
   battery_voltage = (analogRead(0) + 65) * 1.2317;

  // The battery voltage is needed for compensation.
  // A complementary filter is used to reduce noise.
  // 0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  if(battery_voltage <1050)
  {
    digitalWrite(ledPin,HIGH);
  }

  ///////////// Get Joystick Values/////////////
  // writes joystick values to:
  //                      throttle, yaw, pitch, roll
  ///////////////////////////////////////////////
  getJoyStickValues();


    /////////////////////////////////////////////////////////////////
    //                  Motor ESC Values
    /////////////////////////////////////////////////////////////////
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

 /////////////////////////////////////////////////
 //          MOTOR LOWER BOUND LIMIT            //
 /////////////////////////////////////////////////
/*
    if(esc_1 <=0)
    {
      esc_1 = 0;
    }
    if(esc_2 <=0)
    {
      esc_2 = 0;
    }
    if(esc_3 <=0)
    {
      esc_3 = 0;
    }
    if(esc_4 <=0)
    {
      esc_4 = 0;
    }
    */

    /////////////////////////////////////////////////
    //          MOTOR UPPER BOUND LIMIT            //
    /////////////////////////////////////////////////
    /*
    if(esc_1 >= max_motor_val)
    {
        esc_1 = 180;
    }
    if(esc_2 >= max_motor_val)
    {
        esc_2 = 180;
    }
    if(esc_3 >= max_motor_val)
    {
        esc_3 = 180;
    }
    if(esc_4 >= max_motor_val)
    {
        esc_4 = 180;
    }
    */
    


    // Fail Safes for debugging
    if(throttle <=0 || throttle == '0')
    {
      esc_1 = 0;
      esc_2 = 0;
      esc_3 = 0;
      esc_4 = 0;
    }


    if(battery_voltage <1240 && battery_voltage >800)
    {
      esc_1 += esc_1 *((1240-battery_voltage)/(float)3500);
      esc_2 += esc_2 *((1240-battery_voltage)/(float)3500);
      esc_3 += esc_3 *((1240-battery_voltage)/(float)3500);
      esc_4 += esc_4 *((1240-battery_voltage)/(float)3500);
    }

    
      /////////////////////////////////////////////////////////////////
      //                    Setting motor Rates 
      /////////////////////////////////////////////////////////////////
      
     /*
    esc_1 = map(esc_1, 0, 180, 1000, 2000);
    esc_2 = map(esc_2, 0, 180, 1000, 2000);
    esc_3 = map(esc_3, 0, 180, 1000, 2000);
    esc_4 = map(esc_4, 0, 180, 1000, 2000);
    */
    
    Serial.print("Valid: ");
    Serial.println(valid);
    if(valid)
    {
      m1.write(esc_1);
      m2.write(esc_2);
      m3.write(esc_3);
      m4.write(esc_4);
    }
    delay(2);
    

    if(JOYSTICK_DEBUG)
    {
        Serial.println(" ");
        Serial.print("Throttle: ");
        Serial.print("  ");
        Serial.print(throttle);
        Serial.println("  ");
    
        Serial.println(" ");
        Serial.print("roll setpoint: ");
        Serial.print("  ");
        Serial.print(pid_roll_setpoint);
        Serial.println("  ");
    
        Serial.println(" ");
        Serial.print("pitch setpoint: ");
        Serial.print("  ");
        Serial.print(pid_pitch_setpoint);
        Serial.println("  ");
    
        Serial.println(" ");
        Serial.print("yaw setpoint: ");
        Serial.print("  ");
        Serial.print(pid_yaw_setpoint);
        Serial.println("  ");

        //joystick calibration offsets
        Serial.println(" ");
        Serial.print("yaw offset: ");
        Serial.print("  ");
        Serial.print(yaw_offset);
        Serial.println("  ");

        Serial.println(" ");
        Serial.print("pitch offset: ");
        Serial.print("  ");
        Serial.print(pitch_offset);
        Serial.println("  ");

        Serial.println(" ");
        Serial.print("roll offset: ");
        Serial.print("  ");
        Serial.print(roll_offset);
        Serial.println("  ");

         Serial.println(" ");
        Serial.print("throttle offset: ");
        Serial.print("  ");
        Serial.print(throttle_offset);
        Serial.println("  ");

        
    }

    if(MOTOR_DEBUG)
    {
        Serial.println(" ");
        Serial.print("Battery: ");
        Serial.print("  ");
        Serial.print(battery_voltage);
        Serial.println("  ");
    
        Serial.print("Motor1: ");
        Serial.print("  ");
        Serial.print(esc_1);
        Serial.print("  ");
        
        Serial.print("Motor2: ");
        Serial.print("  ");
        Serial.print(esc_2);
        Serial.print("\t");
        
        Serial.print("Motor3: ");
        Serial.print("  ");
        Serial.print(esc_3);
        Serial.print("  ");
        
        Serial.print("Motor4: ");
        Serial.print("  ");
        Serial.println(esc_4);
    }

    if(PID_DEBUG)
    {
        Serial.print("pid_output_pitch: ");
        Serial.print("  ");
        Serial.print(pid_output_pitch);
        Serial.print("  ");
        
        Serial.print("pid_output_roll: ");
        Serial.print("  ");
        Serial.print(pid_output_roll);
        Serial.print("\t");
        
        Serial.print("pid_output_yaw: ");
        Serial.print("  ");
        Serial.print(pid_output_yaw);
        Serial.print("  ");
    }
   
}


void calculate_pid()
{
  //////////////////////////////////////////////////////////
  //     ***********Roll calculations****************
  //////////////////////////////////////////////////////////
  
  pid_error_temp = gyro_roll_input - (pid_roll_setpoint-roll_offset);
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  
  if(pid_i_mem_roll > pid_max_roll)
    pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)
    pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  
  if(pid_output_roll > pid_max_roll)
    pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)
    pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;

  //////////////////////////////////////////////////////////
  //************Pitch calculations*******************
  //////////////////////////////////////////////////////////
  
  pid_error_temp = gyro_pitch_input - (pid_pitch_setpoint-pitch_offset);
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;

 
   //////////////////////////////////////////////////////////
  //                  Yaw calculations 
  //////////////////////////////////////////////////////////
  
  pid_error_temp = gyro_yaw_input - (pid_yaw_setpoint-yaw_offset);
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}

void getJoyStickValues()
{
  //Serial.print(".");
  int inByte;
  if(ser.available())
  {
      if(firstContact == false)
      {
           inByte = ser.read();
           Serial.println(inByte);
                
           if(inByte == 'A')
           {
               while(ser.available()&& ser.read());//clear buffer
               
               firstContact = true;
               ser.write('A');
           }
       }
       else
      {
        valid = true;
        contact_count = 0;
        //delay(200);
        serialCount++;
        if(serialCount == 1)
        {
          throttle = ser.read();
          throttle = map(throttle, 0, 250, 0,180);
        }
        else if(serialCount == 2)
        {
          pid_yaw_setpoint = ser.read();
          pid_yaw_setpoint = map(pid_yaw_setpoint, 0, 250, -180, 180);
        }
        else if(serialCount == 3)
        {
          pid_pitch_setpoint = ser.read();
          pid_pitch_setpoint = map(pid_pitch_setpoint, 0,250, -60, 60);
        }
        else if(serialCount == 4)
        {
          
          pid_roll_setpoint = ser.read();
          pid_roll_setpoint = map(pid_roll_setpoint, 0, 250, -60, 60);
          ser.write('A');
          JoyStick_Complete = 1;
          serialCount = 0;     
        }
        else if(serialCount >=5)
        {
          serialCount = 0;
        }
    
      }

  }//end if(available())
  else
  {
    contact_count++;
    Serial.print(" Contact Count: ");
    Serial.println(contact_count);
    if(contact_count > 50)
    {
    throttle = 0;
    valid = false;
    }
    
  }
 
  
}



void calibrateJoyStick()
{
  for(int i=0; i<100; i++)
  {
    getJoyStickValues();
    roll_offset +=pid_roll_setpoint;
    yaw_offset +=pid_yaw_setpoint;
    pitch_offset +=pid_pitch_setpoint;
    throttle_offset +=throttle;
  }
  roll_offset = roll_offset/100;
  yaw_offset = yaw_offset/100;
  pitch_offset = pitch_offset/100;
  throttle_offset = throttle_offset/100;

}





