#include <ECE3.h>

uint16_t sensorValues[8];
void  ChangeBaseSpeed(int initialBaseSpd, int finalBaseSpd);
const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int right_nslp_pin=11;
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

const int LED_RF = 77;
int prevTime = 0;
bool LED_on = false;

int baseSpeed = 40; 

float oldSumSensorValues = 0.0;

int donut = 0;
int finishLine = 0;
int encoder_count = 0;

const int minimum[8] = {620, 689, 643, 643, 597, 597, 643, 643};
const int maximum[8] = {1880, 1811, 1857, 1210, 1255, 1327, 1399, 1857};
const int scale[8] = {-15, -14, -12, -8, 8, 12, 14, 15};



void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);
  
  pinMode(LED_RF, OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  digitalWrite(LED_RF,LOW);
   
  ChangeBaseSpeed(0, baseSpeed);
}

  
void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);
  int newSensorValues[8] = {0};
  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  for (unsigned char i = 0; i < 8; i++)
  {
    newSensorValues[i] = sensorValues[i];
    newSensorValues[i] -= minimum[i];
    newSensorValues[i] = (newSensorValues[i]*1000)/maximum[i];
//    Serial.print(newSensorValues[i]);
//    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
    
  }
  

  float sumSensorValues = (newSensorValues[0]*-15+newSensorValues[1]*-14+newSensorValues[2]*-12+newSensorValues[3]*-8+newSensorValues[4]*8+newSensorValues[5]*12+newSensorValues[6]*14+newSensorValues[7]*15)/8;
  
  
//  Serial.print( sumSensorValues );
//  Serial.println();

  float k_p = 0.01;
  float k_d = 0.005;
  float delta = k_p * sumSensorValues + k_d * (sumSensorValues - oldSumSensorValues);
  
  

  int allBlack = 0;
  for (int i = 0; i < 8; ++i)
  {
    if (newSensorValues[i] > 950) 
    {
      allBlack++;
    }
  }

  if (allBlack > 7)
  { 
    donut++; 
  }
  else
  {
    donut = 0;
  }
  
  if (donut > 1 && finishLine < 1)
  {
  //OLD DONUT CODE
//    digitalWrite(left_dir_pin,HIGH);
//    ChangeBaseSpeed(baseSpeed,80);
//    delay(450);
//    digitalWrite(left_dir_pin,LOW);
//    ChangeBaseSpeed(80,baseSpeed);
//    donut = 0;
//    finishLine++;

  //NEW DONUT CODE

    resetEncoderCount_left();
    digitalWrite(left_dir_pin,HIGH);
    ChangeBaseSpeed(baseSpeed,80);
    encoder_count = getEncoderCount_left();
    while(encoder_count < 320)
    {
      encoder_count = getEncoderCount_left();
    }
    digitalWrite(left_dir_pin,LOW);
    ChangeBaseSpeed(80,baseSpeed);
    donut = 0;
    finishLine++;
     
    

        

  }
  else if (donut > 1 && finishLine > 0)
  {
    delay(300);
    ChangeBaseSpeed(baseSpeed,0);
    exit(0);
    //yay
  }
  else
  {
    int leftSpd = baseSpeed-delta;
    int RightSpd = baseSpeed+delta;
      
      //  ECE3_read_IR(sensorValues);
      
     analogWrite(left_pwm_pin,leftSpd);
     analogWrite(right_pwm_pin,RightSpd);
  }

  

//  int leftSpd = baseSpeed-delta;
//  int RightSpd = baseSpeed+delta;
//
//  analogWrite(left_pwm_pin,leftSpd);
//  analogWrite(right_pwm_pin,RightSpd);

  float oldSumSensorValues = sumSensorValues;

//  Serial.println();
//  delay(50);

// old LED code
//    LED_timer++;
//    if(LED_timer > 333 && LED_on == false)
//    {
//      digitalWrite(LED_RF, HIGH); 
//      LED_on = true;
//      LED_timer = 0; 
//    }
//    else if (LED_timer > 166 && LED_on == true)
//    {
//      digitalWrite(LED_RF, LOW);
//      LED_on = false;
//      LED_timer = 0;
//    }

//new LED code
  unsigned int currentTime = millis();

  if ((currentTime - prevTime) > 2000 && LED_on == false)
  {
    digitalWrite(LED_RF, HIGH);
    LED_on = true;
    prevTime = currentTime;
  }
  if ((currentTime - prevTime) > 1000 && LED_on == true)
  {
    digitalWrite(LED_RF, LOW);
    LED_on = false;
    prevTime = currentTime;
  }

   
}


void  ChangeBaseSpeed(int initialBaseSpd, int finalBaseSpd) {
/*  
 *   This functin changes the car base speed gradually (in about 300 ms) from
 *   initialspeed to final speed. This non-instantaneous speed change reduces the 
 *   load on the plastic geartrain, and reduces the failure rate of the motors. 
 */
  int numSteps = 5;
  int pwmLeftVal = initialBaseSpd; // initialize left wheel speed 
  int pwmRightVal = initialBaseSpd;  // initialize right wheel speed 
  int deltaLeft = (finalBaseSpd-initialBaseSpd)/numSteps; // left in(de)crement
  int deltaRight = (finalBaseSpd-initialBaseSpd)/numSteps;  // right in(de)crement

  for(int k=0;k<numSteps;k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin,pwmLeftVal);    
    analogWrite(right_pwm_pin,pwmRightVal); 
    delay(60);   
  } // end for int k
} // end void  ChangeBaseSpeed