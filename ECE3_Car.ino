#include <ECE3.h>

uint16_t sensorValues[8];
const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int LED_RF = 41;
int left_spd;
int right_spd;
const float Kp = 0.1;

const float mins[8] = {483,506,628.6,510.8,437,460,460,599};
const float maxs[8] = {1993,1736.8,1871.4,1329,1159,1159,903.8,1901};
const float weights[8] = {-19.0, -17.0, -13.0, -9.0, 8.0, 12.0, 14.0, 15.0};
uint16_t prev[8];

bool started;
bool uturn;

void setup()
{
  // Pin Settings
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  // Setting Initial Values
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  pinMode(LED_RF, OUTPUT);

  left_spd = 0;
  right_spd = 0;

  started = false;
  uturn = false;
  
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(50);
}

float turnAmount(uint16_t x[]) {
  
  float sum = 0;
  for (int i = 0; i < 8; i++) {
    sum = sum + (weights[i] * (x[i] - mins[i]) / maxs[i]);
  }
  return sum;
}


void loop()
{
  // read raw sensor values

  ECE3_read_IR(sensorValues);
  //sensorValues[i]

  //check to see if the car is reading a black line across all sensors (start or finish line)
  if (sensorValues[0] > 1600 && sensorValues[1] > 1600 &&
  sensorValues[2] > 1600 && sensorValues[3] > 1600 &&
  sensorValues[4] > 1600 && sensorValues[5] > 1600 &&
  sensorValues[6] > 1600 && sensorValues[7] > 1600 &&
  prev[0] > 1600 && prev[1] > 1600 &&
  prev[2] > 1600 && prev[3] > 1600 &&
  prev[4] > 1600 && prev[5] > 1600 &&
  prev[6] > 1600 && prev[7] > 1600) {
    if (!started) {
      digitalWrite(LED_RF,HIGH);
      delay(50);
      digitalWrite(LED_RF,LOW);
      started = true;
      left_spd = 30;
      right_spd = 30;
      
    }
    else {
      uturn == true;
      //uturn
    }
  }
  //have a bool to tell if the car has started so when we see this line we know whether it is the start or end of the track
  //if at the start, start moving forward
  //if at the end, do a donut

  //use the amount below to change the speed of the wheels

  float error = turnAmount(sensorValues);
  int turn_amt = Kp * error;
  if (started) {
    Serial.println(turn_amt);
      if (left_spd + (turn_amt) >= 15 && left_spd + (turn_amt) < 45) {
        left_spd = left_spd - (Kp * error);
      }
      if (right_spd - (turn_amt) < 45 && right_spd - (turn_amt) >= 15) {
        right_spd = right_spd + (turn_amt);
      }
    
    

  }

  //Serial.print(right_spd);
  //Serial.print(" ");
  //Serial.println(left_spd);
  analogWrite(left_pwm_pin,left_spd);
  analogWrite(right_pwm_pin,right_spd);

  for (int i = 0; i < 8; i++) {
    prev[i] = sensorValues[i];
    
  }
  delay(50);

}
