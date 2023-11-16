#include <ECE3.h>

// array for storing sensor value of previous tick and current tick
uint16_t sensorValues[8];
uint16_t prev[8];

// left wheel pin definitions
const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

// right wheel pin definitions
const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

// LED pin definition
const int LED_RF = 41;

// variables storing wheel speed (in pwm); value range: (0,255)
int left_spd;
int right_spd;

int next_left_spd;
int next_right_spd;

// variable related to turn amount
// - turn amount (turn_amt) = Kp * error + Kd * dError (change in error)
float error = 0;
float prevError = 0;
float deltaError = error - prevError;
int turn_amt = 0;
int max_turn_amt = 0;


// constants storing values needed to calculate turn amount
const float mins[8] = {483.0,506,628.6,510.8,437.0,460.0,460.0,599.0};
const float maxs[8] = {1993.0,1736.8,1871.4,1329.0,1159.0,1098.0,903.8,1901.0};
const float weights[8] = {-15.0, -14.0, -12.0, -8.0, 8.0, 12.0, 14.0, 15.0};


// Base speed, Kp and Kd values (subject to change over iterations)

//Kp should be a value so that max error (around +/- 20.15) yeilds BASE_SPD / 2;
//
const float BASE_SPD = 30.0;
const float Kp = (BASE_SPD * 0.5) / 20.15;
const float Kd = 0.0;

// bools that store 
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


// each weighting value calculated by equation
// weight * ((raw_value - min_value) * 10) / max_value
// sum all and divide by 8 to get turnAmount

float turnAmount(uint16_t x[]) {
  float sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += (weights[i] * ((x[i] - mins[i]) * 10.0) / maxs[i]);
  }
  return sum / 8.0;
}

bool darkLineDetected(uint16_t x[]) {
  float threshold = 1600;
  for (int i = 0; i < 8; i++)
  {
    if (x[i] < threshold)
      return false;
  }
  return true;
}


void loop()
{
  // read raw sensor values

  ECE3_read_IR(sensorValues);
  //sensorValues[i]

  //check to see if the car is reading a black line across all sensors (start or finish line)
  if (darkLineDetected(prev) && darkLineDetected(sensorValues)) {
    if (!started) {
      digitalWrite(LED_RF,HIGH);
      delay(50);
      digitalWrite(LED_RF,LOW);
      left_spd = BASE_SPD;
      right_spd = BASE_SPD;
      started = true;
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

  // calculates error & dError and turn amount using equation:
  //  - turn_amt = -Kp * error + -Kd * dError
  error = turnAmount(sensorValues);
  deltaError = error - prevError;
  turn_amt = (-1.0) * Kp * error + (-1.0) * Kd * deltaError;

  // set prevError as current error for next loop
  prevError = error;

  // calculate next speed for each wheel for boundary check
  next_left_spd = left_spd + turn_amt;
  next_right_spd = right_spd - turn_amt;
  
  if (started) {
    if (next_left_spd >= (BASE_SPD - BASE_SPD * 0.5) && next_left_spd < (BASE_SPD + BASE_SPD * 0.5)) {
      left_spd = next_left_spd;
    }
    if (next_right_spd >= (BASE_SPD - BASE_SPD * 0.5) && next_right_spd < (BASE_SPD + BASE_SPD * 0.5)) {
      right_spd = next_right_spd;
    }
  }

//  Serial.print("Turn amount      : ");
//  Serial.println(turn_amt);
//  Serial.print("Right wheel speed: ");
//  Serial.println(right_spd); 
//  Serial.print("Left wheel speed : ");
//  Serial.println(left_spd);
//  Serial.println();
  analogWrite(left_pwm_pin,left_spd);
  analogWrite(right_pwm_pin,right_spd);

  for (int i = 0; i < 8; i++) {
    prev[i] = sensorValues[i];  
  }
  //delay(1000);

}
