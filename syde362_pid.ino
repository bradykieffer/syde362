/*
 * Test the rig by periodically turning on each MOSFET and sample a distance with the proximity sensor.
 */
#include <Wire.h>
#include "Adafruit_VCNL4010.h"
#include <PID_v1.h>

Adafruit_VCNL4010 vcnl;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 500.0; //32609.0;
double Ki = 1.0;//5.0;   //34517.0;
double Kd = 1.0;//3.0;   //7086;

PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

static double FIDELITY = 157.35;

static int FET_ONE = 3; // The black magnet
static int FET_TWO = 5; // The yellow magnet

double HIGH_PROX_VAL = 0.0;
double LOW_PROX_VAL  = 0.0;

int WindowSize = 5000;
unsigned long windowStartTime;

void setupProximitySensor(double &high_val, double &low_val); 
void setupPID();
double pollProximity();
void pwm(int pin, double high, double low);

void setup() {
  Serial.begin(9600);
  Serial.print("\eSP F");  // tell to use 7-bit control codes
  Serial.print("\e[?25l"); // hide cursor
  Serial.print("\e[?12l"); // disable cursor highlighting
  
  Serial.println("############### BEGINNING PROGRAM ###############");
  Serial.println("############### CHECKING FET PINS ###############");
  if(FET_ONE == FET_TWO){
    Serial.println("Both FETs are the same :(");
    while(1);  
  }
  Serial.println("############## SENSOR CALIBRATION ###############");
  setupProximitySensor(HIGH_PROX_VAL, LOW_PROX_VAL);
  Serial.println("############ DONE SENSOR CALIBRATION ############");

  Serial.println("################### SET UP PID ##################");
  setupPID();
  Serial.println("################### DONE PID ####################");

  // Set up the fets and set them both low.
  pinMode(FET_ONE, OUTPUT);
  pinMode(FET_TWO, OUTPUT);
  
  digitalWrite(FET_ONE, LOW);
  digitalWrite(FET_TWO, LOW);
  Serial.println("############## BEGINNING MAIN LOOP ##############");
  for(int i = 2; i > 0; --i){
    Serial.print(i); Serial.print("\r");
    delay(1000);  
  }
}

int trigger_pin = -1;
int prev_pin = -1;

double yellow_dist = 0;
double black_dist  = 0;

double yellow_dist_old = 0;
double black_dist_old  = 0;

double yellow_dist_old_old = 0;
double black_dist_old_old  = 0;

void loop() { 
  
  yellow_dist_old_old = yellow_dist_old;
  black_dist_old_old  = black_dist_old;

  yellow_dist_old = yellow_dist;
  black_dist_old  = black_dist;
  
  yellow_dist = pollProximity();       // FET_TWO pin = 5
  black_dist = FIDELITY - yellow_dist - 1.21031; // FET_ONE pin = 3
  
  yellow_dist /= 10000;
  black_dist  /= 10000;

  double yellow_dist_ave = (yellow_dist_old_old + yellow_dist_old + yellow_dist) / 3.0;
  double black_dist_ave = (black_dist_old_old + black_dist_old + black_dist) / 3.0;
  
  if(yellow_dist_ave > 0.0){
    Input = yellow_dist_ave;
    trigger_pin = FET_TWO;
  }else if(black_dist_ave > 0.0){
    Input = black_dist_ave;
    trigger_pin = FET_ONE;
  }else{
    Input = 0.0;  
    trigger_pin = -1;
  }

  // Input *= -1.0;

  if(trigger_pin != prev_pin){
    analogWrite(prev_pin, 0);  
  }
  pid.Compute();
  double scaled_output = 0.0;
  if(trigger_pin > 0){
    scaled_output = map(Output, 0, 7, 0, 255);
    analogWrite(trigger_pin, scaled_output);
  }
  
  Serial.print("Input: "); Serial.print(Input, DEC); Serial.print("\t Output: "); Serial.print(Output, DEC); 
  Serial.print("\tTriggering: "); Serial.print(trigger_pin); Serial.print("\tPrev: "); Serial.print(prev_pin); 
  Serial.print("\tScaled Out: "); Serial.print(scaled_output); Serial.print("\t\t\r");
  
  prev_pin = trigger_pin;
}

/*
 * User defined functions below ------------------------------------------------------------
 */

/*
 * Poll the proximity from the chip and convert it to a value on the range [-FIDELITY, FIDELITY]
 * e.g. [-100, 100]
 */
double pollProximity(){
  return map(vcnl.readProximity(), HIGH_PROX_VAL, LOW_PROX_VAL, FIDELITY, -FIDELITY) + (FIDELITY / 2.0);
}

/*
 * Define an easy way of mocking PWM on a MOSFET. This is probably useless.
 */
void pwm(int pin, double high, double low){
  digitalWrite(pin, HIGH);
  delay(high); 
  digitalWrite(pin, LOW);
  delay(low); 
}

/*
 * Set up the proximity sensor. Thank Andrew for this weirdly easy idea.
 */
void setupProximitySensor(double &high_val, double &low_val){
  int pin =  13;
  pinMode(pin, OUTPUT);
  
  Serial.println("VCNL4010 test");

  if (! vcnl.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.println("Found VCNL4010");  
  
  digitalWrite(pin, LOW);
  Serial.println("Push the cart all the way HIGH");
  Serial.print("3\r"); delay(1000);
  Serial.print("2\r"); delay(1000);
  digitalWrite(pin, HIGH);
  Serial.print("1\r"); delay(1000);
  high_val = vcnl.readProximity();
  digitalWrite(pin, LOW);
  
  Serial.println("Push the cart all the way LOW");
  Serial.print("3\r"); delay(1000);
  Serial.print("2\r"); delay(1000);
  
  digitalWrite(pin, HIGH);
  Serial.print("1\r"); delay(1000);
  low_val = vcnl.readProximity();
  digitalWrite(pin, LOW);  

  Serial.println();
  Serial.print("I'll use the high value: "); Serial.println(high_val);
  Serial.print("and the low value: ");       Serial.println(low_val);
}

void setupPID(){
  //initialize the variables we're linked to
  Setpoint = 0;

  //tell the PID to range between 0 and the full window size
  pid.SetOutputLimits(0, 7);

  //turn the PID on
  pid.SetMode(AUTOMATIC);  
  pid.SetSampleTime(1);
}
