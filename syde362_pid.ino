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
double Kp = 32609.0;
double Ki = 34517.0;
double Kd = 7086;

PID pid(&Input, &Output, &Setpoint, Kp,Ki,Kd, DIRECT);

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
  for(int i = 5; i > 0; --i){
    Serial.print(i); Serial.print("\r");
    delay(1000);  
  }
}


void loop() { 
  
  double yellow_dist = pollProximity();
  yellow_dist /= 10000;
  double black_dist = -yellow_dist;
  if(yellow_dist > 0.003){
    Input = yellow_dist;  
  }else if(black_dist > 0.003){
    Input = black_dist;
  }else{
    Input = 0.0;  
  }
  pid.Compute();
  Serial.print("Output of PID: "); Serial.println(Output, DEC);
  Serial.print("Input to PID: "); Serial.println(Input, DEC);
  Serial.print("Ylw: "); Serial.print(yellow_dist, DEC); Serial.print("\tBlk: "); Serial.println(black_dist, DEC);
  delay(1000);
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
  Serial.print("4\r"); delay(1000);
  Serial.print("3\r"); delay(1000);
  Serial.print("2\r"); delay(1000);
  digitalWrite(pin, HIGH);
  Serial.print("1\r"); delay(1000);
  high_val = vcnl.readProximity();
  digitalWrite(pin, LOW);
  
  Serial.println("Push the cart all the way LOW");
  Serial.print("4\r"); delay(1000);
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
  pid.SetOutputLimits(0, 1);

  //turn the PID on
  pid.SetMode(AUTOMATIC);  
}
