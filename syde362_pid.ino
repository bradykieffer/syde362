/*
 * Test the rig by periodically turning on each MOSFET and sample a distance with the proximity sensor.
 */
#include <Wire.h>
#include "Adafruit_VCNL4010.h"

Adafruit_VCNL4010 vcnl;

static int FIDELITY = 100;

static int FET_ONE = 3; // The black magnet
static int FET_TWO = 6; // The yellow magnet

double HIGH_PROX_VAL = 0.0;
double LOW_PROX_VAL  = 0.0;

void setupProximitySensor(double &high_val, double &low_val); 
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
  // setupProximitySensor(HIGH_PROX_VAL, LOW_PROX_VAL);
  Serial.println("############ DONE SENSOR CALIBRATION ############");

  // Set up the fets and set them both low.
  pinMode(FET_ONE, OUTPUT);
  pinMode(FET_TWO, OUTPUT);
  
  digitalWrite(FET_ONE, LOW);
  digitalWrite(FET_TWO, LOW);
  Serial.println("############## BEGINNING MAIN LOOP ##############");
  for(int i = 1; i < 6; ++i){
    Serial.println(i);
    delay(1000);  
  }
}


void loop() { 
  int delay_ms = 2500;
  int ledPin = FET_ONE;
  // fade in from min to max in increments of 5 points:
  for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
    // sets the value (range from 0 to 255):
    analogWrite(ledPin, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
    // sets the value (range from 0 to 255):
    analogWrite(ledPin, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  /*
  
  Serial.println("FET_ONE IS HIGH AF");
  digitalWrite(FET_ONE, HIGH);
  delay(delay_ms);
  digitalWrite(FET_ONE, LOW);
  Serial.println("FET_ONE IS LOWWWWW");
  //Serial.print("FET_ONE: Proximity to plate: \t"); Serial.print(pollProximity()); Serial.print("\t\t\t\r");
  delay(delay_ms);
  */
  /*
  Serial.println("FET_TWO IS HIGH AF");
  digitalWrite(FET_TWO, HIGH);
  delay(delay_ms);
  digitalWrite(FET_TWO, LOW);
  Serial.println("FET_TWO IS LOWWWWW");
  Serial.print("FET_TWO: Proximity to plate: \t"); Serial.print(pollProximity()); Serial.print("\t\t\t\r");
  delay(delay_ms);
  */
}

/*
 * User defined functions below ------------------------------------------------------------
 */

/*
 * Poll the proximity from the chip and convert it to a value on the range [-FIDELITY, FIDELITY]
 * e.g. [-100, 100]
 */
double pollProximity(){
  return map(vcnl.readProximity(), HIGH_PROX_VAL, LOW_PROX_VAL, -FIDELITY, FIDELITY);
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
