/*
 * Uses a micro-servo with 3 tubes to drop kibble from a tube through a hole.
 * 
 * - servo
 * - Digital input button for 'feed-me-now', for testing
 * - LED output 
 * 
 * Wakes up every second to check the time and goes back to sleep if nothing
 * to be done. This should save battery power.
 * 
 * Copyright (c) by Hennus Bergman
 * 
 * 
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground
 * 
 * 
 */
#include <Servo.h>

const int BUTTON_PIN = 2;     // the number of the pushbutton pin
const int SERVO_PIN = 9;     // the number of the servo control pin
const int GREEN_LED_PIN =  10;  // cannot PWM pin 10 while using 9 for servo, only on/off
const int RED_LED_PIN =  11;  // because the red LED is much brighter use that for PWM-ing
const int ONBOARD_LED_PIN = 12; // the pin number of the on-baord LED; just to turn it off.

// These are the angles that the servo has to go to in order to release the kibble
const int FEED_POS_CLOSED = 0;
const int FEED_POS_OPEN1 = 53;
const int FEED_POS_OPEN2 = 105;
const int FEED_POS_OPEN3 = 165;

Servo myservo;  // create servo object to control a servo



void setup() {
  Serial.begin(9600);
  // servo setup - use custom settings to get a little over 180 degrees
  myservo.attach(SERVO_PIN, 425, 2500);  // attaches the servo on pin 9 to the servo object
  delay(100);
  // servo_move(1, FEED_POS_CLOSED, 1, 15);
  myservo.write(FEED_POS_CLOSED);

  // initialize the LED pins as output:
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  
  // initialize the pushbutton pin as an input:
  //  pinMode(BUTTON_PIN, INPUT);

  // disable the onboard LED
  digitalWrite(ONBOARD_LED_PIN, LOW);


  delay(5000);
  Serial.println("1");
  feed_cup1();
  delay(5000);
  Serial.println("2");
  feed_cup2();
  delay(5000);
  Serial.println("3");
  feed_cup3();
  delay(5000);
  Serial.println("close");
  feed_close();
}

int redVal = 0;
int ledPhase = 1;
void loop() {
//  Serial.println(redVal);
//  analogWrite(RED_LED_PIN, redVal);
  redVal += ledPhase;
  if (redVal > 255 || redVal < 0) {
    ledPhase = -1 * ledPhase;
  }
  delay(100);
}



void loopX() {
  delay(1000);
}




void feed_cup1() {
  servo_move(FEED_POS_CLOSED, FEED_POS_OPEN1, 1, 15);
}

void feed_cup2() {
  servo_move(FEED_POS_OPEN1, FEED_POS_OPEN2, 1, 15);
}

void feed_cup3() {
  servo_move(FEED_POS_OPEN2, FEED_POS_OPEN3, 1, 15);
  delay(500);
  servo_move(FEED_POS_OPEN3, FEED_POS_OPEN3 + 3, 1, 15);
}

void feed_close() {
  servo_move(FEED_POS_OPEN3, FEED_POS_CLOSED, 1, 15);
}

void servo_move(int from, int to, int stepSize, int stepDelay) {
  Serial.print(" servo_move(");
  Serial.print(from);
  Serial.print(",");
  Serial.print(to);
  Serial.print(",");
  Serial.print(stepSize);
  Serial.print(",");
  Serial.print(stepDelay);
  Serial.println(")");
  int dir = 1;
  if (from > to) {
    dir = -1;
  }
  if (stepSize < 0) {
    stepSize = -1 * stepSize;
  }
  int pos = from;
  while (true) {
    Serial.print("  pos=");
    myservo.write(pos);
    Serial.println(pos);
    delay(stepDelay);
    int newPos = pos + (dir * stepSize);
    if (from > to && newPos < to) {
      break;
    }
    if (from < to && newPos > to) {
      break;
    }
    pos = newPos;
  }

  myservo.write(to);
  delay(stepDelay);

}

