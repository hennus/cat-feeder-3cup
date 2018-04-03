/*
 * Uses a micro-servo with 3 tubes to drop kibble from a tube through a hole.
 * 
 * Wakes up every second to check the time and goes back to sleep if nothing
 * to be done. This should save battery power. 
 * 
 * Copyright (c) by Hennus Bergman
 * 
 * Red LED on pin 11 with resistor 300 ohm
 * Green LED on pin 10 with resistor 300 ohm
 * Servo on pin 9
 * 
 * 
 * The green LED blinks the number of feeds that have been given by the machine.
 * The red LED glows when the third feed is done.
 * 
 * TODO:
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground
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

//const long DELAY_2_AND_A_HALF_HOURS_IN_MILLIS = (5L * 60L * 60L * 1000L) / 2L;
const long DELAY_3_HOURS_IN_MILLIS = (3L * 60L * 60L * 1000L); // three hours
const long DELAY_HALF_HOUR_IN_MILLIS = (30L * 60L * 1000L); // half hour
const long DELAY_HOUR_IN_MILLIS = (60L * 60L * 1000L); // hour
const long DELAY_10_SECONDS_IN_MILLIS = 10 * 1000L; // for testing
const long DELAY_3_HOURS_AND_45_MINS_IN_MILLIS = (3L * 60L * 60L * 1000L) + (45L * 60L * 1000L); // 3h+45min


#if 0
/* 
 * Usage to give 4 meals a day:
 * - manual feed at 10:00, reset feeder, fill 3 cups.
 * - feed cup 1 at 15:00
 * - feed cup 2 at 20:00
 * - feed cup 3 at 01:30
 */
const long START_TIME_IN_HR = 10;
const long DELAY_CUP1 = 5 * DELAY_HOUR_IN_MILLIS;
const long DELAY_CUP2 = DELAY_CUP1 + 5 * DELAY_HOUR_IN_MILLIS;
const long DELAY_CUP3 = DELAY_CUP2 + 5.5 * DELAY_HOUR_IN_MILLIS;
#elif 1
/* 
 * Usage to give 4 meals a day:
 * - manual feed at 8:00, reset feeder, fill 3 cups.
 * - feed cup 1 at 13:00
 * - feed cup 2 at 18:00
 * - feed cup 3 at 22:45
 */
const long START_TIME_IN_HR = 8;
const long DELAY_CUP1 = 5 * DELAY_HOUR_IN_MILLIS;
const long DELAY_CUP2 = DELAY_CUP1 + 5 * DELAY_HOUR_IN_MILLIS;
const long DELAY_CUP3 = DELAY_CUP2 + 4.75 * DELAY_HOUR_IN_MILLIS;
#elif 0
/* 
 * Usage to give 5 meals a day:
 * - manual feed at 8:00, reset feeder, fill 3 cups.
 * - feed cup 1 at 11:30
 * - feed cup 2 at 15:30
 * - feed cup 3 at 18:30
 * - manual feed at 22:30
 */
const long START_TIME_IN_HR = 8;
const long DELAY_CUP1 = 3.5 * DELAY_HOUR_IN_MILLIS;
const long DELAY_CUP2 = DELAY_CUP1 + 4 * DELAY_HOUR_IN_MILLIS;
const long DELAY_CUP3 = DELAY_CUP2 + 3 * DELAY_HOUR_IN_MILLIS;;
#elif 0
/* 
 * Usage to give 5 meals a day:
 * - manual feed at 8:00, reset feeder, fill 3 cups.
 * - feed cup 1 at 11:30
 * - feed cup 2 at 15:30
 * - manual feed at 19:00
 * - feed cup 3 at 22:30  (Prefer latest feed to be automatic to prevent 'behavioural issues'.)
 */
const long START_TIME_IN_HR = 8;
// const double START_TIME_IN_HR = 14.75;
// const long DELAY_CUP1 = DELAY_HOUR_IN_MILLIS * 3 /4;
const long DELAY_CUP1 = DELAY_3_HOURS_IN_MILLIS + DELAY_HALF_HOUR_IN_MILLIS;
const long DELAY_CUP2 = DELAY_CUP1 + DELAY_3_HOURS_IN_MILLIS + (DELAY_HOUR_IN_MILLIS);
const long DELAY_CUP3 = DELAY_CUP2 + 2*DELAY_3_HOURS_IN_MILLIS + (2 * DELAY_HALF_HOUR_IN_MILLIS);
#elif 0
/* 
 * Usage to give 5 meals a day:
 * - manual feed at 8:00, 
 * - manual feed at 11:30, reset feeder, fill 3 cups.
 * - feed cup 1 at 15:30
 * - feed cup 2 at 19:00
 * - feed cup 3 at 22:30
 */
const double START_TIME_IN_HR = 11.5;
const long DELAY_CUP1 = DELAY_3_HOURS_IN_MILLIS + (2 * DELAY_HALF_HOUR_IN_MILLIS);
const long DELAY_CUP2 = DELAY_CUP1 + DELAY_3_HOURS_IN_MILLIS + DELAY_HALF_HOUR_IN_MILLIS;
const long DELAY_CUP3 = DELAY_CUP2 + DELAY_3_HOURS_IN_MILLIS + DELAY_HALF_HOUR_IN_MILLIS;
#elif 0
/* 
 * Usage to dispense 2 meals in the evening
 * - manual feed at 8:00, reset feeder, fill 3 cups.
 * - manual feed at 11:30, 
 * - manual feed at 15:30,
 * - feed cup 1 at 19:00
 * - feed cup 2 at 22:30
 * - feed cup 3 at  8:00
 */
const double START_TIME_IN_HR = 17.5;
const long DELAY_CUP1 = 1.0 * DELAY_HOUR_IN_MILLIS;
const long DELAY_CUP2 = DELAY_CUP1 + 4 * DELAY_HOUR_IN_MILLIS;
const long DELAY_CUP3 = DELAY_CUP2 + 9.5 * DELAY_HOUR_IN_MILLIS;
#else
/* 
 * Usage to give 4 meals a day:
 * - manual feed double dosis in food-ball at 8:00, reset feeder, fill 3 cups.
 * - feed cup 1 at 15:00
 * - feed cup 2 at 18:45
 * - feed cup 3 at 22:30
 */
const long START_TIME_IN_HR = 8;
const long DELAY_CUP1 = DELAY_3_HOURS_AND_45_MINS_IN_MILLIS + (1L * DELAY_3_HOURS_AND_45_MINS_IN_MILLIS) - DELAY_HALF_HOUR_IN_MILLIS;
const long DELAY_CUP2 = DELAY_3_HOURS_AND_45_MINS_IN_MILLIS + (2L * DELAY_3_HOURS_AND_45_MINS_IN_MILLIS) - DELAY_HALF_HOUR_IN_MILLIS;
const long DELAY_CUP3 = DELAY_3_HOURS_AND_45_MINS_IN_MILLIS + (3L * DELAY_3_HOURS_AND_45_MINS_IN_MILLIS) - DELAY_HALF_HOUR_IN_MILLIS;
#endif




enum feeder_state {
  STATE_RESET,
  STATE_CUP1,
  STATE_CUP2,
  STATE_CUP3,
  STATE_DONE
};

enum feeder_state state = STATE_RESET;

Servo myservo;  // create servo object to control a servo

void print_delay(const char* msg, long d) {
  Serial.print("  ");
  Serial.print(msg);
  Serial.print(" ");
  Serial.print(d);
  Serial.print("ms = ");
  float hr = d / 1000.0 / 60.0 / 60.0;
  Serial.print(hr);
  Serial.print("hr @");
  Serial.print(hr + START_TIME_IN_HR);
  Serial.println("hr");
}

void setup() {
  Serial.begin(9600);
  Serial.print("Start at ");
  Serial.print(START_TIME_IN_HR);
  Serial.println("hr");
  print_delay("DELAY_CUP1", DELAY_CUP1);
  print_delay("DELAY_CUP2", DELAY_CUP2);
  print_delay("DELAY_CUP3", DELAY_CUP3);

  // position servo at reset to closed position.
  attach_servo();
  myservo.write(FEED_POS_CLOSED);
  delay(200);
  detach_servo();

  // initialize the LED pins as output:
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  
  // initialize the pushbutton pin as an input:
  //  pinMode(BUTTON_PIN, INPUT);

  // disable the onboard LED to save power
  digitalWrite(ONBOARD_LED_PIN, LOW);

  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, LOW);
}

// Current PWM value for red LED initial value must be between upper and lower limits
int redVal = 10;
// step direction for redLed value adjustments
int ledPhase = 1;

void loop() {
  switch (state) {
    case STATE_RESET: {
      // wait for CUP1 time
      if (millis() > DELAY_CUP1) {
        state = STATE_CUP1;
        Serial.println("cup 1");
        feed_cup1();
      }
      break;
    }
    case STATE_CUP1: {
      // wait for CUP2 time
      if (millis() > DELAY_CUP2) {
        state = STATE_CUP2;
        Serial.println("cup 2");
        feed_cup2();
      } else {
        blink_green(1);
      }
      break;
    }
    case STATE_CUP2: {
      // wait for CUP3 time
      if (millis() > DELAY_CUP3) {
        state = STATE_CUP3;
        Serial.println("cup 3");
        feed_cup3();
      } else {
        blink_green(2);
      }
      break;
    }
    case STATE_CUP3: {
      state = STATE_DONE;
      // done; turn the green light off.
      digitalWrite(GREEN_LED_PIN, LOW);
      Serial.println("done");
      feed_close();
      break;
    }

    case STATE_DONE: {
      // maintain RED light
      analogWrite(RED_LED_PIN, redVal);
      redVal += ledPhase;
      Serial.print("redVal=");
      Serial.println(redVal);
      if (redVal >= 30 || redVal <= 1) {
        ledPhase = -1 * ledPhase;
        Serial.print("flip phase to ");
        Serial.println(ledPhase);
      }
      delay(100);
      break;
    }

  }
  
  // sleep one second when not needing to maintain the RED light status
  if (state != STATE_DONE) {
    delay(1000);
  }
}

void blink_green(int numTimes) {
  delay(1000);
  for (int i=0; i < numTimes; i++) {
    delay(300);
    digitalWrite(GREEN_LED_PIN, LOW);
    delay(200);
    digitalWrite(GREEN_LED_PIN, HIGH);
    delay(300);
  }
  delay(1000);
}


void attach_servo() {
  // servo setup - use custom settings to get a little over 180 degrees
  myservo.attach(SERVO_PIN, 425, 2500);  // attaches the servo on pin 9 to the servo object
  delay(100);
}

void detach_servo() {
  // release servo control to keep the servo engine quiet.
  myservo.detach();
}

void feed_cup1() {
  attach_servo();
  servo_move(FEED_POS_CLOSED, FEED_POS_OPEN1, 1, 15);
  detach_servo();
}

void feed_cup2() {
  attach_servo();
  servo_move(FEED_POS_OPEN1, FEED_POS_OPEN2, 1, 15);
  detach_servo();
}

void feed_cup3() {
  attach_servo();
  servo_move(FEED_POS_OPEN2, FEED_POS_OPEN3, 1, 15);
  detach_servo();
}

void feed_close() {
  attach_servo();
  servo_move(FEED_POS_OPEN3, FEED_POS_CLOSED, 1, 15);
  detach_servo();
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

