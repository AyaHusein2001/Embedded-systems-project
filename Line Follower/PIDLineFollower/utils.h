#include <HardwareSerial.h>
#include <Arduino.h>


//--------------------------------//
//---------- Sensor pins----------//
//--------------------------------//
#define IR0 25  // Most right sensor
#define IR1 26
#define IR2 27
#define IR3 14
#define IR4 13  // Most left sensor

#define _numSensors 5
static int last_value = 0;

//--------------------------------//
//---------- Motor pins----------//
//--------------------------------//

#define IN1 23
#define IN2 22
#define IN3 19
#define IN4 18
#define left_motor 17
#define right_motor 16
#define MAX_SPEED 100

void read_sensors(int *sensors_readings, int *sensors_readings_analog) {

  sensors_readings_analog[0] = analogRead(IR0);  // most right sensor
  sensors_readings_analog[1] = analogRead(IR1);
  sensors_readings_analog[2] = analogRead(IR2);
  sensors_readings_analog[3] = analogRead(IR3);
  sensors_readings_analog[4] = analogRead(IR4);  // most left sensor

  if (sensors_readings_analog[0] > 3000) sensors_readings[0] = 1;
  else
    sensors_readings[0] = 0;
  if (sensors_readings_analog[1] > 3000) sensors_readings[1] = 1;
  else sensors_readings[1] = 0;
  if (sensors_readings_analog[2] > 3000) sensors_readings[2] = 1;
  else sensors_readings[2] = 0;
  if (sensors_readings_analog[3] > 3000) sensors_readings[3] = 1;
  else sensors_readings[3] = 0;
  if (sensors_readings_analog[4] > 3000) sensors_readings[4] = 1;
  else sensors_readings[4] = 0;
  Serial.println("--------Analog----------");
  for (int i = 4; i > -1; i--) {
    Serial.print(sensors_readings_analog[i]);
    Serial.print("  ");
  }

  Serial.println(" ");
}



unsigned read_line(int *sensors_readings) {
  unsigned char i;
  bool on_line = false;
  unsigned long avg;  // this is for the weighted total, which is long
                      // before division
  unsigned int sum;   // this is for the denominator which is <= 64000
  // static int last_value = 0; // assume initially that the line is left.

  avg = 0;
  sum = 0;

  for (i = 0; i < _numSensors; i++) {
    int value = sensors_readings[i];
    Serial.print("i ");
    Serial.println(sensors_readings[i]);


    // keep track of whether we see the line at all
    if (value != 0) {
      Serial.println("3al 5attttttt");
      on_line = true;

      avg += (long)(value) * (i * 1000);
      sum += value;
    }
  }

  if (!on_line) {
    Serial.println("Shit");
    // If it last read to the left of center, return 0.
    if (last_value < (_numSensors - 1) * 500)
      return 0;

    // If it last read to the right of center, return the max.
    else
      return (_numSensors - 1) * 500;
  }
  Serial.print("average ");
  Serial.println(avg);
  Serial.print("sum ");
  Serial.println(sum);
  last_value = (avg / sum) / 2; //  to play within a lower range
  Serial.print("Value ");
  Serial.println(last_value);
  return last_value;
}

void print_readings(int *sensors_readings, int proportional) {
  Serial.print("sensors: ");
  for (int i = 4; i > -1; i--) {
    Serial.print(sensors_readings[i]);
    Serial.print("  ");
  }



  Serial.print("  position: ");
  Serial.println(proportional);

  Serial.println(analogRead(IR1));
}


void setup_motors_pins() {
  // define motors output pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // PWM pins for motors speed
  pinMode(left_motor, OUTPUT);
  pinMode(right_motor, OUTPUT);
}

void init_motors_speeds() {
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);

  analogWrite(left_motor, MAX_SPEED);
  analogWrite(right_motor, MAX_SPEED);
}

void change_motors_speed(int left_motor_speed, int right_motor_speed) {
  analogWrite(left_motor, left_motor_speed);
  analogWrite(right_motor, right_motor_speed);
  Serial.print("left_motor_speed ");
  Serial.println(left_motor_speed);
  Serial.print("right_motor_speed ");
  Serial.println(right_motor_speed);
}
