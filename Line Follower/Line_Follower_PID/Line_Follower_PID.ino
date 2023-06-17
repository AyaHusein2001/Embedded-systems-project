#include <rom/ets_sys.h>

// IR SENSORS
const int most_right_sensor = 25;
const int right_sensor = 26;
const int middle_sensor = 27;
const int left_sensor = 14;
const int most_left_sensor = 13;

// LEFT MOTOR PINS
const int left_motor_in1 = 22; // ground 
const int left_motor_in2 = 23; //

// RIGHT MOTOR PINS
const int right_motor_in1 = 19; //
const int right_motor_in2 = 18; // ground

// LEFT ENABLE
const int left_motor_enable = 17;

// RIGHT ENABLE
const int right_motor_enable = 16;

int initial_motor_speed = 100;

void controlMotorSpeed(int left_speed, int right_speed){

  right_speed = -right_speed;

  if(left_speed < 0)
    analogWrite(left_motor_enable, -left_speed);
  else
    analogWrite(left_motor_enable, left_speed);
  
  if(right_speed < 0)
    analogWrite(right_motor_enable, -right_speed);
  else
    analogWrite(right_motor_enable, right_speed);
}

int error = 0;
void calculateError(int sensors_reading){
  switch(sensors_reading){
    case 0b00000:
      error = error;
      break;
    
    case 0b11111:
      error = 0;
      break;

    case 0b00010:
    case 0b00110:
      error = 1;
      break;

    case 0b00001:
    case 0b00011:
    case 0b00111:
      error = 2;
      break;
    
    case 0b00100:
      error = 0;
      break;

    case 0b01000:
    case 0b01100:
      error = -1;
      break;

    case 0b10000:
    case 0b11000:
    case 0b11100:
      error = -2;
      break;    

    default:
      error = error;
      break;
  }

  float difference = calculatePID(error);
  controlMotorSpeed(initial_motor_speed - difference, initial_motor_speed + difference);
}

int sensor_readings[5] = {0, 0, 0, 0, 0};
int readSensors(){
  int sensors_reading = 0b00000;

  sensor_readings[0] = digitalRead(most_right_sensor);
  sensor_readings[1] = digitalRead(right_sensor);
  sensor_readings[2] = digitalRead(middle_sensor);
  sensor_readings[3] = digitalRead(left_sensor);
  sensor_readings[4] = digitalRead(most_left_sensor);

  Serial.print("Most Left: ");
  Serial.println(sensor_readings[0]); // Middle Left
  Serial.print("Left: ");
  Serial.println(sensor_readings[1]); // Middle Left
  Serial.print("Middle: ");
  Serial.println(sensor_readings[2]); // Middle Right
  Serial.print("Right: ");
  Serial.println(sensor_readings[3]); // Right
  Serial.print("Most Right: ");
  Serial.println(sensor_readings[4]); // Right
  Serial.println("-------------------------------------------"); // Right
  
  for(int i = 0; i < 5; i++) 
    if(sensor_readings[i] == 1) 
      sensors_reading += (0x1<< i); 
  
  return sensors_reading;
}
// PID parameters
float Kp, Kd, Ki;
float P, I, D;
float previous_error, previous_integral;

float calculatePID(float error){
  float PID = 0;
  P = P * 0.7 + error * 0.3;
  D = P - previous_error;
  I = constrain(P + previous_integral, -50, 50);

  PID = Kp * P + Kd * D + Ki * I;

  previous_error = P;

  return PID;
}

void setup() {
  // put your setup code here, to run once:
  // Setting up pinmodes of Sensors
  Serial.begin(115200);

  // setup sensors pinmodes as inputs
  pinMode(left_sensor, INPUT);
  pinMode(middle_sensor, INPUT);
  pinMode(right_sensor, INPUT);

  // Setting up pinmodes of motors
  pinMode(left_motor_in1, OUTPUT);
  pinMode(left_motor_in2, OUTPUT);
  pinMode(right_motor_in1, OUTPUT);
  pinMode(right_motor_in2, OUTPUT);

  analogWrite(left_motor_enable, initial_motor_speed);
  analogWrite(right_motor_enable, initial_motor_speed);
}

int position;
void loop() {
  // put your main code here, to run repeatedly:
  delay(4);
  position = readSensors();
  calculateError(position);
}
