#include <rom/ets_sys.h>
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

// IR SENSORS
const int left_sensor = 14; 
const int middle_sensor = 27; 
const int right_sensor = 26;

int motor_speed = 85;

int speed_difference = 20;

int sensors_values[3] = {0, 0, 0}; // most left => most right

void forward()  
{
  digitalWrite(left_motor_in2, HIGH);
  digitalWrite(left_motor_in1, LOW);
  digitalWrite(right_motor_in1, HIGH);
  digitalWrite(right_motor_in2, LOW);

  analogWrite(left_motor_enable, motor_speed);
  analogWrite(right_motor_enable, motor_speed);
}

void turnLeft()
{
  digitalWrite(left_motor_in1, LOW);   // Right Motor forword Pin
  digitalWrite(left_motor_in2, LOW);   // Right Motor backword Pin
  digitalWrite(right_motor_in1, HIGH); // Left Motor backword Pin
  digitalWrite(right_motor_in2, LOW);  // Left Motor forword Pin

  // analogWrite(left_motor_enable, motor_speed - speed_difference);
  analogWrite(right_motor_enable, motor_speed);// - speed_difference);
}

void turnRight()
{
  digitalWrite(left_motor_in1, LOW);   // Right Motor forword Pin
  digitalWrite(left_motor_in2, HIGH);  // Right Motor backword Pin
  digitalWrite(right_motor_in1, LOW);  // Left Motor backword Pin
  digitalWrite(right_motor_in2, LOW);  // Left Motor forword Pin

  analogWrite(left_motor_enable, motor_speed);// + speed_difference);
  // analogWrite(right_motor_enable, motor_speed - speed_difference);
}

void Stop()
{
  digitalWrite(left_motor_in1, LOW);  // Right Motor forword Pin
  digitalWrite(left_motor_in2, LOW);  // Right Motor backword Pin
  digitalWrite(right_motor_in1, LOW); // Left Motor backword Pin
  digitalWrite(right_motor_in2, LOW); // Left Motor forward Pin
}

void readSensors(){

  // read sensors
  sensors_values[0] = digitalRead(left_sensor);
  sensors_values[1] = digitalRead(middle_sensor);
  sensors_values[2] = digitalRead(right_sensor);
}

void printSensors(){
  Serial.print("Left: ");
  Serial.println(sensors_values[0]); // Middle Left
  Serial.print("Middle: ");
  Serial.println(sensors_values[1]); // Middle Right
  Serial.print("Right: ");
  Serial.println(sensors_values[2]); // Right
  Serial.println("-------------------------------------------"); // Right
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

  analogWrite(left_motor_enable, motor_speed);
  analogWrite(right_motor_enable, motor_speed);
}

void loop() {
  // put your main code here, to run repeatedly:
  readSensors();
  printSensors();
  // forward();
  if((sensors_values[0] == 0 && sensors_values[1] == 1 && sensors_values[2] == 0) || (sensors_values[0] == 1 && sensors_values[1] == 1 && sensors_values[2] == 1) ){
    Serial.println("Moving Forward");
    forward();
  }
  else if((sensors_values[0] == 0 && sensors_values[1] == 0 && sensors_values[2] == 1)){// || (sensors_values[0] == 0 && sensors_values[1] == 1 && sensors_values[2] == 1)){
    Serial.println("Turning Right");
    turnRight();
  }
  else if((sensors_values[0] == 1 && sensors_values[1] == 0 && sensors_values[2] == 0)){// || (sensors_values[0] == 1 && sensors_values[1] == 1 && sensors_values[2] == 0)){
    Serial.println("Turning Left");
    turnLeft();
  }
}
