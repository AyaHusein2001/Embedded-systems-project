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
const int most_left_sensor = 13;  
const int left_sensor = 14; 
const int middle_sensor = 27; 
const int right_sensor = 26;
const int most_right_sensor = 25;

int initial_motor_speed = 80;

float error = 0;
float P, I, D = 0;
float PID = 0;
float Kp = 15, Kd = 47, Ki = 0.001;
float previous_error = 0, previous_I = 0;


int sensors_values[5] = {0, 0, 0, 0, 0}; // most left => most right

void forward()  
{
  digitalWrite(left_motor_in2, HIGH);
  digitalWrite(left_motor_in1, LOW);
  digitalWrite(right_motor_in1, HIGH);
  digitalWrite(right_motor_in2, LOW);
}

void turnLeft()
{
  digitalWrite(left_motor_in1, LOW);   // Right Motor forword Pin
  digitalWrite(left_motor_in2, LOW);   // Right Motor backword Pin
  digitalWrite(right_motor_in1, HIGH); // Left Motor backword Pin
  digitalWrite(right_motor_in2, LOW);  // Left Motor forword Pin
}

void turnRight()
{
  digitalWrite(left_motor_in1, LOW);   // Right Motor forword Pin
  digitalWrite(left_motor_in2, HIGH);  // Right Motor backword Pin
  digitalWrite(right_motor_in1, LOW);  // Left Motor backword Pin
  digitalWrite(right_motor_in2, LOW);  // Left Motor forword Pin
}

void Stop()
{
  digitalWrite(left_motor_in1, LOW);  // Right Motor forword Pin
  digitalWrite(left_motor_in2, LOW);  // Right Motor backword Pin
  digitalWrite(right_motor_in1, LOW); // Left Motor backword Pin
  digitalWrite(right_motor_in2, LOW); // Left Motor forward Pin
}

void calculatePID(){

  P = error;
  I = I + error;
  D = error - previous_error;
  // if(D != 0)
  // { Stop();
  //   while(true);}

  Serial.print("Error: ");
  Serial.println(error);
  
  Serial.print("Previous rror: ");
  Serial.println(previous_error);
  Serial.print("P: ");
  Serial.println(P);
  Serial.print("D: ");
  Serial.println(D);
  Serial.print("I: ");
  Serial.println(I);

  PID = (P * Kp) + (I * Ki) + (D * Kd);

  Serial.print("PID: ");
  Serial.println(PID);

  previous_I = I;
  previous_error = error;
}

void controlMotorSpeed(){

  int left_motor_speed = initial_motor_speed + PID;
  int right_motor_speed = initial_motor_speed - PID;

  // The motor speed should not exceed the max PWM value
  constrain(left_motor_speed, 0, 255);
  constrain(right_motor_speed, 0, 255);

  analogWrite(left_motor_enable, left_motor_speed);
  analogWrite(right_motor_enable, right_motor_speed);

  Serial.print("Speed left: ");
  Serial.println(left_motor_speed);
  Serial.print("Speed right: ");
  Serial.println(right_motor_speed);

  forward();
}

void readSensors(){

  // read sensors
  sensors_values[0] = digitalRead(most_left_sensor);
  sensors_values[1] = digitalRead(left_sensor);
  sensors_values[2] = digitalRead(middle_sensor);
  sensors_values[3] = digitalRead(right_sensor);
  sensors_values[4] = digitalRead(most_right_sensor);

  // calculate error
  if((sensors_values[0] == 0) && (sensors_values[1] == 0) && (sensors_values[2] == 0) && (sensors_values[3] == 0) && (sensors_values[4] == 1)) error = 4;

  else if((sensors_values[0] == 0) && (sensors_values[1] == 0) && (sensors_values[2] == 0) && (sensors_values[3] == 1) && (sensors_values[4] == 1)) error = 3; 

  else if((sensors_values[0] == 0) && (sensors_values[1] == 0) && (sensors_values[2] == 0) && (sensors_values[3] == 1) && (sensors_values[4] == 0)) error = 2;

  else if((sensors_values[0] == 0) && (sensors_values[1] == 0) && (sensors_values[2] == 1) && (sensors_values[3] == 1) && (sensors_values[4] == 0)) error = 1;

  else if((sensors_values[0] == 0) && (sensors_values[1] == 0 ) && (sensors_values[2] == 1) && (sensors_values[3] == 0) && (sensors_values[4] == 0)) error = 0;

  else if((sensors_values[0] == 0) && (sensors_values[1] == 1) && (sensors_values[2] == 1) && (sensors_values[3] == 0) && (sensors_values[4] == 0)) error =- 1;

  else if((sensors_values[0] == 0) && (sensors_values[1] == 1) && (sensors_values[2] == 0) && (sensors_values[3] == 0) && (sensors_values[4] == 0)) error = -2;

  else if((sensors_values[0] == 1) && (sensors_values[1] == 1 ) && (sensors_values[2] == 0 ) && (sensors_values[3] == 0) && (sensors_values[4] == 0)) error = -3;

  else if((sensors_values[0] == 1) && (sensors_values[1] == 0) && (sensors_values[2] == 0) && (sensors_values[3] == 0 ) && (sensors_values[4] == 0)) error = -4;

  Serial.print("Error: ");
  Serial.println(error);
}

void printSensors(){
  Serial.print("Most left: ");
  Serial.println(sensors_values[0]); // Left
  Serial.print("Left: ");
  Serial.println(sensors_values[1]); // Middle Left
  Serial.print("Middle: ");
  Serial.println(sensors_values[2]); // Middle Right
  Serial.print("Right: ");
  Serial.println(sensors_values[3]); // Right
  Serial.print("Most right: ");
  Serial.println(sensors_values[4]);
  Serial.println("---------------------------");
}



void setup() {
  // put your setup code here, to run once:

  // Setting up pinmodes of Sensors
  Serial.begin(115200);

  // setup sensors pinmodes as inputs
  pinMode(most_left_sensor, INPUT);
  pinMode(left_sensor, INPUT);
  pinMode(middle_sensor, INPUT);
  pinMode(right_sensor, INPUT);
  pinMode(most_right_sensor, INPUT);

  // Setting up pinmodes of motors
  pinMode(left_motor_in1, OUTPUT);
  pinMode(left_motor_in2, OUTPUT);
  pinMode(right_motor_in1, OUTPUT);
  pinMode(right_motor_in2, OUTPUT);

    // analogWrite(left_motor_enable, 100); //Left Motor Speed
    // analogWrite(right_motor_enable, 100); //Right Motor Speed 
}

void loop(){

  // put your main code here, to run repeatedly:
  readSensors();
  printSensors();
  calculatePID();
  controlMotorSpeed();

}