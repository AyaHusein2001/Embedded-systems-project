#define LEDC_CHANNEL_0 0 
#define LEDC_CHANNEL_1 1
#define LEDC_TIMER_13_BIT 8 
#define LEDC_BASE_FREQ 5000
#define velocity 100

byte sensors_values[5] = {0, 0, 0, 0, 0};
// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

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

int r=0;
int rr=0;

int l=0;
int ll=0;

int i=0;

int rz=0;
int lz=0;

int counter=0;

void Push(){
  ledcWrite(LEDC_CHANNEL_0,170);  
  ledcWrite(LEDC_CHANNEL_1,170);
  delay(1);
}
void Forward(){
  RightForward();
  LeftForward();
  // if(rz==1){
  //   ledcWrite(LEDC_CHANNEL_1,170);
  //   rz=0;
  // }
  // if(lz==1){
  //   ledcWrite(LEDC_CHANNEL_0,170);
  //   lz=0;
  // }
  ledcWrite(LEDC_CHANNEL_0,velocity);  
  ledcWrite(LEDC_CHANNEL_1,velocity);  
}

void Stop()
{
  ledcWrite(LEDC_CHANNEL_0, 0);  
  ledcWrite(LEDC_CHANNEL_1, 0);  
}

void TurnLeft(){
  //Push();
  // if(rz==1){
  //   ledcWrite(LEDC_CHANNEL_1,170);
  //   rz=0;
  // }
  // if(lz==1){
  //   ledcWrite(LEDC_CHANNEL_0,170);
  //   lz=0;
  // }
  RightForward();
  ledcWrite(LEDC_CHANNEL_0, 175); 
  LeftBack(); 
  ledcWrite(LEDC_CHANNEL_1, 175);  
}
void TurnLeft2(){
  //Push();
  // if(rz==1){
  //   ledcWrite(LEDC_CHANNEL_1,170);
  //   rz=0;
  // }
  RightForward();
  ledcWrite(LEDC_CHANNEL_0, 125); 
  lz=1;
  LeftBack(); 
  ledcWrite(LEDC_CHANNEL_1, 125);   
}

void TurnRight(){
  // if(rz==1){
  //   ledcWrite(LEDC_CHANNEL_1,170);
  //   rz=0;
  // }
  // if(lz==1){
  //   ledcWrite(LEDC_CHANNEL_0,170);
  //   lz=0;
  // }
  LeftForward();
  ledcWrite(LEDC_CHANNEL_0, 175);  
  RightBack();
  ledcWrite(LEDC_CHANNEL_1, 175);  
}

void TurnRight2(){
  // if(lz==1){
  //   ledcWrite(LEDC_CHANNEL_0,170);
  //   lz=0;
  // }
  LeftForward();
  ledcWrite(LEDC_CHANNEL_0, 125);  
  RightBack();
  ledcWrite(LEDC_CHANNEL_1, 125); 
  rz=1;  
}
void readSensors(){

  // read sensors
  sensors_values[0] = digitalRead(most_left_sensor);
  sensors_values[1] = digitalRead(left_sensor);
  sensors_values[2] = digitalRead(middle_sensor);
  sensors_values[3] = digitalRead(right_sensor);
  sensors_values[4] = digitalRead(most_right_sensor);
}

void LeftForward(){
  digitalWrite(left_motor_in2, HIGH);
  digitalWrite(left_motor_in1, LOW);
  }

void LeftBack(){
  digitalWrite(left_motor_in1, HIGH);
  digitalWrite(left_motor_in2, LOW);
  }

void RightForward(){
  digitalWrite(right_motor_in1, HIGH);
  digitalWrite(right_motor_in2, LOW);
  }

void RightBack(){
  digitalWrite(right_motor_in2, HIGH);
  digitalWrite(right_motor_in1, LOW);
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
  // Setting up pinmodes of Sensors
  Serial.begin(115200);
  // Setting up pinmodes of motors
  pinMode(left_motor_in1, OUTPUT);
  pinMode(left_motor_in2, OUTPUT);
  pinMode(right_motor_in1, OUTPUT);
  pinMode(right_motor_in2, OUTPUT);
  pinMode(left_motor_enable, OUTPUT);
  pinMode(right_motor_enable, OUTPUT);

    // setup sensors pinmodes as inputs
  pinMode(most_left_sensor, INPUT);
  pinMode(left_sensor, INPUT);
  pinMode(middle_sensor, INPUT);
  pinMode(right_sensor, INPUT);
  pinMode(most_right_sensor, INPUT);

digitalWrite(left_motor_in2, HIGH);
  digitalWrite(left_motor_in1, LOW);
  digitalWrite(right_motor_in1, HIGH);
  digitalWrite(right_motor_in2, LOW);

  /*digitalWrite(left_motor_in2, LOW);
  digitalWrite(left_motor_in1, HIGH);
  digitalWrite(right_motor_in1, LOW);
  digitalWrite(right_motor_in2, HIGH);*/

  // digitalWrite(right_motor_enable, HIGH);
  // digitalWrite(left_motor_enable, HIGH);
  ledcAttachPin(left_motor_enable, LEDC_CHANNEL_0); 
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  
  ledcAttachPin(right_motor_enable, LEDC_CHANNEL_1); 
  ledcSetup(LEDC_CHANNEL_1, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);

  i=0;
}

void loop(){
  if(i==0)
  {
    Push();
    delay(1);
    i=1;
  }
    //Forward();
  readSensors();
  //printSensors();
  // // TurnRight();
  // // delay(5000);
  // // TurnLeft();
  // // delay(5000);
  if(sensors_values[4]==1 /*&& sensors_values[3]==1*/)
  {
    counter=0;
    Serial.println("1");
    rr=1;
    TurnRight();
  }
  else if(sensors_values[4]==0 && sensors_values[3]==0 && sensors_values[0]==0 && sensors_values[1]==0 && sensors_values[2]==1)
  {
    counter=0;
    Serial.println("2");
    r=0;
    rr=0;
    l=0;
    ll=0;
    Forward();
  }

  else if( sensors_values[3]==1 && sensors_values[2]==0)
  {
    counter=0;
    Serial.println("4");
    TurnRight2();
  }
  else if(sensors_values[0]==1 /*&& sensors_values[1]==1*/)
  {
    counter=0;
    Serial.println("3");
    ll=1;
    TurnLeft();
  }
  else if(sensors_values[1]==1 && sensors_values[2]==0){
    counter=0;
    Serial.println("5");
    TurnLeft2();
  }
  // else if( sensors_values[4]==1 )
  // {
  //   Serial.println("6");
  //   TurnRight2();
  // }
  // else if(sensors_values[0]==1){
  //   Serial.println("7");
  //   TurnLeft2();
  // }
  // else if(sensors_values[4]==0 && sensors_values[3]==0 && sensors_values[0]==0 && sensors_values[1]==0 && sensors_values[2]==0)
  // {
  //   counter++;
  //   Forward();
  //   Serial.println("6");
  //   if(counter==500)
  //   {
  //     Stop();
  //     delay(5000);
  //   }
  // } 
}