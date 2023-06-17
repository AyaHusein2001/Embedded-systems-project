// #include "Motors.h"
#include "utils.h"

//-------------------------------//
// kinda working cases          //
// kp = 0.05, kd = 0.5, no delay, mapping (400, 800, 10, 40), speed 100, d*0.75, sensors near eachother//
const int kp_tuner = 34;  //right
const int kd_tuner = 35;  //left

// PID parameters
float kp = 0.06;
float kd = 0.9;
// float ki = 0.001;
unsigned int last_proportional = 0;
int last_error = 0;

void setup() {
  Serial.begin(115200);
  pinMode(kp, INPUT);
  pinMode(kd, INPUT);

  setup_motors_pins();
  init_motors_speeds();
}

void loop() {
  // float kp_pot = analogRead(kp_tuner);
  // float kd_pot = analogRead(kd_tuner);
  // Serial.print("kp pot ");
  // Serial.println(kp_pot);

  // Serial.print("kd pot ");
  // Serial.println(kd_pot);

  // kp = kp_pot/5000;
  // kd = kd_pot/1000;

  // kp = map(kp_pot, 0, 4096, 0, 2000)/1000;
  // kd = map(kd_pot, 0, 4096, 0, 2000)/1000;

  // Serial.print("kp ");
  // Serial.println(kp);

  // Serial.print("kd ");
  // Serial.println(kd);

  int sensors_readings[_numSensors];
  int sensors_readings_analog[_numSensors];

  read_sensors(sensors_readings, sensors_readings_analog);
  // calibrate_readings(sensors_readings);

  // The returned position from 0 to 2000
  unsigned int position = read_line(sensors_readings);

  // The "proportional" term should be 0 when we are on the line.
  const int diff = 1000;  //target position
  int proportional = (int)position - diff;
  Serial.print("Proportional ");
  Serial.println(proportional);



  // Compute the derivative (change) and integral (sum) of the
  // position.
  int derivative = proportional - last_proportional;

  // Remember the last position.
  last_proportional = proportional;

  // Compute the difference between the two motor power settings,

  int power_difference = (proportional * kp + derivative * kd);


  Serial.print("power_difference ");
  Serial.println(power_difference);
  int maximum = MAX_SPEED;

  // To slow down for turns
  // if (abs(proportional) >= 400) {
  int d;
  // if (abs(proportional) == 250)
  d = map(abs(proportional), 250, 1000, 10, 25);  //10 -> 40 , 400 -> 800
  // else
  //   d = map(abs(proportional), 250, 1000, 10, 40);  //10 -> 40 , 400 -> 800

  maximum -= d * 0.9;
  Serial.print("Maximum ");
  Serial.println(maximum);
  Serial.print("d: ");
  Serial.println(d);
  // maximum = 80;
  // }
  //  else {
  //   maximum = 120;
  // }
  // power_difference = power_difference * ((float)maximum/MAX_SPEED);
  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)
    power_difference = -maximum;

  if (power_difference < 0)
    change_motors_speed(maximum, maximum + power_difference);
  else
    change_motors_speed(maximum - power_difference, maximum);

  print_readings(sensors_readings, proportional);
  // if(derivative != 0){
  //   delay(25);
  //   change_speed(0,0);

  //   while(true){}
  // }
  // delay(10);  // WITH SPEED 18, SENSORS CLOSE TO EACH OTHER, KP = 0.5, KD = 15
}