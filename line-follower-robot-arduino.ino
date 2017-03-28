 #include <QTRSensors.h>

// This example is designed for use with six QTR-1A sensors or the first six sensors of a
// QTR-8A module.  These reflectance sensors should be connected to analog inputs 0 to 5.
// The QTR-8A's emitter control pin (LEDON) can optionally be connected to digital pin 2, 
// or you can leave it disconnected and change the EMITTER_PIN #define below from 2 to 
// QTR_NO_EMITTER_PIN.

// The main loop of the example reads the raw sensor values (uncalibrated).
// You can test this by taping a piece of 3/4" black electrical tape to a piece of white 
// paper and sliding the sensor across it.  It prints the sensor values to the serial 
// monitor as numbers from 0 (maximum reflectance) to 1023 (minimum reflectance).

#define Kp 0.25 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 1 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed           250 // max speed of the robot
#define leftMaxSpeed            250 // max speed of the robot
#define rightBaseSpeed          90 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed           90  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define TIMEOUT                 2500  // waits for 2500 us for sensor outputs to go low
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  16  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2
#define RIGHT_SPEED             10 // sağ motor PWM hız kontrol pini.    
#define LEFT_SPEED              9  // sol motor PWM hız kontrol pini.
#define RIGHT_MOTOR_FORWARD     7
#define RIGHT_MOTOR_BACKWARD    6
#define LEFT_MOTOR_FORWARD      5
#define LEFT_MOTOR_BACKWARD     4            

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]) {7, 6, 5, 4, 3, 2, 1, 0}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS], a, Output[NUM_SENSORS];


void setup()
{
  
  delay(500);
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);
  
  pinMode(RIGHT_SPEED,OUTPUT);
  pinMode(LEFT_SPEED,OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD,OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD,OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD,OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD,OUTPUT);
}

int lastError = 0;
void loop()
{
  // read raw sensor values
  qtra.read(sensorValues);

  int position = qtra.readLine(sensorValues);
  int error = position - 3500;

  

  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > rightMaxSpeed ) 
      rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) 
      leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
      
  if (rightMotorSpeed < 0) 
      rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) 
      leftMotorSpeed = 0; // keep the motor speed positive
  /*
  Serial.print("right: ");
  Serial.print(rightMotorSpeed);
  Serial.print("  left: ");
  Serial.println(leftMotorSpeed);
  delay(100);
  */
  analogWrite(RIGHT_SPEED, leftMotorSpeed);
  analogWrite(LEFT_SPEED, rightMotorSpeed);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  
  // print the sensor values as numbers from 0 to 1023, where 0 means maximum reflectance and
  // 1023 means minimum reflectance
  /*
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    if(sensorValues[i] > 200)
      a = 1;
    else
      a = 0;
    Serial.print(a);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();
  
  delay(250);
  */
  /*
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    if(sensorValues[i] > 200)
      Output[i] = 1;
    else
      Output[i] = 0;
  }
  */
  
}
