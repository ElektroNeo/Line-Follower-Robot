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

#define Kp 0.15 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 1 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed           250 // max speed of the robot
#define leftMaxSpeed            250 // max speed of the robot
#define rightBaseSpeed          200 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed           200  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  8  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2
#define RIGHT_MOTOR_SPEED       10 // sağ motor PWM hız kontrol pini.    
#define LEFT_MOTOR_SPEED        9  // sol motor PWM hız kontrol pini.
#define RIGHT_MOTOR_FORWARD     7
#define RIGHT_MOTOR_BACKWARD    6
#define LEFT_MOTOR_FORWARD      5
#define LEFT_MOTOR_BACKWARD     4   
#define FORWARD                 1
#define BACKWARD                0
#define RIGHT_MOTOR             0
#define LEFT_MOTOR              1
#define trigPin                 12
#define echoPin                 11


void move(unsigned char selectMotor, unsigned char selectMovement, int selectMotorSpeed)
{
  if(selectMotor == RIGHT_MOTOR) {
    if(selectMovement == FORWARD) {
      digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
      digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
      analogWrite(RIGHT_MOTOR_SPEED, selectMotorSpeed);
      }
    if(selectMovement == BACKWARD) {
      digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
      digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
      analogWrite(RIGHT_MOTOR_SPEED, -1*selectMotorSpeed);
      }
  }
  else {
    if(selectMovement == FORWARD) {
      digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
      digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
      analogWrite(LEFT_MOTOR_SPEED, selectMotorSpeed);
      }
    if(selectMovement == BACKWARD) {
      digitalWrite(LEFT_MOTOR_FORWARD, LOW);
      digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
      analogWrite(LEFT_MOTOR_SPEED, -1*selectMotorSpeed);
      }
    }
  }
// sensors 0 through 7 are connected to analog inputs 7 through 0, respectively
QTRSensorsAnalog qtra((unsigned char[]) {7, 6, 5, 4, 3, 2, 1, 0}, 
                      NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
                      
unsigned int sensorValues[NUM_SENSORS], a, Output[NUM_SENSORS];


void setup()
{
  
  delay(500);
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);
  
  pinMode(RIGHT_MOTOR_SPEED,OUTPUT);
  pinMode(LEFT_MOTOR_SPEED,OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD,OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD,OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD,OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD,OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

int lastError = 0;
long sure, mesafe;
void loop()
{
  // read raw sensor values
  qtra.read(sensorValues);

  int position = qtra.readLine(sensorValues);
  int error = position - 3500;
    
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int leftMotorSpeed = rightBaseSpeed + motorSpeed;
  int rightMotorSpeed = leftBaseSpeed - motorSpeed;
  
  if (leftMotorSpeed > rightMaxSpeed ) 
      leftMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed > leftMaxSpeed ) 
      rightMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
      
  if (leftMotorSpeed < -1*leftMaxSpeed) 
      leftMotorSpeed = -1*leftMaxSpeed; // keep the motor speed positive
  if (rightMotorSpeed < -1*rightMaxSpeed) 
      rightMotorSpeed = -1*rightMaxSpeed; // keep the motor speed positive


  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  sure = pulseIn(echoPin, HIGH);
  mesafe = (sure/2) / 29.1;
  //Serial.println(mesafe);
  
     while (mesafe < 15){
      
      leftMotorSpeed = 0;
      rightMotorSpeed = 0;
      delay(100);
      move(RIGHT_MOTOR, FORWARD, rightMotorSpeed);
      move(LEFT_MOTOR, FORWARD, leftMotorSpeed);

      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      sure = pulseIn(echoPin, HIGH);
      mesafe = (sure/2) / 29.1;
      
     }
      /*
      Serial.print("  left: ");
      Serial.println(leftMotorSpeed);
      Serial.print("right: ");
      Serial.print(rightMotorSpeed);
      */
    
      /*
      for(unsigned char i = 0; i < 8; i ++){
         Serial.print(i+1);
         Serial.print(": ");
         Serial.print(sensorValues[i]);
         Serial.print("  ");
         }
         
         Serial.println();
      */
    
      //********** For moving motors **********
      
        //move(RIGHT_MOTOR, FORWARD, rightMotorSpeed);
        //move(LEFT_MOTOR, FORWARD, leftMotorSpeed);
    
        if(leftMotorSpeed < 0) {
          move(RIGHT_MOTOR, FORWARD, rightMotorSpeed);
          move(LEFT_MOTOR, BACKWARD, leftMotorSpeed);
        }
        else if(rightMotorSpeed < 0) {
          move(RIGHT_MOTOR, BACKWARD, rightMotorSpeed);
          move(LEFT_MOTOR, FORWARD, leftMotorSpeed);
        }
        else if(rightMotorSpeed > 0 && leftMotorSpeed > 0) {
          move(RIGHT_MOTOR, FORWARD, rightMotorSpeed);
          move(LEFT_MOTOR, FORWARD, leftMotorSpeed);
          }/*
        else if(leftMotorSpeed < 0) {
          move(RIGHT_MOTOR, FORWARD, rightMotorSpeed);
          move(LEFT_MOTOR, BACKWARD, 0);
        }
        else if(rightMotorSpeed < 0) {
          move(RIGHT_MOTOR, BACKWARD, 0);
          move(LEFT_MOTOR, FORWARD, leftMotorSpeed);
        }*/
        else {
          move(RIGHT_MOTOR, FORWARD, 100);
          move(LEFT_MOTOR, FORWARD, 100);
          }
        
      /*
      analogWrite(LEFT_MOTOR_SPEED, leftMotorSpeed);
      analogWrite(RIGHT_MOTOR_SPEED, rightMotorSpeed);
      digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
      digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
      digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
      digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
      */
      /*
      Serial.print("  left: ");
      Serial.print(leftMotorSpeed);
      Serial.print("    right: ");
      Serial.print(rightMotorSpeed);
      Serial.print("    mesafe: ");
      Serial.print(mesafe);
      Serial.println();
      */
  
}