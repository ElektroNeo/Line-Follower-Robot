#include <QTRSensors.h>

//*********** Motorlar, motorların hızları ve PID algoritması için ***********

#define                         Kp 0.15 // Kp < Kd olmalı.
#define                         Kd 1 
#define maxSpeed                250     // motorların maksimum hızı
#define baseSpeed               200     // tam çizgide olduğu zaman motorların hızı
#define NUM_SENSORS             8       // sensörlerin sayısı
#define NUM_SAMPLES_PER_SENSOR  8       // her sensör okumasında 8 değer okunur ve bunların ortalaması alınır
                                        // her bir okuma 100us sürüyor; buna göre toplam 8(sensor)*8*100us = 6.4ms sürüyor.
#define EMITTER_PIN             2       // emitter 2 nolu dijital pin ile kontrol ediliyor
#define RIGHT_MOTOR_SPEED       10      // sağ motor PWM hız kontrol pini.    
#define LEFT_MOTOR_SPEED        9       // sol motor PWM hız kontrol pini.
#define RIGHT_MOTOR_POSITIVE    7       // sağ motorun artı ucu
#define RIGHT_MOTOR_NEGATIVE    6       // sağ motorun eksi ucu
#define LEFT_MOTOR_POSITIVE     5       // sol motorun artı ucu
#define LEFT_MOTOR_NEGATIVE     4       // sol motorun eksi ucu  

//************** move() fonksiyonu için **************

#define FORWARD                 1
#define BACKWARD                0
#define RIGHT_MOTOR             0
#define LEFT_MOTOR              1

//************** move() fonksiyonu için **************

#define FORWARD                 1
#define BACKWARD                0
#define RIGHT_MOTOR             0
#define LEFT_MOTOR              1


//**************** Mesafe sensörü için ****************

#define trigPin                 12
#define echoPin                 11

void move(unsigned char selectMotor, unsigned char selectMovement, int selectMotorSpeed);
void stop();
  
// 8 sensör sırasıyla 7-0 analog girişlerine bağlanmıştır.
QTRSensorsAnalog qtra((unsigned char[]) {7, 6, 5, 4, 3, 2, 1, 0}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
                      
// Sensörden gelen analog değerleri tutmak için sensorValues[NUM_SENSORS] dizisi tanımlanmıştır.                      
unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  Serial.begin(9600);
  delay(500); // Robot çalışmaya 2 saniye sonra başlar.

  // Motorların ve HC SR04 sensörünün pin durumunun ayarlanması.
  pinMode(RIGHT_MOTOR_SPEED,OUTPUT);
  pinMode(LEFT_MOTOR_SPEED,OUTPUT);
  pinMode(RIGHT_MOTOR_POSITIVE,OUTPUT);
  pinMode(RIGHT_MOTOR_NEGATIVE,OUTPUT);
  pinMode(LEFT_MOTOR_POSITIVE,OUTPUT);
  pinMode(LEFT_MOTOR_NEGATIVE,OUTPUT);
  // Mesafe sensörününün pin durumunun ayarlanması.
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

//PID algoritması için son hata değeri tanımlanıyor.
int lastError = 0;
long sure, mesafe, counter = 0;
int leftMotorSpeed, rightMotorSpeed;

void loop()
{
  /*
   * Normalde her döngüde uzaklık sensörünü kontrol ediyorduk, fakat bu durum QTR-8A sensörünün
   * daha geç kontrol edilmesini sağlıyordu ve robotun yoldan çıkmasına sebep oluyordu. Bu durumu 
   * engellemek için counter adında bir değişken oluşturuldu. Bu değişken sayesinde robot her 
   * döngüde değilge 50 döngüde bir uzaklık sensörünü kontrol edecek. Böylece robot daha az
   * yoldan çıkacak.
  */
  if (counter == 50) {
    // Bu döngüye girildiğinde counter sıfırlanır.
    counter = 0;

    // Engele olan uzaklık ölçülür.
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    sure = pulseIn(echoPin, HIGH);
    mesafe = (sure/2) / 29.1;
    //Serial.println(mesafe);

    // Sağ ve sol mottorların hızları 150 yapılır.
    rightMotorSpeed = 150;
    leftMotorSpeed = 150;

    // Eğer engele olan uzaklık 30cm 'den küçükse bu döngüye girilir.
       while (mesafe <= 30){
        // Motorlar ters yönde 150 hızı ile 200 milisaniye çalıştırılır.
        move(RIGHT_MOTOR, BACKWARD, leftMotorSpeed);
        move(LEFT_MOTOR, BACKWARD, rightMotorSpeed);
        delay(200);
        // Engele olan uzaklık ölçülür.
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        sure = pulseIn(echoPin, HIGH);
        mesafe = (sure/2) / 29.1;
        // Daha önce sağ ve sol motorların hızını 150 yapmıştık. Bu döngüde
        // iken robot geri gitmesin diye motorların hızlarını sıfırlıyoruz.
        rightMotorSpeed = 0;
        leftMotorSpeed = 0;
        
       }
  }
  
  // Her döngüde counter değişkenini 1 arttırıyoruz.
  counter++;
  
  // Tüm sensörleri okur ve okunan değerleri sensorValue[] dizisine atar.
  qtra.read(sensorValues);

  /*  
   *  Sensör değerlerini okur ve buna göre bir değer geri dönderir.
   *  Bu değer 0 ile 7000 arasında bir değerdir. 
   *  3500 değeri geri dönülürse robot tam çizgi üzerinde anlamına gelir.
   */
  int position = qtra.readLine(sensorValues);
  
  // Çizginin bulunduğu konuma göre bir hata değeri üretilir.
  // Eğer robot tam çizginin üzerindeyse değer 3500 olur ve hata 0 olur.
  int error = position - 3500;

  // PID algoritmasına göre motorun hızı belirlenir.
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  // Sağ ve sol motorun hızları algoritmaya göre düzenlenir.
  leftMotorSpeed = baseSpeed + motorSpeed;
  rightMotorSpeed = baseSpeed - motorSpeed;

  // Buradaki if ifadeleri motor hızlarının -250 ile +250 arasında kalmasını sağlıyor.
  if (leftMotorSpeed > maxSpeed ) 
      leftMotorSpeed = maxSpeed;
  if (rightMotorSpeed > maxSpeed ) 
      rightMotorSpeed = maxSpeed;
      
  if (leftMotorSpeed < -1*maxSpeed) 
      leftMotorSpeed = -1*maxSpeed;
  if (rightMotorSpeed < -1*maxSpeed) 
      rightMotorSpeed = -1*maxSpeed;
    
  //********** Motorların hareketi için **********
      
  if(leftMotorSpeed < -100) {
    move(RIGHT_MOTOR, FORWARD, rightMotorSpeed);
    move(LEFT_MOTOR, BACKWARD, leftMotorSpeed);
  }
  else if(rightMotorSpeed < -100) {
    move(RIGHT_MOTOR, BACKWARD, rightMotorSpeed);
    move(LEFT_MOTOR, FORWARD, leftMotorSpeed);
  }
  else if(rightMotorSpeed > 0 && leftMotorSpeed > 0) {
    move(RIGHT_MOTOR, FORWARD, rightMotorSpeed);
    move(LEFT_MOTOR, FORWARD, leftMotorSpeed);
    }
  else if(leftMotorSpeed < 0) {
    move(RIGHT_MOTOR, FORWARD, rightMotorSpeed);
    move(LEFT_MOTOR, BACKWARD, 0);
  }
  else if(rightMotorSpeed < 0) {
    move(RIGHT_MOTOR, BACKWARD, 0);
    move(LEFT_MOTOR, FORWARD, leftMotorSpeed);
  }
  else {
    move(RIGHT_MOTOR, FORWARD, 100);
    move(LEFT_MOTOR, FORWARD, 100);
    }
  
}

//*****************************************************************************************************************************************************************************//

/*
  move(Motor Seçimi, Yön Seçimi, Hız Değeri)
  
  Motor Seçimi: RIGHT_MOTOR = Sağ Motor
                LEFT_MOTOR  = Sol Motor
  Y�n Seçimi:   FORWARD     = İleri
                BACKWARD    = Geri
  H�z Değeri:  0 ile 255 arasında bir değer alır.
  
  Not: Hız değeri negatif olamaz. 
*/

void move(unsigned char selectMotor, unsigned char selectMovement, int selectMotorSpeed)
{
  if(selectMotor == RIGHT_MOTOR) {
    if(selectMovement == FORWARD) {
      digitalWrite(RIGHT_MOTOR_POSITIVE, HIGH);
      digitalWrite(RIGHT_MOTOR_NEGATIVE, LOW);
      analogWrite(RIGHT_MOTOR_SPEED, selectMotorSpeed);
      }
    if(selectMovement == BACKWARD) {
      digitalWrite(RIGHT_MOTOR_POSITIVE, LOW);
      digitalWrite(RIGHT_MOTOR_NEGATIVE, HIGH);
      analogWrite(RIGHT_MOTOR_SPEED, -1*selectMotorSpeed);
      }
  }
  else {
    if(selectMovement == FORWARD) {
      digitalWrite(LEFT_MOTOR_POSITIVE, HIGH);
      digitalWrite(LEFT_MOTOR_NEGATIVE, LOW);
      analogWrite(LEFT_MOTOR_SPEED, selectMotorSpeed);
      }
    if(selectMovement == BACKWARD) {
      digitalWrite(LEFT_MOTOR_POSITIVE, LOW);
      digitalWrite(LEFT_MOTOR_NEGATIVE, HIGH);
      analogWrite(LEFT_MOTOR_SPEED, -1*selectMotorSpeed);
      }
    }
  }
  
//*****************************************************************************************************************************************************************************//

  // stop() fonksiyonu robotu durdurmak için kullanılır.
  
  void stop() {
    move(RIGHT_MOTOR, FORWARD, 0);
    move(LEFT_MOTOR, FORWARD, 0);
  }