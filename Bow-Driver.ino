#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
int pinA1 = 26; // Connected to CLK
int pinB1 = 25; // Connected to DT
int encoderPosCount1 = 0;
int pinA1Last;
int interval = 100;
float rps;
boolean bCW;
unsigned long previousMillis = 0;

void setup() {
  while (!Serial)
    delay(10);
  pinMode (pinA1,INPUT);
  pinMode (pinB1,INPUT);
  /* Read Pin A
  Whatever state it's in will reflect the last position
  */
  pinA1Last = digitalRead(pinA1);
  Serial.begin (9600);
  previousMillis = millis();
  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}


int poscount(int aVal, int pinB, int pinALast){
  bool eqlast = pinALast == aVal;
  pinALast = aVal;
  if (!eqlast){  
    if (digitalRead(pinB) != aVal) { // Means pqin A Changed first - We're Rotating
      return 1;
    } 
    else {// Otherwise B changed first and we're moving CCW
      return -1;
    }
  }
  return 0;
}
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long currentMillis = millis();
  int aVal1 = digitalRead(pinA1);
  encoderPosCount1 += poscount(aVal1, pinB1, pinA1Last);
  pinA1Last = aVal1;
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  if(currentMillis - previousMillis >= interval){

    rps = abs(3.14*2.0 * (((float)encoderPosCount1)/16.0) * 1000/interval);
    if(rps > 0) {
      Serial.print("Encoder position:");
      Serial.println(encoderPosCount1);
      Serial.print("Rotation speed (radians per second): ");
      Serial.println(rps);

    }
    
    encoderPosCount1 = 0;
    previousMillis = currentMillis;
  }

}
