#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
int pinA1 = 26; // Connected to CLK
int pinB1 = 25; // Connected to DT
int encoderPosCount1 = 0;
int pinA1Last;
int interval = 100;

float xoffset; 
float yoffset;
float zoffset;

float gxoffset;
float gyoffset;
float gzoffset;

float xstandarddiff;
float ystandarddiff;
float zstandarddiff;

float gxstandarddiff;
float gystandarddiff;
float gzstandarddiff;

float xvel;
float yvel;
float zvel;

float xpos;
float ypos;
float zpos;

float mintotacc;

float rps;
boolean bCW;
unsigned long previousMillis = 0;
unsigned long lastUpdMillis = 0;

void calibrate(Adafruit_MPU6050 m, unsigned long duration){
  int totaldata = 0;
  unsigned long startttime = millis();
  Serial.println("Calibrating...");

  float xmin = 9999999;
  float xmax = -999999;
  float ymin = 9999999;
  xpos = 0;
  ypos = 0;
  zpos = 0;
  xvel = 0;
  yvel = 0;
  zvel = 0;
  float xoffsettotal = 0; 
  float yoffsettotal = 0;
  float zoffsettotal = 0;

  float gxoffsettotal = 0;
  float gyoffsettotal = 0;
  float gzoffsettotal = 0;

  while(millis() - startttime < duration) {
    sensors_event_t a, g, temp;
    m.getEvent(&a, &g, &temp);
    xoffsettotal += a.acceleration.x; 
    yoffsettotal += a.acceleration.y;
    zoffsettotal += a.acceleration.z;

    gxoffsettotal += g.gyro.x;
    gyoffsettotal += g.gyro.y;
    gzoffsettotal += g.gyro.z;

    totaldata++;
  }
  xoffset = xoffsettotal/totaldata; 
  yoffset = yoffsettotal/totaldata;
  zoffset = zoffsettotal/totaldata;

  gxoffset = gxoffsettotal/totaldata;
  gyoffset = gyoffsettotal/totaldata;
  gzoffset = gzoffsettotal/totaldata;

  Serial.print("Collected a total of: ");
  Serial.print(totaldata);
  Serial.println(" datapoints");

  Serial.print("Setting offset datapoints to X: ");
  Serial.print(xoffset);
  Serial.print(", Y: ");
  Serial.print(yoffset);
  Serial.print(", Z: ");
  Serial.print(zoffset);
  Serial.println(" m/s^2");
  Serial.print(gxoffset);
  Serial.print(", Y: ");
  Serial.print(gyoffset);
  Serial.print(", Z: ");
  Serial.print(gzoffset);
  Serial.println("rad/s");

}

void setup() {
  pinMode (pinA1,INPUT);
  pinMode (pinB1,INPUT);
  /* Read Pin A
  Whatever state it's in will reflect the last position
  */
  pinA1Last = digitalRead(pinA1);
  Serial.begin (115200);
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
  calibrate(mpu, 20000);
  lastUpdMillis = millis();
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
  unsigned long currentMillis = millis();
  int aVal1 = digitalRead(pinA1);
  encoderPosCount1 += poscount(aVal1, pinB1, pinA1Last);
  pinA1Last = aVal1;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x - xoffset);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y - yoffset);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z - zoffset);
  Serial.println(" m/s^2");

  float delta = ((float)(currentMillis - lastUpdMillis))/1000.0;

  float xacc = a.acceleration.x - xoffset;
  float yacc = a.acceleration.y - yoffset;
  float zacc = a.acceleration.z - zoffset;

  xpos += xvel *  delta + (xacc*delta*delta/2);
  ypos += yvel *  delta + (yacc*delta*delta/2);
  zpos += zvel *  delta + (zacc*delta*delta/2);

  xvel += xacc * delta; 
  yvel += yacc * delta;
  zvel += zacc * delta;

  
  Serial.print("delta: ");
  Serial.println(delta);
  
  Serial.print("Velocity X: ");
  Serial.print(xvel);
  Serial.print(", Y: ");
  Serial.print(yvel);
  Serial.print(", Z: ");
  Serial.print(zvel);
  Serial.println(" m/s");

  Serial.print("Position X: ");
  Serial.print(xpos);
  Serial.print(", Y: ");
  Serial.print(ypos);
  Serial.print(", Z: ");
  Serial.print(zpos);
  Serial.println(" m");

  lastUpdMillis = currentMillis;

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
