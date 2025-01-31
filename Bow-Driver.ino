#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//#define TEST_OUTPUT 1
Adafruit_MPU6050 mpu;
int pinA1 = 26; // Connected to CLK
int pinB1 = 25; // Connected to DT

int pinA2 = 14;
int pinB2 = 27;
int encoderPosCount1 = 0;
int encoderPosCount2 = 0;
int pinA1Last; 
int interval = 32;
int pinA2Last;

int button1 = 16;
int button2 = 17;
int lastbutton1 = 0;
int lastbutton2 = 0;


int heldframes1 = 0;
int heldframes2 = 0;
bool button1pressed = false;
bool button2pressed = false;

bool button1unpressed = true;
bool button2unpressed = true;
bool rps_change = false; 

class datapoint{
  public:
    float min;
  public:
    float max;
  public:
    float offset;
  float standarddiff;
  public:   
    float pos;
  public: 
    void reset(){
      min = 9999999;
      max = -9999999;
      offset = 0;
      standarddiff = 0;
      pos = 0;
    }
  public: 
    void setstandarddiff(){
      standarddiff = abs(abs(max)-abs(min));
    }
  public: void calcpos(float vel, float delta){
    if(abs(vel) > standarddiff){
      pos += vel *  delta;
    }
  }
};

class coord: public datapoint{
  public:
    float vel;
  int resetcount;
  public: void reset(){
    datapoint::reset();
    vel = 0;
    resetcount = 0;
  }
  public: void calcpos(float acc, float delta){
    if(abs(acc) > standarddiff){
      pos += vel *  delta + (acc*delta*delta/2);
      vel += acc * delta; 
    }
    else{
      resetcount++;
      if(resetcount == 100){
        vel = 0;
        resetcount = 0;
      }
    }
  }
};


coord x;
coord y;
coord z;

datapoint gx;
datapoint gy; 
datapoint gz;



int minxtime;
int maxxtime;

float rps;
boolean bCW;
unsigned long previousMillis = 0;
unsigned long lastUpdMillis = 0;

unsigned long lastAccMillis = 0;

void calibrate(Adafruit_MPU6050 m, unsigned long duration){
  int totaldata = 0;
  unsigned long startttime = millis();
  #ifdef TEST_OUTPUT
  Serial.println("Calibrating...");
  #endif

  int startup_wait = 100;

  float xoffsettotal = 0;
  float yoffsettotal = 0;
  float zoffsettotal = 0;
  float gxoffsettotal = 0;
  float gyoffsettotal = 0;
  float gzoffsettotal = 0;
  x.reset();
  y.reset();
  z.reset();
  gx.reset();
  gy.reset();
  gz.reset();

  while(millis() - startttime < duration) {
    sensors_event_t a, g, temp;
    m.getEvent(&a, &g, &temp);
    
  
    //God forgive me
    if(totaldata > startup_wait){
      xoffsettotal += a.acceleration.x; 
      yoffsettotal += a.acceleration.y;
      zoffsettotal += a.acceleration.z;
      if(a.acceleration.x > x.max) {maxxtime = totaldata; x.max = a.acceleration.x;}
      if(a.acceleration.x < x.min) {minxtime = totaldata; x.min = a.acceleration.x;}
      if(a.acceleration.y > y.max) {y.max = a.acceleration.y;}
      if(a.acceleration.y < y.min) {y.min = a.acceleration.y;}
      if(a.acceleration.z > z.max) {z.max = a.acceleration.z;}
      if(a.acceleration.z < z.min) {z.min = a.acceleration.z;}
      if(g.gyro.x > gx.max) {gx.max = g.gyro.x;}
      if(g.gyro.x < gx.min) {gx.min = g.gyro.x;}
      if(g.gyro.y > gy.max) {gy.max = g.gyro.y;}
      if(g.gyro.y < gy.min) {gy.min = g.gyro.y;}
      if(g.gyro.z > gz.max) {gz.max = g.gyro.z;}
      if(g.gyro.z < gz.min) {gz.min = g.gyro.z;}
      gxoffsettotal += g.gyro.x;
      gyoffsettotal += g.gyro.y;
      gzoffsettotal += g.gyro.z;

    }

    totaldata++;
  }

  totaldata -= startup_wait;
  x.offset = xoffsettotal/totaldata; 
  y.offset = yoffsettotal/totaldata;
  z.offset = zoffsettotal/totaldata;

  gx.offset = gxoffsettotal/totaldata;
  gy.offset = gyoffsettotal/totaldata;
  gz.offset = gzoffsettotal/totaldata;

  x.setstandarddiff();
  y.setstandarddiff();
  z.setstandarddiff();
  gx.setstandarddiff();
  gy.setstandarddiff();
  gz.setstandarddiff();

  #ifdef TEST_OUTPUT
  Serial.print("Collected a total of: ");
  Serial.print(totaldata);
  Serial.println(" datapoints");

  Serial.print("The smallest value for X was found at datapoint: ");
  Serial.print(minxtime);
  Serial.print(" with a value of: ");
  Serial.println(x.min);
  Serial.print("The greatest value for X was found at datapoint: ");
  Serial.print(maxxtime);
  Serial.print(" with a value of: ");
  Serial.println(x.max);

  Serial.print("Setting offset to X: ");
  Serial.print(x.offset);
  Serial.print(", Y: ");
  Serial.print(y.offset);
  Serial.print(", Z: ");
  Serial.print(z.offset);
  Serial.println(" m/s^2");
  Serial.print(gx.offset);
  Serial.print(", Y: ");
  Serial.print(gy.offset);
  Serial.print(", Z: ");
  Serial.print(gz.offset);
  Serial.println("deg/s");

  Serial.print("Setting standard deviation to X: ");
  Serial.print(x.standarddiff);
  Serial.print(", Y: ");
  Serial.print(y.standarddiff);
  Serial.print(", Z: ");
  Serial.print(z.standarddiff);
  Serial.println(" m/s^2");
  Serial.print(gx.standarddiff);
  Serial.print(", Y: ");
  Serial.print(gy.standarddiff);
  Serial.print(", Z: ");
  Serial.print(gz.standarddiff);
  Serial.println("deg/s");
  #endif
  while (Serial.available() > 0){char received = Serial.read();}

}

void setup() {
  pinMode (pinA1,INPUT);
  pinMode (pinB1,INPUT);
  pinMode (pinB2,INPUT);
  pinMode (pinA2,INPUT);
  pinMode (button1,INPUT);
  pinMode (button2,INPUT);
  /* Read Pin A
  Whatever state it's in will reflect the last position
  */
  pinA1Last = digitalRead(pinA1);
  pinA2Last = digitalRead(pinA2);
  Serial.begin (115200);
  previousMillis = millis();
  if (!mpu.begin()) {
    #ifdef TEST_OUTPUT
    Serial.println("Sensor init failed");
    #endif
    while (1)
      yield();
  }
  #ifdef TEST_OUTPUT
  Serial.println("MPU6050 Found!");
  #endif

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  #ifdef TEST_OUTPUT
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
  #endif

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  #ifdef TEST_OUTPUT
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
  #endif

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  #ifdef TEST_OUTPUT
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
  #endif
  delay(1000);
  calibrate(mpu, 2000);
  lastUpdMillis = millis();
}


int poscount(int aVal, int pinB, int pinALast){
  bool eqlast = pinALast == aVal;
  pinALast = aVal;
  if (!eqlast){  
    //Serial.println("rotating!");
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
  int aVal2 = digitalRead(pinA2);

  int currentbutton1 = digitalRead(button1);
  int currentbutton2 = digitalRead(button2);
  if(currentbutton1 == HIGH){
    heldframes1 += 1;
    if(heldframes1 >= 100 && button1unpressed){
      #ifdef TEST_OUTPUT
      Serial.println("Button 1 pressed");
      #endif
      button1pressed = true;
      button1unpressed = false;
      heldframes1 = 0;
    }
  }
  else {
    heldframes1 = 0;
    button1unpressed = true;
  }
  if(currentbutton2 == HIGH){
    heldframes2 += 1;
    if(heldframes2 >= 100 && button1unpressed){
      #ifdef TEST_OUTPUT
      Serial.println("Button 2 pressed");
      #endif
      button2unpressed = false;
      button2pressed = true;
      heldframes2 = 0;
      calibrate(mpu, 2000);
    }
  }
  else{
    button2unpressed = true;
    heldframes2 = 0;
  }

  lastbutton1 = currentbutton1;
  lastbutton2 = currentbutton2;
  encoderPosCount1 += poscount(aVal1, pinB1, pinA1Last);
  encoderPosCount2 += poscount(aVal2, pinB2, pinA2Last);
  pinA1Last = aVal1;
  pinA2Last = aVal2;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  #ifdef TEST_OUTPUT
  #endif

  float delta = ((float)(currentMillis - lastUpdMillis))/1000.0;

  float xacc = a.acceleration.x - x.offset;
  float yacc = a.acceleration.y - y.offset;
  float zacc = a.acceleration.z - z.offset;

  x.calcpos(xacc, delta);
  y.calcpos(yacc, delta);
  z.calcpos(zacc, delta);

  float gxvel = g.gyro.x - gx.offset;
  float gyvel = g.gyro.y - gy.offset;
  float gzvel = g.gyro.z - gz.offset;
  gx.calcpos(xacc, delta);
  gy.calcpos(yacc, delta);
  gz.calcpos(zacc, delta);

  #ifndef TEST_OUTPUT
  //if(Serial.availableForWrite() >= 13){
  //  byte buff[1] = {1};
  //  Serial.write(buff, 1);
  //  byte gxbuff[16] = {0};
  //  memcpy(gxbuff, (byte*)&gxvel, sizeof(float));
  //  Serial.write(gxbuff, sizeof(float));
  //  byte gybuff[16] = {0};
  //  memcpy(gybuff, (byte*)&gyvel, sizeof(float));
  //  Serial.write(gybuff, sizeof(float));
  //  byte gzbuff[16] = {0};
  //  memcpy(gzbuff, (byte*)&gzvel, sizeof(float));
  //  Serial.write(gzbuff, sizeof(float));
  //}

  #endif

  #ifndef TEST_OUTPUT
  if(Serial.available() > 0){
    byte incomingByte = Serial.read();

    if(incomingByte == 1){
      byte buff[16] = {1};
      memcpy(buff+1, (byte*)&gxvel, sizeof(float));
      memcpy(buff+1+sizeof(float), (byte*)&gyvel, sizeof(float));
      memcpy(buff+1+2*sizeof(float), (byte*)&gzvel, sizeof(float));
      Serial.write(buff, 13);
    }
    if(incomingByte == 2){
      byte rpsbuff[32] = {0};
      memcpy(rpsbuff+1, (byte*)&rps,sizeof(float));
      rpsbuff[0] = (byte)2;
      Serial.write(rpsbuff, sizeof(float)+1);
      rps = 0;
    }
    //if(incomingByte == 3){
    //  byte button1buff[1] = {0};
    //  if(button1pressed){
    //    button1buff[0] = 3;
    //  }
    //  button1pressed = false;
    //  Serial.write(button1buff, 1);
    //}
    //if(incomingByte == 4){
    //  byte button2buff[1] = {0};
    //  if(button2pressed){
    //    button2buff[0] = 4;
    //  }
    //  button2pressed = false;
    //  Serial.write(button2buff, 1);
    //}
    //if(incomingByte == 5){
    //  byte button2buff[1] = {5};
    //  Serial.write(button2buff, 1);
    //  //calibrate(mpu, 5000);
    //}
  }
  #endif
  #ifdef TEST_OUTPUT
  //Serial.print("Acceleration X: ");
  //Serial.print(a.acceleration.x - x.offset);
  //Serial.print(", Y: ");
  //Serial.print(a.acceleration.y - y.offset);
  //Serial.print(", Z: ");
  //Serial.print(a.acceleration.z - z.offset);
  //Serial.println(" m/s^2");
  //Serial.print("delta: ");
  //Serial.println(delta);
  //
  //Serial.print("Velocity X: ");
  //Serial.print(x.vel);
  //Serial.print(", Y: ");
  //Serial.print(y.vel);
  //Serial.print(", Z: ");
  //Serial.print(z.vel);
  //Serial.println(" m/s");

  //Serial.print("Position X: ");
  //Serial.print(x.pos);
  //Serial.print(", Y: ");
  //Serial.print(y.pos);
  //Serial.print(", Z: ");
  //Serial.print(z.pos);
  //Serial.println(" m");

  //Serial.println("Angle X: ");
  //Serial.print(gx.pos);
  //Serial.print(", Y: ");
  //Serial.print(gy.pos);
  //Serial.print(", Z: ");
  //Serial.print(gz.pos);
  //Serial.println(" deg");
  #endif

  lastUpdMillis = currentMillis;
  if(currentMillis - previousMillis >= interval){

    rps = 3.14*2.0 * (((float)(encoderPosCount2)+(float)(encoderPosCount1))/32.0) * 1000/interval;
    rps_change = false;
    if(rps > 0) {
      #ifdef TEST_OUTPUT
      //Serial.print("Encoder position:");
      //Serial.println(encoderPosCount2);
      //Serial.print("Rotation speed (radians per second): ");
      //Serial.println(rps);
      #endif

      #ifndef TEST_OUTPUT
      //if(Serial.availableForWrite() >= 5){
      //  byte rpsbuff[32] = {0};
      //  rpsbuff[0] = (byte)2;
      //  memcpy(rpsbuff+1, (byte*)&rps,sizeof(float));
      //  Serial.write(rpsbuff, sizeof(float)+1);
      //}
      #endif


    }
    
    encoderPosCount2 = 0;
    encoderPosCount1 = 0;
    previousMillis = currentMillis;
  }

}
