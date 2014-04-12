// Balancing robot
// Written by Ajay Bhaga
#include <MsTimer2.h>
#include <stdarg.h>

// Analog pins
//#define GYRO_X_AXIS 1
#define GYRO_Y_AXIS 1
#define GYRO_V_REF 2

#define ACC_ZERO_G 0
#define ACC_X 3
#define ACC_Y 4
#define ACC_Z 5

// Digital pins
#define PWMA 9
#define AIN2 2
#define AIN1 4
#define BIN1 7
#define BIN2 8
#define PWMB 5

#define V_REF 3.3 // Running at 3.3V
#define V_ZERO_G 1.65 // Running at 1.65V

// Measurement sensitivities
#define GYRO_SENSITIVITY 0.0005
#define ACC_SENSITIVITY 0.800

volatile int i = 0;
volatile int countS = 0;
volatile int recOmegaI[10];
volatile float omegaI = 0;
volatile long thetaI = 0;
volatile long sumPower = 0;
volatile long sumSumP = 0;
const int kAngle = 45;
const int kOmega = 85;
const long kSpeed = 57;
const long kDistance = 60;
volatile float powerScale;
volatile int power;
volatile long vE5 = 0;
volatile long xE5 = 0;

static unsigned long currTime = 0;

struct sVector
{
  float x;
  float y;
  float z;

  sVector& operator =(const sVector& a)
  {
    x = a.x;
    y = a.y;
    z = a.z;
    return *this;
  }

  float magnitude() {
    return sqrt((x*x)+(y*y)+(z*z));
  }
};


// Gyro zero rate
float gyroZeroRate = 0.0;
// Current gyro y-axis reading (rate of change on xz plane)
float RateAxz = 0.0;
// Previous gyro y-axis reading (rate of change on xz plane)
float prevRateAxz = 0.0;
// Current accelerometer reading
sVector Racc;
// Previous accelerometer reading
sVector prevRacc;
// Corrected accelerometer reading 
sVector Rest;
// Previous corrected accelerometer reading
sVector prevRest;
// Gyro vector reading
sVector Rgyro;

void p(char *fmt, ... ){
  char buf[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(buf, 128, fmt, args);
  va_end (args);
  Serial.print(buf);
}

void p(const __FlashStringHelper *fmt, ... ){
  char buf[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt);
#ifdef __AVR__
  vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args); // progmem for AVR
#else
  vsnprintf(buf, sizeof(buf), (const char *)fmt, args); // for the rest of the world
#endif
  va_end(args);
  Serial.print(buf);
}

void setup() {

  // Set baud rate
  Serial.begin(115200);

  // Set pin mode
  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Read metrics from instruments  
  readAccel();
  readGyro();

  // Store new value
  Rest = Racc;

  // Store first time value
  currTime = millis();

  // Setup interrupt
  MsTimer2::set(5, checkAndControl);
  MsTimer2::start();
}

void loop() {
  if (power > 0) {    
    // Move forward
    digitalWrite(AIN2, HIGH);
    digitalWrite(AIN1, LOW);
    analogWrite(PWMA, power);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, power);
  } 
  else {
    // Move backward
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, HIGH);
    analogWrite(PWMA, -power);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -power);
  }
}

void readGyro() {
  // Get gyro zero rate and convert signal to voltage
  gyroZeroRate = ((analogRead(GYRO_V_REF)*V_REF)/1023.0);

  // Store previous rate
  prevRateAxz = RateAxz;

  // Get gyro y-axis reading
  RateAxz = (((analogRead(GYRO_Y_AXIS)*V_REF)/1023.0)-gyroZeroRate)/GYRO_SENSITIVITY;

  if (abs(RateAxz) < 15) {
    RateAxz = 0.0;
  }

}

void readAccel() {
  // Get accelerometer readings
  Racc.x = (((analogRead(ACC_X)*V_REF)/1023.0)-V_ZERO_G)/ACC_SENSITIVITY;
  Racc.y = (((analogRead(ACC_Y)*V_REF)/1023.0)-V_ZERO_G)/ACC_SENSITIVITY;
  Racc.z = (((analogRead(ACC_Z)*V_REF)/1023.0)-V_ZERO_G)/ACC_SENSITIVITY;
}

void checkAndControl() {

  // Get delta time  
  float delta = (millis() - currTime);
  // Store new time 
  currTime = millis();

  // Store previous corrected value
  prevRest = Rest;

  // Read metrics from instruments  
  readAccel();
  readGyro();

  // Get magnitude of vector
  float magRAcc = Racc.magnitude();

  // Normalize values
  Racc.x /= magRAcc;
  Racc.y /= magRAcc;
  Racc.z /= magRAcc;

  // Determine gyro vector
  float prevAxz = atan2(prevRest.x, prevRest.z);
  float RateAxzAvg = (RateAxz + prevRateAxz)/2.0;
  float Axz = prevAxz + (RateAxzAvg * delta);

  Rgyro.x = sin(Axz);
  Rgyro.y = 0.0;
  Rgyro.z = cos(Axz);

  float wGyro = 10.0;
  Rest.x = (Racc.x + Rgyro.x * wGyro) / (1 + wGyro);
  Rest.y = (Racc.y + Rgyro.y * wGyro) / (1 + wGyro);
  Rest.z = (Racc.z + Rgyro.z * wGyro) / (1 + wGyro);  

  float magRest = Rest.magnitude();
  Rest.x /= magRest;
  Rest.y /= magRest;
  Rest.z /= magRest;

  // Calculate gyro rate
  omegaI = RateAxz;

  if (abs(omegaI) < 10.0) {
    omegaI = 0.0;
  }

  // Store omega for this iteration
  recOmegaI[0] = omegaI;

  // Integrate to position
  thetaI = thetaI + round((omegaI/10.0));

  countS = 0;

  // Check for little movement
  for (i = 0; i < 10; i++) {
    if (abs(recOmegaI[i]) < 15.0) {
      countS++;
    }
  }

  // If there has been no movement, reset
  if (countS > 5) {
    thetaI = 0;
    vE5 = 0;
    xE5 = 0;
    sumPower = 0;
    sumSumP = 0;
  }

  // Right shift historical omega's
  for (i = 9; i > 0; i--) {
    recOmegaI[i] = recOmegaI[i-1];
  }

  powerScale = (kAngle * thetaI / 200) + (omegaI / 100);// + (kSpeed * vE5 / 1000) + (kDistance * xE5 / 1000);

  power = 0;// max(min(powerScale, 100), -100);
  sumPower = sumPower + power;
  sumSumP = sumSumP + sumPower;

  vE5 = sumPower;
  xE5 = sumSumP / 1000;


  static unsigned long currTime2 = 0;
  if (abs (millis() - currTime2) > 150)
  {
    currTime2 = millis();     
    p("delta=[%.4f], prevAxz=[%.4f], Axz=[%.4f], RateAxz=[%.4f], RateAxzAvg=[%.4f]\n", delta, prevAxz, Axz, RateAxz, RateAxzAvg);
  }
}





