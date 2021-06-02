// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;
Servo Lservo;
Servo Rservo;

// angle read variales
float VerticalCalZ = 0.90;
float FortyFiveCalZ = -6.2;
float theta;
float theta_dot;
float accZ;
float theta_0 = 5.0; // initial theta reading
float theta_dot_0 = 0.0; // initial theta reading
int CalInt = 10000; // integer for calibration

// control variables
float theta_stable = 3.0; // -7.0; // theta where the robot is balanced
float P = 20.0; // 20.0; // control gain (proportional)
float D = 1.0; //  5.0; // control gain (derivative)

float theta_history[50];
float theta_dot_history[50];
int hist = 30; // number of samples to keep for the mean
int up = 0; // index to place new recording

// servo write variables
int LservoPin = 3;       // Pin that the left servomotor is connected to
int RservoPin = 6;       // Pin that the right servomotor is connected to
float phi;          // angle to send to servo

// time variables
float deltaT;
float oldTime = 0.0; // old time reading

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
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

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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

  // initialize servos
  Lservo.attach(LservoPin);
  Rservo.attach(RservoPin);

  Serial.println("");
  delay(100);

  // ENTERING CALIBRATION
  Serial.println("Calibrating ...");
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // initialize sensor
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  for (int j = 0; j < CalInt; j = j + 1) {
    accZ = a.acceleration.z;
    
    theta_0 += float_map(accZ, FortyFiveCalZ, VerticalCalZ, 45.0, 0.0);
    theta_dot_0 += g.gyro.x;
  }

  theta_0 = theta_0/float(CalInt);
  theta = theta_0;

  theta_dot_0 = theta_dot_0/float(CalInt);
  
  digitalWrite(13, LOW); // ending calibration
  Serial.println("Done calibrating !");
  Serial.println("Waiting ...");
  delay(3000);
}

void loop() {
  // calculate delta t
  deltaT = CalculateDeltaTime()/1000.0;

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // find current theta_dot
  theta_dot = 180*(g.gyro.x - theta_dot_0)/3.14159;

  // update theta dot history
  theta_dot_history[up] = theta_dot;

  // find moving average
  theta_dot = AverageArray(theta_dot_history);

  // find current accZ
  accZ = a.acceleration.z;

  // find theta
  theta += theta_dot*deltaT; 
  
  // print results
  printData();

  // create response command
  phi = ( P*(theta - theta_stable) ) + ( D*theta_dot );

  // send this to servo
  Rservo.writeMicroseconds(1500-phi);
  Lservo.writeMicroseconds(1500+phi);

  // update index
  if (up == hist - 1) {
    up = 0;
  }
  else {
    up += 1;
  }
  
}
