
// configuring IMU variables
#include <Adafruit_LSM6DS33.h>

Adafruit_LSM6DS33 lsm6ds33;


double mX_t = 0.0; //moment_X: falling towards the pi is +ve, falling towards wheels is -ve
double mX_t_1 = 0.0;
double aY_t = 9.81; // acceleration_Y: gravity is +ve
double aY_t_1 = 9.81; 
double aZ_t = 0.0; 
double aZ_t_1 = 0.0; 
const double target_aZ = 0.0;
const double target_aY = 9.81;

//configuring DC MOTOR drive:

#define M1_ENABLE 3
#define M1_DIRA 5
#define M1_DIRB 4
#define M2_ENABLE 9
#define M2_DIRA 7
#define M2_DIRB 8 

//PID variables 
const double Kp = 4000; 
const double Ki = 0.75; 
const double Kd = 2000; 
double e_t = 0; 
double e_t_1 = 0; 
double sigma_e = 0; 
double u_t; 

void engage_forward(){ // forward will spin wheels in +ve mX direction
     digitalWrite(M1_DIRA,HIGH); 
     digitalWrite(M1_DIRB,LOW);
     digitalWrite(M2_DIRA,HIGH); 
     digitalWrite(M2_DIRB,LOW);
  };

void engage_reverse(){ // will spin wheels in -ve mX direction
     digitalWrite(M1_DIRA,LOW); 
     digitalWrite(M1_DIRB,HIGH);
     digitalWrite(M2_DIRA,LOW); 
     digitalWrite(M2_DIRB,HIGH);
  };
  


void configure_IMU(){
    if (!lsm6ds33.begin_I2C()) {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DS33 Found!");

  // lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (lsm6ds33.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (lsm6ds33.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DS33
  }

  // lsm6ds33.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (lsm6ds33.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // lsm6ds33.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (lsm6ds33.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  lsm6ds33.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds33.configInt2(false, true, false); // gyro DRDY on INT2
  }

void update_input_vals(){
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  mX_t_1 = mX_t; 
  mX_t = gyro.gyro.x; 
  aY_t_1 = aY_t; 
  aY_t = accel.acceleration.y;
  aZ_t_1 = aZ_t; 
  aZ_t = accel.acceleration.z; 
  }

void update_error(){
  e_t_1 = e_t; 
  e_t = target_aZ - aZ_t; 
  sigma_e += e_t;
  }

void compute_output(){
  u_t = round(Kp*e_t + Ki*sigma_e + Kd*(e_t - e_t_1)); 
  }

void drive_motor(int u){
    if (u < 0){
      engage_forward();
      } else {
      engage_reverse();
      }
    analogWrite(M1_ENABLE,abs(u)); 
    analogWrite(M2_ENABLE,abs(u));
  }


void setup() {
  //initialize DC motor 
  pinMode(M1_ENABLE,OUTPUT);
  pinMode(M1_DIRA,OUTPUT);
  pinMode(M1_DIRB,OUTPUT);
  pinMode(M2_ENABLE,OUTPUT); 
  pinMode(M2_DIRA,OUTPUT); 
  pinMode(M2_DIRB,OUTPUT);
  Serial.begin(115200);
  configure_IMU(); // initialize IMU
}


void loop() {
 update_input_vals(); 
 update_error(); 
 compute_output();
 if (u_t <= -255) {
    drive_motor(-255); 
  } else if (u_t >= 255){
    drive_motor(255);
    } else {
    drive_motor(u_t);
      }
 Serial.println("aZ (m/s^2) : ");
 Serial.print(aZ_t); 
 Serial.println();
 Serial.println("Error : "); 
 Serial.print(u_t); 
 Serial.println();
 //Serial.println(sigma_e);
 Serial.println();

}
