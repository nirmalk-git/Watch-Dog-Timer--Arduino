/* This code uses WDT to reset the code whenever it get stuck.
 *  The timing is very important in this scenario when ever the 
 *  cod eget stuck the timing infromation is svaed in the   eeprom.
 *  All other codes are the same.However the initial time should be
 *  stored in the EEPROM before the code is running. */

#include <EEPROM.h>
#include <Time.h>
#include <avr/wdt.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Wire.h>
#include "Kalman.h"
#include<Servo.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);


//RA and Dec of jupiter.
double RA = 3.895833; // RA is in degrees.
double Dec = -2.1558333*(PI/180); // Radian.

// Time calculation
double addr,DAY,HOUR,MIN,SEC;
float eeAddress = 0;
double YEAR = 2017;
double MON = 6;

double w,L,M,d,J2000,UTH,SIDTIME,AZI_ERROR,ALT_ERROR;
double LATITUDE,LONGITUDE,HA,ACT_ALTITUDE,ACT_AZIMUTH,ALTITUDE,AZIMUTH,alt,az;

//pid loop

float pGain = 1.5;
float dGain = 0.5;
float iGain = 0.5;

float pGain1 = 0.5;
float dGain1 = 0.15;
float iGain1 = 0.25;

// servomotors
Servo altservo;
Servo aziservo;

//Kalman filter
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

//timers
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
double accXangle, accYangle; // Angle calculate using the accelerometer
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter
uint32_t timer;
unsigned long  Starttimer,Starttimer1,Starttimer2;


// For mpu 9150
#define MPU9150_GYRO_CONFIG        0x1B   // R/W
#define MPU9150_ACCEL_CONFIG       0x1C   // R/W
#define MPU9150_ACCEL_XOUT_H       0x3B   // R  
#define MPU9150_ACCEL_XOUT_L       0x3C   // R  
#define MPU9150_ACCEL_YOUT_H       0x3D   // R  
#define MPU9150_ACCEL_YOUT_L       0x3E   // R  
#define MPU9150_ACCEL_ZOUT_H       0x3F   // R  
#define MPU9150_ACCEL_ZOUT_L       0x40   // R  
#define MPU9150_GYRO_XOUT_H        0x43   // R  
#define MPU9150_GYRO_XOUT_L        0x44   // R  
#define MPU9150_GYRO_YOUT_H        0x45   // R  
#define MPU9150_GYRO_YOUT_L        0x46   // R  
#define MPU9150_GYRO_ZOUT_H        0x47   // R  
#define MPU9150_GYRO_ZOUT_L        0x48   // R
#define MPU9150_PWR_MGMT_1         0x6B   // R/W

// I2C address 0x69 could be 0x68 depends on your wiring.
int MPU9150_I2C_ADDRESS = 0x68;

void setup(){
EEPROM.get(eeAddress, DAY);
EEPROM.get(eeAddress+sizeof(double), HOUR);
EEPROM.get(eeAddress+2*sizeof(double), MIN);
EEPROM.get(eeAddress+3*sizeof(double), SEC);
setTime(HOUR,MIN,SEC,DAY,MON,YEAR);
watchdogSetup();
Serial.begin(9600);

altservo.attach(10);
  aziservo.attach(8);
  Wire.begin();
  MPU9150_writeSensor(MPU9150_PWR_MGMT_1, 0);
  accY = MPU9150_readSensor(MPU9150_ACCEL_YOUT_L,MPU9150_ACCEL_YOUT_H);
  accZ = MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L,MPU9150_ACCEL_ZOUT_H);
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  kalmanX.setAngle(accXangle); // Set starting angle
  gyroXangle = accXangle;
  timer = micros();
 
  // for magneto meter
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
 Starttimer=millis();
 Starttimer1=millis();
 Starttimer2 = millis();
}

void watchdogSetup(void)
{
cli();
wdt_reset(); // reset the WDT timer
// Enter Watchdog Configuration mode:
WDTCSR |= (1<<WDCE) | (1<<WDE);
// Set Watchdog settings:
WDTCSR = (1<<WDIE) | (1<<WDE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
sei();
}



void loop(){
wdt_reset();

DAY =  day();
HOUR = hour();
MIN = minute();
SEC = second();

Serial.println(DAY);
Serial.println(HOUR);
Serial.println(MIN);
Serial.println(SEC);

LATITUDE = 13.0730*(PI/180);  // latitude is converted into radians.
LONGITUDE = 77.7967;


unsigned long Present = millis();
unsigned long Endtimer=Starttimer + 1000;


if (Endtimer < Present)
 
{
unsigned long now3 = millis();
HOUR = hour();
MIN = minute();
SEC = second();

//calculate altitude and azimuth
//Calculation of Julian day
 
 double jd = (floor (365.25*(YEAR + 4716.0)) + floor( 30.6001*( MON + 1.0)) +
 2.0- floor( YEAR/100.0 ) + floor( floor( YEAR/100.0 )/4.0 ) + DAY - 1524.5 + (HOUR + MIN/60 + (SEC+16)/3600)/24);

 //CALCULATION OF KEPLARIAN PARAMETERS
  d = jd-2451543.5;
 //CALCULATING ALTITUDE AND AZIMUTH
 // Find the J2000 value
   J2000 = jd - 2451545.0;
   UTH = HOUR + MIN/60 + (SEC)/3600;
 
 //Calculate local sidereal time

  SIDTIME = 100.46 + (0.985647 * J2000 )+ LONGITUDE + 15*UTH;
  SIDTIME = fmod(SIDTIME,360);
  HA =  SIDTIME- RA;// HA is in degrees.
 
  if (HA < 0){
  HA = HA +360;}
  HA = HA*(PI/180);  // HA is converted into radians.

 // calculation of actual altitude and azimuth.
  hadec_aa (LATITUDE, HA, Dec, &alt, &az);
 
  ACT_ALTITUDE = alt*(180/PI);
  ACT_AZIMUTH = az*(180/PI);
 
 if (ACT_AZIMUTH < 0 ) {
    ACT_AZIMUTH = ACT_AZIMUTH + 360;
 }
    unsigned long now4 = millis();
    Starttimer = Starttimer + now4 -now3 + 1000;
    
   String dataString = "";
   dataString += String(ACT_ALTITUDE)+" "+String(ALTITUDE)+ " "+ String(AZIMUTH)+" "+String(ACT_AZIMUTH);
   Serial.println(dataString);
   //Serial.println(dataString);
 }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned long Present1 = millis();
unsigned long Endtimer1=Starttimer1 + 100;

unsigned long now5 = millis();
if (Endtimer1 < Present1)
 
{
  gyroX = MPU9150_readSensor(MPU9150_GYRO_XOUT_L,MPU9150_GYRO_XOUT_H);
  accY = MPU9150_readSensor(MPU9150_ACCEL_YOUT_L,MPU9150_ACCEL_YOUT_H);
  accZ = MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L,MPU9150_ACCEL_ZOUT_H);
 
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  double gyroXrate = -((double)gyroX /131);
  gyroXangle += (gyroXrate+1.32) * ((double)(micros() - timer) / 1000000); // Calculate gyro angle without any filter
  //gyroXangle = gyroXrate * ((double)(micros() - timer) / 1000000);
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); // Calculate the angle using a Kalman filter
  timer = micros();
  ALTITUDE = kalAngleX-180;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);
 
  //float heading = atan2(-event.magnetic.x, event.magnetic.y);
 float heading = atan2(-event.magnetic.x, event.magnetic.y);
 
  float declinationAngle = 0.0270;
  heading += declinationAngle;
 
 // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
// Convert radians to degrees for readability.
  AZIMUTH = heading * 180/M_PI;
  unsigned long now6 = millis();
  Starttimer1 = Starttimer1 + 100 + now6 - now5;
}


//PID LOOP - ALT SERVO
unsigned long last_time1;
unsigned long now1 = millis();

float period1 = now1-last_time1;

if (period1 >= 200){
  float previousError1 = ALT_ERROR;
  ALT_ERROR = ACT_ALTITUDE-ALTITUDE;
  float integralPart1 =+ ALT_ERROR;
  if (integralPart1 < 0){
    integralPart1 = 0;}
    else if (integralPart1 > 180){
      integralPart1 = 180;
      }
   float Value1 = pGain1*(ALT_ERROR + dGain1*(ALT_ERROR - previousError1) + iGain1*integralPart1);
   altservo.write(174-2*(ACT_ALTITUDE-11)-Value1);

   float previousError = AZI_ERROR;  
   AZI_ERROR = ACT_AZIMUTH-AZIMUTH;
   float integralPart = integralPart+ AZI_ERROR;
   if (integralPart < -500){
    integralPart = -500;}
    else if (integralPart > 500){
      integralPart = 500 ;}
   float Value = pGain*AZI_ERROR+ dGain*(AZI_ERROR - previousError) + iGain*integralPart;
   if (Value<-500){
    Value = -500;}
    else if (Value>500){
      Value = 500;
      }
      if (abs(AZI_ERROR)<0.6)
      {
        aziservo.writeMicroseconds(1500);
          delay(4);}
       else{
        aziservo.writeMicroseconds(1500-Value);
        delay(4);}
        last_time1 = millis();
        }
}

int MPU9150_readSensor(int addrL, int addrH){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrL);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte L = Wire.read();

  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrH);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte H = Wire.read();

  return (H<<8)+L;
}

int MPU9150_readSensor(int addr){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  return Wire.read();
}

int MPU9150_writeSensor(int addr,int data){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission(true);

  return 1;
}


//new code to calculate alt and az

void hadec_aa (double lt,double ha, double dec,double *alt, double *az)

{
    aaha_aux (lt, ha, dec, az, alt);
}

#ifdef NEED_GEOC

double geoc_lat (double phi)
{
#define    MAXLAT    degrad(89.9999)    /* avoid tan() greater than this */
    return (fabs(phi)>MAXLAT ? phi : atan(tan(phi)/1.00674));
}
#endif

static void aaha_aux (double lt,double x, double y,double *p, double *q)
{
    static double last_lt = -3434, slt, clt;
    double cap, B;

    if (lt != last_lt) {
        slt = sin(lt);
        clt = cos(lt);
        last_lt = lt;
    }

    solve_sphere (-x, PI/2-y, slt, clt, &cap, &B);
    *p = B;
    *q = PI/2 - acos(cap);
}

void solve_sphere (double A, double b, double cc, double sc, double *cap, double *Bp)
{
    double cb = cos(b), sb = sin(b);
    double sA, cA = cos(A);
    double x, y;
    double ca;
    double B;

    ca = cb*cc + sb*sc*cA;
    if (ca >  1.0) ca =  1.0;
    if (ca < -1.0) ca = -1.0;
    if (cap)
        *cap = ca;

    if (!Bp)
        return;

    if (sc < 1e-7)
        B = cc < 0 ? A : PI-A;
    else {
        sA = sin(A);
        y = sA*sb*sc;
        x = cb - ca*cc;
        B = y ? (x ? atan2(y,x) : (y>0 ? PI/2 : -PI/2)) : (x>=0 ? 0 : PI);
    }

    *Bp = B;
    
}
  
// Write new values to the EEPROM. 
// write only when system hangs.
void EEPROM_writeDouble(int ee, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
       EEPROM.write(ee++, *p++);
}




//Interrupt Sub routine 
ISR(WDT_vect) // Watchdog timer interrupt.
{
Serial.println("Hi World");
addr = 0;
EEPROM_writeDouble(addr,day());
addr = addr + sizeof(double);
EEPROM_writeDouble(addr,hour());
addr = addr + sizeof(double);
EEPROM_writeDouble(addr,minute());
addr = addr + sizeof(double);
EEPROM_writeDouble(addr,second());
addr = addr + sizeof(double);
}
