/*
 * Script for an Arduino Drone, by Miguel Sánchez Domínguez
 * https://github.com/Miguel-San/Arduino-Drone 
 * 
 * NOTE: FlySkyIBus, MPU6050 and I2Cdev libraries are not mine. 
 * Please, to get the files from their original repositories and give credit to their authors, read 
 * the README.md from the GitHub repo linked above.
 * 
 */

#include "PID.h"
#include "brushlessMt.h"

#include "MPU6050.h"
#include "Wire.h"
#include "I2Cdev.h"

#include "FlySkyIBus.h"

#include <SoftwareSerial.h>

//------------------------------------------------------  Global Variable Declarations

// Motor disposal

//   1       2
//    \\   //
//     \\ //
//     // \\
//    //   \\
//   4       3

//Motor control              

brushlessMt mt1;             
brushlessMt mt2;             
brushlessMt mt3;            
brushlessMt mt4;   

double thrust;
int vel;
int velMax = 1250, velMin;


//MPU6050

#define RESTRICT_PITCH

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

MPU6050 mpu;    //SDA ->  A4
                //SCL ->  A5

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter


//Time control

uint32_t timer;
double dt;
double loopInterval = 0.005;


//PID

double kp, ki, kd;
double input_x, input_y, output_x = 0, output_y = 0, setpoint_x = 0, setpoint_y = 0;
double minOutput = -50, maxOutput = 50;

PID pid_x(&input_x, &output_x);
PID pid_y(&input_y, &output_y);


//FlySky Communication

int pwrSwitch = 1000;


//Bluetooth and serial communication

SoftwareSerial serialBt(10, 11); //RX-TX

String msg;
bool reading = false;
bool graphs = false;

enum variableNames{
  PING,  ACTION,  KP1,  KI1,  KD1,  KP2,  KI2,  KD2,
  M1,  M2,  M3, M4,  VM,  Vm,  V1,  V2,  V3,  PID1,  PID2,
  PID3,  PID4,  EJEX,  EJEY,  EJEZ,   CHK  
  };

//------------------------------------------------------  End of Global Variable Declarations

void setup() {
  pinMode(13, OUTPUT);  //LED will turn on when setup is done

  
  //Comm initialization

  IBus.begin(Serial);
  serialBt.begin(9600);

  
  //PID Configuration

  pid_x.setOutputLimits(minOutput, maxOutput);
  pid_x.setSetpoint(setpoint_x);
  pid_x.setSampleTime(0.05);

  pid_y.setOutputLimits(minOutput, maxOutput);
  pid_y.setSetpoint(setpoint_y);
  pid_y.setSampleTime(0.05);


  //MPU6050 Initialization

  mpu.initialize(); 

  int16_t acc[3];
  int16_t g[3];

  mpu.getMotion6(&acc[0], &acc[1], &acc[2], &g[0], &g[1], &g[2]);

  accX = (double) acc[0];
  accY = (double) acc[1];
  accZ = (double) acc[2];

  gyroX = (double) g[0];
  gyroY = (double) g[1];
  gyroZ = (double) g[2];

  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  delay(2000);  //Delay needed for the MPU to steady its readings


  //Motor initialization

  mt1.init(9);
  mt1.setRange(1100, 1250);   //Reduced range for testing
  mt1.powerOff();
  
  mt2.init(6);
  mt2.setRange(1100, 1250);
  mt2.powerOff();
  
  mt3.init(3);
  mt3.setRange(1100, 1250);
  mt3.powerOff();
  
  mt4.init(5);
  mt4.setRange(1100, 1250);
  mt4.powerOff();


  digitalWrite(13, HIGH);   //Setup finished
  timer = micros();

}

void loop() {
  readSer();

  dt = (double) (micros() - timer) / 1000000;

  readFlySky();

  pid_x.setSetpoint(setpoint_x);
  pid_y.setSetpoint(setpoint_y);

  while(pwrSwitch < 2000){        //Power switch on RC controller
    readFlySky();
    mt1.powerOff();
    mt2.powerOff();
    mt3.powerOff();
    mt4.powerOff();  
    
  }

  if(dt >= loopInterval){
    readAngle();
    input_x = compAngleX;
    input_y = compAngleY;
    timer = micros();

    pid_x.calc();
    pid_y.calc();

    setMotors(thrust, output_x, output_y);
        
  }

}



void setMotors(double _thrust, double corr_x, double corr_y){

  //Motor 1
  vel = (int) (_thrust + corr_x + corr_y);
  mt1.setVel(vel);

  //Motor 2
  vel = (int) (_thrust + corr_x - corr_y);
  mt2.setVel(vel);

  //Motor 3
  vel = (int) (_thrust - corr_x - corr_y);
  mt3.setVel(vel);

  //Motor4
  vel = (int) (_thrust - corr_x + corr_y);
  mt4.setVel(vel);
  
  
}



void readFlySky(){
  IBus.loop();

  pwrSwitch = IBus.readChannel(6);
  setpoint_x = map(IBus.readChannel(0), 1000, 2000, -30, 30);   //The setpoint will vary between -30 and 30 degrees.
  setpoint_y = map(IBus.readChannel(1), 1000, 2000, -30, 30);   //Susceptible of being increased in the future
  thrust = IBus.readChannel(2);
}



void readSer(){
  if(serialBt.available()){
    char ch = serialBt.read();

    if(ch == '#' && reading){
      reading = false;
      serialBt.flush();
      processMsg();  
      
    }else if(ch == '$'){
      reading = true;  
      
    }else if(reading){
      msg.concat(ch);  
      
    }
    
  }  
  
}



boolean doCheckSum(String _msg, int start, int ending, int _checkSum){

  int checkSum = 0;

  if(ending - 1 > _msg.length()) return false;

  for (int i = 0; i < ending - 1 - start; i++){ 
    
    checkSum += _msg[i];
    
    }

  return(checkSum == _checkSum);

}



void processMsg(){

  int index = 0, lastIndex = 0;  
  int count = 0;
  int checkSumIndex = 0;

  //Count how many values we have to extract

  while(index >= 0){

    index = msg.indexOf('/', lastIndex + 1);
    count++;

    //If index == -1, We arrived at the end of the String, being lastIndex the checksum index
    if(index == -1) checkSumIndex = lastIndex;
    
    lastIndex = index + 1;
    
    }

  String *readVariables = new String[count];

  //We go over the string extracting the data

  for(int i = 0; i < count - 1; i++){

      index = msg.indexOf('/', lastIndex + 1);
      readVariables[i] = msg.substring(lastIndex, index);
      lastIndex = index + 1;    
    }

  if(doCheckSum(msg, 0, checkSumIndex, msg.substring(checkSumIndex, msg.length()).toInt())){
  asignVariables(readVariables);
    
  }
  
  delete[] readVariables;
  readVariables = 0;

  msg = "";
  
}



void asignVariables(String *_readVariables){
  
  serialBt.println("$*#");

  //We check if its an info request

  if(_readVariables[PING] == "1"){
    return;
    
  }else if (_readVariables[PING] == "2"){
    serialBt.print("KP1");serialBt.print('\t');serialBt.print("KI1");serialBt.print('\t');serialBt.print("KD1");serialBt.print('\t');
    serialBt.print("M1");serialBt.print('\t');serialBt.print("PID1");serialBt.print('\t');serialBt.print("X_AXIS");serialBt.print('\t');
    serialBt.print("Y_AXIS");serialBt.print('\t');serialBt.print("X_OUTPUT");serialBt.print('\t');serialBt.print("Y_OUTPUT");serialBt.println('\t');
    
    serialBt.print(kp);serialBt.print('\t');serialBt.print(ki);serialBt.print('\t');serialBt.print(kd);serialBt.print('\t');
    serialBt.print(thrust);serialBt.print('\t');serialBt.print(pid_x.isPowered());serialBt.print('\t');serialBt.print(compAngleX);serialBt.print('\t');
    serialBt.print(compAngleY);serialBt.print('\t');serialBt.print(output_x);serialBt.print('\t');serialBt.print(output_y);serialBt.println('\t');
    
  }else if (_readVariables[PING] == "15"){

    graphs = true;
      
  }
  else{ 
    
    kp = _readVariables[KP1].toDouble();
    ki = _readVariables[KI1].toDouble();
    kd = _readVariables[KD1].toDouble();

    pid_x.setConstants(kp, ki, kd);
    pid_y.setConstants(kp, ki, kd);

    velMin = _readVariables[Vm].toInt()*2 + 1000;

    thrust = _readVariables[M1].toDouble()*2 + 1000;    

    minOutput = velMin - thrust;
    maxOutput = velMax - thrust;

    pid_x.setOutputLimits(minOutput, maxOutput);
    pid_y.setOutputLimits(minOutput, maxOutput);
  
    if(_readVariables[ACTION] == "St"){

      //TurnOff the PID

      pid_x.setPower(POWEROFF);
      pid_y.setPower(POWEROFF);

      //Set motor speeds to 0

      mt1.powerOff();
      mt2.powerOff();
      mt3.powerOff();
      mt4.powerOff();
    
    
    }else{

      if(_readVariables[ACTION] == "Cf" || _readVariables[ACTION] == "Rp"){

        //Set motor speeds to min vel.
        mt1.setVel(velMin);
        mt2.setVel(velMin);
        mt3.setVel(velMin);
        mt4.setVel(velMin);
        
        //Turn off PID
        pid_x.setPower(POWEROFF);
        pid_y.setPower(POWEROFF);
      
      }else if(_readVariables[ACTION] == "ON"){        

        if(_readVariables[PID1] == "1"){

          pid_x.setPower(POWERON);
          pid_y.setPower(POWERON);
          
        }else if(_readVariables[PID1] == "0"){
          
          pid_x.setPower(POWEROFF);
          pid_y.setPower(POWEROFF);  
          
        }      
      }    
    }
  }

  _readVariables = 0;
  
}



void readAngle(){

  int16_t acc[3];
  int16_t g[3];
  
  mpu.getMotion6(&acc[0], &acc[1], &acc[2], &g[0], &g[1], &g[2]);

  accX = (double) acc[0];
  accY = (double) acc[1];
  accZ = (double) acc[2];

  gyroX = (double) g[0];
  gyroY = (double) g[1];
  gyroZ = (double) g[2];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s


  #ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && compAngleX > 90) || (roll > 90 && compAngleX < -90)) {
      compAngleX = roll;
      gyroXangle = roll;
    } else
      //kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(compAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
      //kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && compAngleY > 90) || (pitch > 90 && compAngleY < -90)) {
      compAngleY = pitch;
      gyroYangle = pitch;
    } else
      //kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  
    if (abs(compAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
      //kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif


  gyroXangle += gyroXrate * dt;
  gyroYangle += gyroYrate * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = compAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = compAngleY;
  
}
