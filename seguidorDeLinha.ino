#include <QTRSensors.h> //For the sensors to function
#include "MotorController.h" //For the engine to work

QTRSensors qtr; //Calls the functions to the sensors


/*
>>>>>>>>>>>>>>>     DEFINE SECTION     <<<<<<<<<<<<<<< 
*/

// Important values to be utilized later in
#define RUNTIME 15500               // Defines the time the line follower must go over the circuit
#define kP 0.2                      // Defines the value of the constant for Proportional (vary from robot to robot)
#define kD 5.0                        // Defines the value of the constant for Derivative (vary from robot to robot)
#define SETPOINT 255                // Base speed for the robot
#define MAX_SET 255                 // Maximum speed for the robot
#define MIN_SET   1                 // Minimum speed for the robot
const uint8_t sensorCount = 6;      // Num of sensors
uint16_t sensors[sensorCount];      // Array of sensors
DCMotorController m_right = DCMotorController(6,2,4); //Set pins for the engines (enable, in1, in2)
DCMotorController m_left = DCMotorController(3,5,7);  //Set pins for the engines (enable, in1, in2)

/*
>>>>>>>>>>>>>>>     SETUP & LOOP     <<<<<<<<<<<<<<< 
*/

void setup() {
  Serial.begin(9600); //Listening port
  pinMode(LED_BUILTIN, OUTPUT);
  delay(500);
  engineTest(255,255);
  readSensors();
}

void loop() {
    //AUTOBOTS, ROLL OUT!
  LINE_FOLLOWER(false);
}

/*
>>>>>>>>>>>>>>>     ENGINE SETUP     <<<<<<<<<<<<<<<
*/

void speed_controller (int s_left, int s_right){
  //Setting speed control for left and right engines
  m_left.write(s_left);
  m_right.write(s_right);
}

/*
>>>>>>>>>>>>>>>     QTR SENSORS SETUP     <<<<<<<<<<<<<<<
*/

void setSensors (){
  // Initializes the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensorCount);
}

void caliSensors(){
  setSensors();
  delay(500);
  //calibrates the sensors for best performance
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i=0;i<150;i++){
    qtr.calibrate();
    delay(20);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

/*
>>>>>>>>>>>>>>>     TESTING FUNCTIONS     <<<<<<<<<<<<<<< 
*/

void engineTest(int left, int right){
  // Engine calibration
  digitalWrite(LED_BUILTIN, HIGH);
  while(millis()<2000){
     speed_controller(left, right); // Goes forward
  }
  delay(500);
  while(millis()<4500){
     speed_controller(-left, -right); // Goes backward
  }
  delay(500);
  while(millis()<6000){
     speed_controller(0, 0); // Stay still
  }
  delay(500);
  while(millis()<7500){
     speed_controller(-left, right); // Goes to the right
  }
  delay(500);
  while(millis()<9000){
     speed_controller(left, -right); // Goes to the left
  }
  speed_controller(0,0);
  digitalWrite(LED_BUILTIN, LOW);
}

void readSensors(){
  //Display what was read by sensors
  caliSensors();
  for(uint8_t i=0;i<sensorCount;i++){
    Serial.print(qtr.calibrationOn.minimum[i]); //prints minimum reads
    Serial.print(' ');
  }
  Serial.println();
  for(uint8_t i=0;i<sensorCount;i++){
    Serial.print(qtr.calibrationOn.maximum[i]); //prints maximum reads
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

/*
>>>>>>>>>>>>>>>     PID: GETTING ERROR, MAKING THE CALCULUS, RUNNING     <<<<<<<<<<<<<<< 
*/

void LINE_FOLLOWER (bool prints){
  // receives position based on the position of sensor over the line
  bool flag = true;
  long currentTime = millis();

  while(flag){
    flag = engineStop(RUNTIME, currentTime);  
    
    uint16_t position = qtr.readLineWhite(sensors);
    if(prints){
      for(uint8_t i=0;i<sensorCount;i++){
        Serial.print(sensors[i]);
        Serial.print('\t');
      }
      Serial.println(position);
      delay(250);
    }else{
      static int pErr=0;
      int err = position - 2500;
      int speedo = (kP*err)+kD*(err-pErr);
      pErr=err;
      int motor_l = SETPOINT - speedo;
      int motor_r = SETPOINT + speedo;
      VRUM(motor_l,motor_r);
    }  
  }
}

void VRUM (int speedoL, int speedoR){
  if(speedoL>MAX_SET){
    speedoL = MAX_SET;
  }else if(speedoL<MIN_SET){
    speedoL = MIN_SET;
  }
  if(speedoR>MAX_SET){
    speedoR = MAX_SET;
  }else if(speedoR<MIN_SET){
    speedoR = MIN_SET;
  }
  speed_controller(speedoL,speedoR);
}

bool engineStop(long runtime, long currentTime){ //a paradinha a, a
  // Robot stops accordinly to the defined RUNTIME
  if (millis() >= (runtime+currentTime)){
    engineTest(0,0);
    return false;
  }
  return true;
}