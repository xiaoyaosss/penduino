/*
https://playground.arduino.cc/Main/RotaryEncoders?action=sourceblock&num=9

State machine boilerplate from https://www.edn.com/electronics-blogs/embedded-basics/4406821/Function-pointers---Part-3--State-machines
*/

#include "ArduinoJson-v6.9.1.h"

bool debug = false;
bool encoderPlain = false;
int sameNeeded = 100;
unsigned long interval = 10;

#define COMMAND_SIZE 64

StaticJsonDocument<COMMAND_SIZE> doc;
char command[COMMAND_SIZE];

enum PinAssignments {
encoderPinA = 2,
encoderPinB = 3,
drivePin = 6,
bumpPin = 5,
loadPin = 4,
clearButton = 8
};

volatile int encoderPos = 0;
int lastReportedPos = 1;

boolean A_set = false;
boolean B_set = false;

/**
 * Defines the valid states for the state machine
 * 
 */
typedef enum
{
  STATE_START_CALIBRATION,
  STATE_ZERO_ENCODER,
  STATE_AWAITING_STOP,
  STATE_DRIVING,
  STATE_START,
  STATE_STOPPED,
} StateType;

//state Machine function prototypes
void Sm_State_Start_Calibration(void);
void Sm_State_Zero_Encoder(void);
void Sm_State_Awaiting_Stop(void);
void Sm_State_Driving(void);
void Sm_State_Start(void);
void Sm_State_Stopped(void);


/**
 * Type definition used to define the state
 */
 
typedef struct
{
  StateType State; /**< Defines the command */
  void (*func)(void); /**< Defines the function to run */
} StateMachineType;
 
/**
 * A table that defines the valid states of the state machine and
 * the function that should be executed for each state
 */
StateMachineType StateMachine[] =
{
  {STATE_START_CALIBRATION, Sm_State_Start_Calibration},
  {STATE_ZERO_ENCODER, Sm_State_Zero_Encoder},
  {STATE_AWAITING_STOP, Sm_State_Awaiting_Stop},
  {STATE_DRIVING, Sm_State_Driving},
  {STATE_START, Sm_State_Start},
  {STATE_STOPPED, Sm_State_Stopped}
};
 
int NUM_STATES = 6;

/**
 * Stores the current state of the state machine
 */
 
StateType SmState = STATE_START_CALIBRATION;


void Sm_State_Start_Calibration(void)
{
  digitalWrite(bumpPin, HIGH); //HIGH for No Bump
  digitalWrite(loadPin, HIGH);  //LOW for No Load
  digitalWrite(drivePin, HIGH); //HIGH for No Drive
  SmState = STATE_AWAITING_STOP;
  
}

bool timeIsUp = false;

void timeUp();

void timeUp(void){
  
  timeIsUp = true;
}

void Sm_State_Awaiting_Stop(void)
{
  digitalWrite(loadPin,HIGH); //turn on the load
  int initial_position = encoderPos;
  bool swinging = true;
  int lastPos = 0;
  int thisPos = 0;
  int sameCount = sameNeeded; 
  while (swinging)
  {
    lastPos = thisPos;
    thisPos = encoderPos;
    if (thisPos == lastPos){
      sameCount = sameCount - 1;
    }
    else{
      sameCount = sameNeeded;
    }
      if (encoderPlain){
        Serial.println(encoderPos);
      }
      else{
        Serial.print("{\"cal\":true,\"enc\":");
        Serial.print(encoderPos);
        Serial.print(",\"sameCount\":");
        Serial.print(sameCount);
        Serial.println("}");
      }
    if (sameCount <= 0){
      swinging = false;
    }
  }

  digitalWrite(loadPin, LOW); 
  SmState = STATE_ZERO_ENCODER;
  

}






void Sm_State_Zero_Encoder(void)
{
  encoderPos = 0;
  Serial.println("{\"result\":\"calibrated\"}");
  SmState = STATE_START;
  
}


void report_encoder(void);

void report_encoder(void)
{
  unsigned long startTime = 0;

  bool waiting = true;
  startTime = millis();

  while(waiting){
    if ((millis() - startTime) >= interval){
      if (encoderPlain){
        Serial.println(encoderPos);
      }
      else{
        Serial.print("{\"enc\":");
        Serial.print(encoderPos);
        Serial.println("}");
      }
      waiting = false;
    }
  }  
  
}

void Sm_State_Driving(void)
{
  digitalWrite(bumpPin, HIGH);
  digitalWrite(drivePin, LOW);
  digitalWrite(loadPin, LOW);

  report_encoder();
  
  SmState = STATE_DRIVING;
  
  
  
}

void Sm_State_Start(void){
  digitalWrite(loadPin,LOW);//LOW is off
  digitalWrite(drivePin, HIGH); //HIGH is off
  digitalWrite(bumpPin,LOW);
  delay(20);
  digitalWrite(bumpPin,HIGH);
  delay(100);
  SmState = STATE_DRIVING;
}

void Sm_State_Stopped(void){
  digitalWrite(bumpPin,HIGH);
  digitalWrite(drivePin,HIGH);
  //digitalWrite(loadPin,LOW);
  report_encoder();
  SmState = STATE_STOPPED;
}



void setup() {
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(clearButton, INPUT);
  digitalWrite(encoderPinA, HIGH);  // turn on pull-up resistor
  digitalWrite(encoderPinB, HIGH);  // turn on pull-up resistor
  digitalWrite(clearButton, HIGH);
  pinMode(bumpPin, OUTPUT);
  pinMode(loadPin, OUTPUT);
  pinMode(drivePin, OUTPUT);
  digitalWrite(bumpPin, HIGH);
  digitalWrite(loadPin, LOW);
  digitalWrite(drivePin, LOW);
  

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);

  Serial.setTimeout(50);
  Serial.begin(57600);
  
}


void loop(){
  
  Sm_Run();
  
} 

/**
 *  Example commmands
 *  {"cmd":"interval","param":200}
 *  {"cmd":"stop","param":"loaded"}
 *  {"cmd":"stop","param":"unloaded"}
 *  {"cmd":"stop"}
 *  {"cmd":"start"}
 *  {"cmd":"calibrate"}
 */

StateType readSerialJSON(StateType SmState){
  

  
  if (Serial.available() > 0) 
  {

    char start[] = "start";
    char stop[] = "stop";
    char cal[] = "calibrate";
    char report_interval[] = "interval";
    char loaded[] = "loaded";
    char unloaded[] = "unloaded";
    
    Serial.readBytesUntil(10, command, COMMAND_SIZE);
                
    deserializeJson(doc, command);

    const char* cmd = doc["cmd"];
    
    if (strcmp(cmd,report_interval)==0)
    {
      int new_interval = doc["param"];
      if ((new_interval > 0) && (new_interval < 1000))
      {
        interval = new_interval;
        Serial.println("{\"result\":\"ok\"}");
      }
      else
      {
        Serial.println("{\"error\":\"interval must be between 0 - 1000\"}");
      }
    }
    
    if (strcmp(cmd,stop)==0)
    {
      if (((SmState == STATE_DRIVING) || (SmState == STATE_STOPPED)))
      { 
        Serial.print("{\"result\":\"stopped\"");
        const char * param = doc["param"];
      
        if (strcmp(param, loaded)==0){
          digitalWrite(loadPin,HIGH); //load is on
        }
        else if (strcmp(param, unloaded)==0){
          digitalWrite(loadPin,LOW); //load is off
        }
        
        Serial.print(",\"loaded\":");
        Serial.print(digitalRead(loadPin));
        Serial.print("}");
        SmState = STATE_STOPPED;
      }
      else
      {
        Serial.print("{\"error\":\"In wrong state to stop\"}");
      } 
  
    }//if stop
    
    if (strcmp(cmd,cal)==0)
    {
    
      if ((SmState == STATE_DRIVING) || (SmState == STATE_STOPPED))
      { 
        Serial.println("{\"result\":\"calibrating\"}");
        SmState = STATE_START_CALIBRATION;
      }
      else{
      
      Serial.println("{\"error\":\"In wrong state to calibrate\"}");
      }
    } //if cal

    if (strcmp(cmd,start)==0)
    {
    
      if ((SmState == STATE_DRIVING) || (SmState == STATE_STOPPED))
      { 
        Serial.println("{\"result\":\"starting\"}");
        SmState = STATE_START;
      }
      else{
      
      Serial.println("{\"error\":\"In wrong state to start\"}");
      }
    } //if cal

  } //if bytes available
  return SmState;
}

StateType readSerial(StateType SmState){
  
  byte incomingByte;

  if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();
                
    if ((incomingByte == 10) ||  (incomingByte == 13))//linefeed or carriage return
    {
      return SmState; 
    }
                
    else if ((SmState == STATE_DRIVING) && (incomingByte == 88)){ //X
      SmState = STATE_STOPPED;
      Serial.println("{\"cmd\":\"X\"}");
    }
    else if ((SmState == STATE_STOPPED) && (incomingByte == 66)) //B
    {
      SmState = STATE_START;
      Serial.println("{\"cmd\":\"B\"}");
    }
    else if (((SmState == STATE_STOPPED) || (SmState == STATE_DRIVING))&&(incomingByte == 67)) //C
    {
      SmState = STATE_START_CALIBRATION;
      Serial.println("{\"cmd\":\"C\"}");
    }
    else if (((SmState == STATE_STOPPED) || (SmState == STATE_DRIVING)) && (incomingByte == 76)) //L
    {
      digitalWrite(loadPin,HIGH); //Load goes on 
      SmState = STATE_STOPPED;
      Serial.println("{\"cmd\":\"L\"}");
    }
    else if (((SmState == STATE_STOPPED) || (SmState == STATE_DRIVING)) && (incomingByte == 108)) //l
    {
      digitalWrite(loadPin,LOW); //Load goes off
      SmState = STATE_STOPPED;
      Serial.println("{\"cmd\":\"l\"}");
    }
  }
  return SmState;
}

void Sm_Run(void)
{
  if (SmState < NUM_STATES)
  {
    if (debug){
      Serial.print("{\"State\":");
      Serial.print(SmState);
      Serial.println("}");
    }
    SmState = readSerialJSON(SmState);
    (*StateMachine[SmState].func)();
    
  }
  else{
    Serial.println("Exception in State Machine");
  }
  
}


void old_loop(){
  
  if (true)  {
    //Serial.print("Index:");
    Serial.print(encoderPos, DEC);
    Serial.println();
    lastReportedPos = encoderPos;
  }
  if (digitalRead(clearButton) == LOW)  {
    encoderPos = 0;
  }
}

// Interrupt on A changing state
void doEncoderA() {
  // Test transition
  A_set = digitalRead(encoderPinA) == HIGH;
  // and adjust counter + if A leads B
  encoderPos += (A_set != B_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoderB() {
  // Test transition
  B_set = digitalRead(encoderPinB) == HIGH;
  // and adjust counter + if B follows A
  encoderPos += (A_set == B_set) ? +1 : -1;
}
