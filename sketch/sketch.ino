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
clearButton = 8,
encoderPinA = 2,
encoderPinB = 3,
driveLed = 10,
drivePin = 6,
loadLed = 12,
loadPin = 4,
brakingLed = 8
};

volatile int encoderPos = 0;
volatile int encoderPosLast = 0;
volatile bool goingPosDir = false;
volatile bool onPosSide = false;
volatile int driveThresh = 20;
volatile int startDelay = 10;
volatile bool brakeStop = false;
volatile int brakeThresh = 100;
volatile bool doDriving = false;
volatile bool doStopping = false;
volatile int modifiedBrakeThresh = 100;
volatile bool goingPosDirLast  = false;
volatile int peakSwing = 0;

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
  
  // can't use the brake because the encoder values could be wrong
  // so use the load instead.
  
  //drive OFF
  digitalWrite(drivePin, HIGH); // PNP - HIGH for OFF
  digitalWrite(driveLed, LOW);  // opposite sense to drivePin
  coilMessage(LOW);
  
  // load ON
  digitalWrite(loadPin, HIGH);  // NPN - HIGH for ON
  digitalWrite(loadLed, HIGH);  // same sense as loadPin
  loadMessage(HIGH);

  doDriving = false;  // ensure drive not applied
  doStopping = false; //ensure brake not used
  
  SmState = STATE_AWAITING_STOP; 
  
}

bool timeIsUp = false;

void timeUp();

void timeUp(void){
  
  timeIsUp = true;
}

void Sm_State_Awaiting_Stop(void)
{
 
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
        Serial.print(",\"time\":");
        Serial.print(millis());      
        Serial.println("}");
      }
    if (sameCount <= 0){
      swinging = false;
    }
  }

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
        Serial.print(",\"time\":");
        Serial.print(millis());  
        Serial.println("}");
      }
      waiting = false;
    }
  }  
  
}

void Sm_State_Driving(void){
  doDriving = true;
  doStopping = false;
  // let the encoder interrupts drive the drive pin

  //load OFF
  digitalWrite(loadLed, LOW);
  digitalWrite(loadPin, LOW);
  loadMessage(LOW);
  
  report_encoder();
  
  SmState = STATE_DRIVING;
  
}

void Sm_State_Start(void){
  
  doDriving = true;
  doStopping = false;
  
  // load OFF
  digitalWrite(loadLed,LOW); //
  digitalWrite(loadPin,LOW);
  loadMessage(LOW);
  
  //drive ON
  digitalWrite(driveLed, HIGH);
  digitalWrite(drivePin, LOW); 
  coilMessage(HIGH);

  delay(startDelay);
  SmState = STATE_DRIVING;
}

void Sm_State_Stopped(void){
  doDriving = false;
  doStopping = true;
  // load stays as it was
  
  // drive OFF
  digitalWrite(driveLed, LOW); 
  digitalWrite(drivePin,HIGH);
  coilMessage(LOW);
  
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

  pinMode(brakingLed, OUTPUT);
  digitalWrite(brakingLed, LOW);  
 
  // load is OUTPUT, start OFF
  pinMode(loadLed, OUTPUT);
  digitalWrite(loadLed, LOW);  
  loadMessage(LOW);
  
  pinMode(loadPin, OUTPUT);
  digitalWrite(loadPin, LOW); //NPN
  
  // drive is OUTPUT, start OFF
  pinMode(driveLed, OUTPUT);
  digitalWrite(driveLed, LOW);
  coilMessage(LOW);
     
  pinMode(drivePin, OUTPUT);  
  digitalWrite(drivePin, HIGH); // PNP

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
 *  {"cmd":"drive","param":30}
 *  {"cmd":"stop","param":"loaded"}
 *  {"cmd":"stop","param":"unloaded"}
 *  {"cmd":"stop","param":"brake"}
 *  {"cmd":"brake","param":30}
 *  {"cmd":"stop"}
 *  {"cmd":"start","param":50}
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
    char brake[] = "brake";
    char drive[] = "drive";
    
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

    if (strcmp(cmd,drive)==0)
    {
      int newDriveThresh = doc["param"];
      if ((newDriveThresh >= 0) && (newDriveThresh <= 100))
      {
        driveThresh = newDriveThresh;
        Serial.println("{\"result\":\"ok\"}");
      }
      else
      {
        Serial.println("{\"error\":\"drive must be between 0 - 100 (inclusive)\"}");
      }
    }

    
    if (strcmp(cmd,brake)==0)
    {
      int newBrakeThresh = doc["param"];
      if ((newBrakeThresh >= 0) && (newBrakeThresh <= 100))
      {
        brakeThresh = newBrakeThresh;
        Serial.println("{\"result\":\"ok\"}");
      }
      else
      {
        Serial.println("{\"error\":\"break must be between 0 - 100 (inclusive)\"}");
      }
    }
    
    if (strcmp(cmd,stop)==0)
    {
      if (((SmState == STATE_DRIVING) || (SmState == STATE_STOPPED)))
      { 
        Serial.print("{\"result\":\"stopped\"");
        const char * param = doc["param"];
      
        if (strcmp(param, loaded)==0){
          //load is on
          digitalWrite(loadLed,HIGH); 
          digitalWrite(loadPin,HIGH); 
          loadMessage(HIGH);
          brakeStop = false;

        }
        else if (strcmp(param, unloaded)==0){
          //load is off
          digitalWrite(loadLed,LOW); 
          digitalWrite(loadPin,LOW); 
          loadMessage(LOW);
          brakeStop = false;
        }
        else if (strcmp(param, brake)==0){
          //load is off - else short circuit!
          digitalWrite(loadLed,LOW); 
          digitalWrite(loadPin,LOW); 
          loadMessage(LOW);
          brakeStop = true;
        }        
        Serial.print(",\"loaded\":");
        Serial.print(digitalRead(loadPin));
        Serial.print(",\"brake\":");
        Serial.print(brakeStop);
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

        int newStartDelay = doc["param"];
        if ((newStartDelay >= 0) && (newStartDelay <= 500))
        {
          startDelay = newStartDelay;

          Serial.println("{\"result\":\"starting\"}");
          SmState = STATE_START;
          
        }
        else
        {
          Serial.println("{\"error\":\"start must be between 0 - 500 [ms](exclusive)\"}");
        }
        

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
      digitalWrite(loadLed,HIGH); 
      digitalWrite(loadPin,HIGH); //Load goes on 
      SmState = STATE_STOPPED;
      Serial.println("{\"cmd\":\"L\"}");
    }
    else if (((SmState == STATE_STOPPED) || (SmState == STATE_DRIVING)) && (incomingByte == 108)) //l
    {
        digitalWrite(loadLed,LOW); 
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


// Interrupt on A changing state
void doEncoderA() {
  // Test transition
  A_set = digitalRead(encoderPinA) == HIGH;
  // and adjust counter + if A leads B
  encoderPosLast = encoderPos;
  encoderPos += (A_set != B_set) ? +1 : -1;
  encoderWrap();
  driver();
  
}

// Interrupt on B changing state
void doEncoderB() {
  // Test transition
  B_set = digitalRead(encoderPinB) == HIGH;
  // and adjust counter + if B follows A
  encoderPosLast = encoderPos;
  encoderPos += (A_set == B_set) ? +1 : -1;
  encoderWrap();
  driver();
}

void driver(){
  // default to LEDs off, no drive to coil
  bool applyCoilPower = false;
  bool showDriveLed = false;
  bool showBrakeLed = false;
  
  if (encoderPos > 0){
    onPosSide = true;  
  } else {
    onPosSide = false;  
  }
  
  goingPosDirLast = goingPosDir;
  if ((encoderPos - encoderPosLast) > 0) {
    goingPosDir = true;  
  } else{
    goingPosDir = false;  
  }

  // we've changed directions
  if (goingPosDir != goingPosDirLast) {
    peakSwing = abs(encoderPos);
  }
  
  if (doDriving && 
     (goingPosDir == onPosSide) && 
     (abs(encoderPos) > 0) && 
     (abs(encoderPos) < driveThresh)) {
  
          showDriveLed = true; 
          applyCoilPower = true;

  }  else if (doStopping && (brakeStop == true)){

      // use full braking thresh if swing is large
      if (peakSwing > modifiedBrakeThresh) {
          modifiedBrakeThresh = brakeThresh; 
      } 
      
      //reduce if swing is within thresh, to avoid hangup
      if (peakSwing < modifiedBrakeThresh){
          modifiedBrakeThresh = peakSwing - 3; //is an abs value
          if (peakSwing < 0 ){
             peakSwing = 0;  
          }
      }
  
      if ((goingPosDir != onPosSide) && (abs(encoderPos) < modifiedBrakeThresh)) {
          // approaching centre, within the modified brake threshold, so apply brake    
          showBrakeLed = true; 
          applyCoilPower = true;
      } 
  } 
  
  //update outputs with what we have decided to do
  digitalWrite(brakingLed, showBrakeLed);
  digitalWrite(driveLed, showDriveLed);
  digitalWrite(drivePin, !applyCoilPower); //active low driver
  coilMessage(applyCoilPower);
  loadMessage(showDriveLed);   
    
}

void coilMessage(bool energised){
        return;
        Serial.print("{\"coil\":");
        Serial.print(energised);
        Serial.print(",\"time\":");
        Serial.print(millis());  
        Serial.println("}");
}

void loadMessage(bool on){
        return;
        Serial.print("{\"load\":");
        Serial.print(on);
        Serial.print(",\"time\":");
        Serial.print(millis());  
        Serial.println("}");
}

void encoderWrap(void){
  if (encoderPos > 1200) {
    encoderPos -= 2400;
    } else if (encoderPos < -1200) {
      encoderPos += 2400;
      }
}
