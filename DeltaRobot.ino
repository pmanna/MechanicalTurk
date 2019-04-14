
#include <string.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "Kinematics.h"

#define DEBUG
//#define TEST_MOVES

const int MAGNET_PIN  = PA8;
const float MAX_Z     = -120.0;
const long stepsPerRevolution = 4076;
const float stepsPerDegree   = (float)stepsPerRevolution / 360.0;
const long zeroOffsetSteps  = (long)(stepsPerDegree * 20.0);

char inData[80];
char *outData[4] = {NULL};
byte idx = 0;
bool motorsEnabled = true;

// Define a stepper and the pins it will use
// WARNING: it's IN1, IN2, IN4, IN3, due to the winding!
AccelStepper motor1(AccelStepper::FULL4WIRE,PC14,PC15,PA2,PA1);
AccelStepper motor2(AccelStepper::FULL4WIRE,PA4,PA5,PA7,PA6);
AccelStepper motor3(AccelStepper::FULL4WIRE,PB0,PB1,PB11,PB10);
MultiStepper arm;

#ifdef TEST_MOVES
float samplePositions[6][3] = {
  {0.0,0.0,-200.0},
  {15.0,15.0,-260.0},
  {15.0,15.0,-200.0},
  {60.0,60.0,-200.0},
  {60.0,60.0,-260.0},
  {0.0,0.0,-120.0}
};

bool magnetOn[6] = {false, true, true, true, true, false};

int progress = 0;
#endif

void enableMotors()
{
  motor1.enableOutputs();
  motor2.enableOutputs();
  motor3.enableOutputs();

  motorsEnabled = true;
}

void disableMotors()
{
  motor1.disableOutputs();
  motor2.disableOutputs();
  motor3.disableOutputs();

  motorsEnabled = false;
}

void resetMotors(long backstep)
{
  if (backstep != 0) {
    motor1.moveTo(backstep); motor1.runToPosition();
    motor2.moveTo(backstep); motor2.runToPosition();
    motor3.moveTo(backstep); motor3.runToPosition();
  }
  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);
  motor3.setCurrentPosition(0);

  disableMotors();
  
#ifdef DEBUG
  float minX, minY, minZ;
  
  if (delta_calcForward(-20.0, -20.0, -20.0, minX, minY, minZ) == 0) {
    Serial.print(minX); Serial.print(" mm  ");
    Serial.print(minY); Serial.print(" mm  ");
    Serial.print(minZ); Serial.println(" mm");
  }
  
  Serial.println("Robot reset!");
#endif

  delay(1000);
}

void moveRobotTo(float positions[3]) {
  float angle1, angle2, angle3;
  long steps[3];

  // Ensure arms aren't set to an invalid physical position
  // (found empirically, more checks within the Kinematics)
  if (positions[2] > MAX_Z) positions[2] = MAX_Z;
  
  if (!motorsEnabled) enableMotors();
  
  if (delta_calcInverse(positions[0], positions[1], positions[2], angle1, angle2, angle3) == 0) {
    steps[0]  = (long)(angle1 * stepsPerDegree) + zeroOffsetSteps;
    steps[1]  = (long)(angle2 * stepsPerDegree) + zeroOffsetSteps;
    steps[2]  = (long)(angle3 * stepsPerDegree) + zeroOffsetSteps;

#ifdef DEBUG
    Serial.print("Angles: ");
    Serial.print(angle1); Serial.print("˚  ");
    Serial.print(angle2); Serial.print("˚  ");
    Serial.print(angle3); Serial.println("˚  ");
    
    Serial.print("Steps:  ");
    Serial.print(steps[0]); Serial.print("  ");
    Serial.print(steps[1]); Serial.print("  ");
    Serial.println(steps[2]);
#endif
    
    arm.moveTo(steps);
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(MAGNET_PIN, OUTPUT);
  
  // Change these to suit your stepper if you want
  motor1.setMaxSpeed(250);
  motor1.setAcceleration(500);
  arm.addStepper(motor1);

  motor2.setMaxSpeed(250);
  motor2.setAcceleration(500);
  arm.addStepper(motor2);
  
  motor3.setMaxSpeed(250);
  motor3.setAcceleration(500);
  arm.addStepper(motor3);

  resetMotors(-30);
  
#ifdef TEST_MOVES
  moveRobotTo(samplePositions[0]);
#endif

#ifdef DEBUG
  Serial.println("Ready");
#endif
}

#ifndef TEST_MOVES
bool parseInput()
{
  while(Serial.available() > 0)
  {
    char aChar = Serial.read();
    
    if(aChar == ';')
    {
      char *p = inData; //point to *p to the string in inData
      char *str;        //declaring *str
      int counter = 0; //initialise the counter
      
      while (str = strtok(p, ",")) // delimiter is the comma
      {
        outData[counter] = str; //use the counter as an index to add each value to the array
        counter++; //increment the counter
        
        p = NULL;
        
        if (counter > 3) break;
      }
      
 #ifdef DEBUG
     if (!strcmp(outData[0],"R")) {
        Serial.println("Resetting");
      } else {
        Serial.print("X: "); Serial.println(atof(outData[0]));
        Serial.print("Y: "); Serial.println(atof(outData[1]));
        Serial.print("Z: "); Serial.println(atof(outData[2]));
        Serial.print("M: "); Serial.println(atoi(outData[3]));
      }
 #endif
     if (!strcmp(outData[0],"R")) {
        resetMotors(-20);
      } else {
        float target[3];
        bool magnetOn;
        
        target[0] = atof(outData[0]);
        target[1] = atof(outData[1]);
        target[2] = atof(outData[2]);
        
        moveRobotTo(target);

        magnetOn = atoi(outData[3]) != 0;
        if (magnetOn) {
          digitalWrite(MAGNET_PIN, HIGH);
        } else {
          digitalWrite(MAGNET_PIN, LOW);
        }
        
        delay(500);
      }
      
      for (int ii=0; ii<4; ii++) {
        outData[ii] = NULL;
      }
      
      idx = 0;
      inData[idx] = NULL;
    } else if (idx < 80) {
      inData[idx] = aChar;
      idx++;
      inData[idx] = '\0'; // Keep the string NULL terminated
    }
  }
}
#endif  // TEST_MOVES

void loop()
{
#ifdef TEST_MOVES
  if (!arm.run()) {
    progress++;

    if (progress >= 6) {
      progress = 0;
      resetMotors(-20);
      enableMotors();
   }
    
    moveRobotTo(samplePositions[progress]);
    
    if (magnetOn[progress]) {
      digitalWrite(MAGNET_PIN, HIGH);
    } else {
      digitalWrite(MAGNET_PIN, LOW);
    }

    delay(500);
  }
#else

  if (!arm.run()) parseInput();
  
#endif
}
