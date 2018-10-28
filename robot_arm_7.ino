#include "Nextion.h"
#define rxPin 10
#define txPin 11 //for software serial
#include <Servo.h>
Servo myservo; //servo pos 100-175
#include <SoftwareSerial.h>
SoftwareSerial mySerial(rxPin, txPin);

const int stepPin1 = 3;
const int dirPin1 = 2;
const int stepPin2 = 5;
const int dirPin2 = 4;
const int stepPin3 = 7;
const int dirPin3 = 6;
int servoPos = 170;
int stepperArray[3][10] = {{0}};
int arrayPin;
int ledPin = 13;

bool stepperXup = false;
bool stepperXdown = false;
bool stepperYup = false;
bool stepperYdown = false;
bool stepperZup = false;
bool stepperZdown = false;
int xReset;
int yReset;
int zReset;


int xSteps;
int ySteps;
int zSteps;
int xStepSave;
int yStepSave;
int zStepSave;

long buttonTimer = 0;
long longPressTime = 1000;
bool b6Button = false;
float pressLength_milliSeconds = 0;
int optionOne_milliseconds = 100;
int optionTwo_milliseconds = 1000;


bool saveState = false;

bool save;
bool exec;
bool rec = false;

bool triggerX = false;
bool triggerY = false;
bool triggerZ = false;

NexButton b0 = NexButton(0, 1, "up");
NexButton b1 = NexButton(0, 2, "down");
NexButton b2 = NexButton(0, 3, "back");
NexButton b3 = NexButton(0, 4, "forward");
NexButton b4 = NexButton(0, 5, "right");
NexButton b5 = NexButton(0, 6, "left");

NexButton b6 = NexButton(0, 7, "save");
NexButton b7 = NexButton(0, 8, "exec");
NexButton b8 = NexButton(0, 9, "rec");

NexButton b9 = NexButton(0, 10, "1");
NexButton b10 = NexButton(0, 11, "2");
NexButton b11 = NexButton(0, 12, "3");

NexDSButton bt0 = NexDSButton(0, 13, "dual");

NexTouch *nex_listen_list[] = {
  &b0,
  &b1,
  &b2,
  &b3,
  &b4,
  &b5,
  &b6,
  &b7,
  &b8,
  &b9,
  &b10,
  &b11,
  &bt0,
  NULL
};

void setup() {
  myservo.attach(8);
  myservo.write(175);
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println("hardware serial OK");
  delay(50);

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(ledPin, OUTPUT);


  b0.attachPush(b0PushCallback);
  b1.attachPush(b1PushCallback);
  b2.attachPush(b2PushCallback);
  b3.attachPush(b3PushCallback);
  b4.attachPush(b4PushCallback);
  b5.attachPush(b5PushCallback);
  b6.attachPush(b6PushCallback);
  b7.attachPush(b7PushCallback);
  b8.attachPush(b8PushCallback);
  b9.attachPush(b9PushCallback);
  //  b10.attachPush(b10PushCallback);
  b11.attachPush(b11PushCallback);

  b0.attachPop(b0PopCallback);
  b1.attachPop(b1PopCallback);
  b2.attachPop(b2PopCallback);
  b3.attachPop(b3PopCallback);
  b4.attachPop(b4PopCallback);
  b5.attachPop(b5PopCallback);
  b6.attachPop(b6PopCallback);
  //  b7.attachPop(b7PopCallback);
  //  b8.attachPop(b8PopCallback);
  //  b9.attachPop(b9PopCallback);
  //  b10.attachPop(b10PopCallback);
  b11.attachPop(b11PopCallback);

}

void loop() {
  nexLoop(nex_listen_list); //Nextion display
  moveStepper();
  savePos();
  runSave();

  if (b6Button == true) {
    pressLength_milliSeconds = pressLength_milliSeconds + 100;
    delay(100);
    Serial.println("presslength active");
  }//long press:
  if (pressLength_milliSeconds >= optionTwo_milliseconds)
  {
    stepperArray[3][10] = {{0}};
    Serial.println("Memory deleted");
  }// short press:
  else if (pressLength_milliSeconds >= optionOne_milliseconds)
  {
    saveState = true;
    Serial.println("saving");
  }


pressLength_milliSeconds = 0;
}

void servoGrabber() {
  myservo.write(100);
  delay(500) ;
  myservo.write(170);
  delay(500);
}
// SYSTEM INFO            *********************
void b9PushCallback(void *ptr) {
  systemStatus();
}
void systemStatus() {
  Serial.println("-------------");
  Serial.println("SYSTEM STATUS");
  Serial.println("-------------");
  Serial.print("xSteps: ");
  Serial.println(xSteps);
  Serial.print("ySteps: ");
  Serial.println(ySteps);
  Serial.print("zSteps: ");
  Serial.println(zSteps);
  Serial.println("-------------");
  for ( int i = 0; i < 10; i++) {
    Serial.print("stepPin: ");
    Serial.print(stepperArray[0][i]);
    Serial.print("  ");
    Serial.print("StepSave: ");
    Serial.print("  ");
    Serial.println(stepperArray[1][i]);
  }
  Serial.println("-------------");

}
// STEPPER x opp/ned  *******************
void b0PushCallback(void *ptr) {
  stepperXup = true;
}

void b0PopCallback(void *ptr) {
  stepperXup = false;
  triggerX = true;
  Serial.print("xSteps høyre: ");
  Serial.println(xSteps);
}

void b1PushCallback(void *ptr) {
  stepperXdown = true;
}

void b1PopCallback(void *ptr) {
  stepperXdown = false;
  triggerX = true;
  Serial.print("xSteps venstre: ");
  Serial.println(xSteps);
}
void b11PushCallback(void *ptr) {
  servoGrabber();
}

void b11PopCallback(void *ptr) {

}

// STEPPER Y fram/tilbake  *******************
void b2PushCallback(void *ptr) {
  stepperYup = true;
}

void b2PopCallback(void *ptr) {
  stepperYup = false;
  triggerY = true;
  Serial.print("ySteps høyre: ");
  Serial.println(ySteps);
}

void b3PushCallback(void *ptr) {
  stepperYdown = true;
}

void b3PopCallback(void *ptr) {
  stepperYdown = false;
  triggerY = true;
  Serial.print("ySteps venstre: ");
  Serial.println(ySteps);
}

// STEPPER Z høyre/venstre  *******************
void b4PushCallback(void *ptr) {
  stepperZup = true;
}

void b4PopCallback(void *ptr) {
  stepperZup = false;
  triggerZ = true;
  Serial.print("zSteps høyre: ");
  Serial.println(zSteps);
}

void b5PushCallback(void *ptr) {
  stepperZdown = true;
}

void b5PopCallback(void *ptr) {
  stepperZdown = false;
  triggerZ = true;
  Serial.print("zSteps venstre: ");
  Serial.println(zSteps);
}

//SAVE                      ********************

void b6PushCallback(void *ptr) {
  //    saveState = true;
  b6Button = true;
}
void b6PopCallback(void *ptr) {
  Serial.println("Save");
  xReset = zSteps;
  yReset = zSteps;
  zReset = zSteps;
  b6Button = false;
}
//EXEC RESET                **********************
void b7PushCallback(void *ptr) {
  reset();
}
//REC
void b8PushCallback(void *ptr) {
  //resetSave();
  rec = true;
}

// SAVE POSITIONS TO ARRAY  ********************


void savePos() {
  if (saveState == true) {
    if (triggerX == true) {
      stepperArray[0][arrayPin] = stepPin1;
      stepperArray[2][arrayPin] = dirPin1;
      stepperArray[1][arrayPin] = xStepSave;

      triggerX = false;
      arrayPin++;
      xStepSave = 0;
    }

    if (triggerY == true) {
      stepperArray[0][arrayPin] = stepPin2;
      stepperArray[2][arrayPin] = dirPin2;
      stepperArray[1][arrayPin] = yStepSave;

      triggerY = false;
      arrayPin++;
      yStepSave = 0;
    }
    if (triggerZ == true) {
      stepperArray[0][arrayPin] = stepPin3;
      stepperArray[2][arrayPin] = dirPin3;
      stepperArray[1][arrayPin] = zStepSave;

      triggerZ = false;
      arrayPin++;
      zStepSave = 0;
    }
  }
}



// RUN SAVE POSITIONS       ********************
void runSave() {
  if (rec == true) {
    resetSave();
    delay(1000);
    for (int i = 0; i < 10; i++) { //stepper(int dirPin, int steps, int stepPin)
      if (stepperArray[0][i] != 0) {
        Serial.print(i + 1);
        Serial.print(" stepper no: ");
        Serial.print(stepperArray[0][i]);
        Serial.print(" ");
        Serial.println(stepperArray[1][i]);
      }
      stepper(stepperArray[2][i], stepperArray[1][i], stepperArray[0][i]);

      delay(10);
      if (stepperArray[0][i] == 0) {
        Serial.println("end of line");
        rec = false;
      }
    }
  }
}

//manual control from display *******************
void moveStepper() {
  if (stepperXup == true) {
    stepper(dirPin1, 1, stepPin1);
  }
  if (stepperXdown == true) {
    stepper(dirPin1, -1, stepPin1);
  }

  if (stepperYup == true) {
    stepper(dirPin2, 1, stepPin2);
  }
  if (stepperYdown == true) {
    stepper(dirPin2, -1, stepPin2);
  }
  if (stepperZup == true) {
    stepper(dirPin3, 1, stepPin3);
  }
  if (stepperZdown == true) {
    stepper(dirPin3, -1, stepPin3);
  }
}
//stepper function          *********************
void stepper(int dirPin, int steps, int stepPin) {
  int dir = steps > 0 ? HIGH : LOW;

  digitalWrite(dirPin, dir);
  if ( steps > 0 ) {
    for (int i = 0; i < steps; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(100);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1300);
      if (stepPin == 3) {
        xSteps++;
        if (saveState == true) {
          xStepSave++;
        }
      }
      if (stepPin == 5) {
        ySteps++;
        if (saveState == true) {
          yStepSave++;
        }
      }
      if (stepPin == 7) {
        zSteps++;
        if (saveState == true) {
          zStepSave++;
        }
      }
    }
  }
  if (steps < 0) {
    digitalWrite(dirPin, dir);
    for (int i = 0; i > steps; i--) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(100);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1300);
      if (stepPin == 3) {
        xSteps--;
        if (saveState == true) {
          xStepSave--;
        }
      }
      if (stepPin == 5) {
        ySteps--;
        if (saveState == true) {
          yStepSave--;
        }
      }
      if (stepPin == 7) {
        zSteps--;
        if (saveState == true) {
          zStepSave--;

        }
      }
    }
  }
}


// RESET                  *******************************
void reset() {
  stepper(dirPin1, xSteps * -1, stepPin1);
  stepper(dirPin2, ySteps * -1, stepPin2);
  stepper(dirPin3, zSteps * -1, stepPin3);
}

void resetSave() {
  //  stepper(dirPin3, zStepSave * -1, stepPin3);
  Serial.print("zReset: ");
  Serial.println(zReset);
  stepper(dirPin1, (xSteps - xReset) * -1, stepPin1);
  stepper(dirPin2, (ySteps - zReset) * -1, stepPin2);
  stepper(dirPin3, (zSteps - yReset) * -1, stepPin3);
}
