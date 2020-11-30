#include <HCSR04.h>
#include <MsTimer2.h>


#define trigPin1 11
#define echoPin1 10
#define trigPin2 43
#define echoPin2 42
#define trigPin3 28
#define echoPin3 29
#define trigPin4 30
#define echoPin4 31
#define trigPin5 32
#define echoPin5 33
//////////////////////////////////
#define in1 55
#define in2 56
#define ena 76

#define enal 11
#define inl1 12
#define inl2 14
//////////////////////////////
#define inr1 5
#define inr2 3
#define enar 2
/////////////////////////////


#define SYS_TICK_TIME 1
#define IDLE_STATE 0
#define DRIVE_STATE 1
#define TURN_STATE 2
#define RETURN_STATE 3
#define REVERSE_STATE 4
#define FRW_STATE 5
#define BK_STATE 6
#define LEFT_STATE 7
#define RIGHT_STATE 8
#define CAR_TSTOP 9
#define GO_LEFT_STATE 10
#define GO_RIGHT_STATE 11
#define LEFT_STATE_DELAY 12
#define RIGHT_STATE_DELAY 13


#define TURN_TIME 500/SYS_TICK_TIME
#define REVERS_TIME 2800 /SYS_TICK_TIME
#define CRUISE_POWER 240
#define TURN_POWER 250
#define MANUAL_DRIVE_POWER 20
#define CRUISE_POWERMAN 230

#define interval  100
/////////////////////////////////////timpul de rotatie a volnului in regim manual////////////////////////
#define SECOND_TURN 10
int putere;
int car_state = IDLE_STATE;
int turnTime = 0;
int reversTime = 0;
unsigned long previousMillis = 0;
unsigned long  currentMillis;
int a, b;
long RightSensor, BackSensor, FrontSensor, LeftSensor;
int GetSonarSensorValue(int trigPin, int echoPin);
///////////////////////
long RightSensor3, RightSensor4;
///////////////////////////
void CarStop(void) {
  digitalWrite(inl2, LOW);
  digitalWrite(inl1, LOW);
  analogWrite(enal, 0);

  digitalWrite(in2, LOW);
  digitalWrite(in1, LOW);
  analogWrite(ena, 0);
  /////////////////////////////////

}
void CarDrive(int pov) {
  digitalWrite(inl2, LOW);
  digitalWrite(inl1, HIGH);
  analogWrite(enal, pov);

  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
  analogWrite(ena, pov);
}

void CarReverse(int pov) {

  digitalWrite(inl2, HIGH);
  digitalWrite(inl1, LOW );
  analogWrite(enal, 250);

  digitalWrite(in2, HIGH );
  digitalWrite(in1, LOW);
  analogWrite(ena, 250);
}

void CarTLeft(int pov)
{
  digitalWrite(inr1, LOW);
  digitalWrite(inr2, HIGH);
  analogWrite(enar, pov);
  //Serial.println("____ENAR_LEFT___");
}
void CarTRight(int pov) {
  digitalWrite(inr1, HIGH);
  digitalWrite(inr2, LOW);
  analogWrite(enar, pov);
}
void CarTStop()
{
  digitalWrite(inr1, LOW);
  digitalWrite(inr2, LOW);
  analogWrite(enar, 0);
}
void carDrive(void) {
  const uint32_t turn_volan_time = 3000;
  static uint32_t timeLastTransition_volan_right = 0;
  static uint32_t  timeLastTransition_volan_left = 0;
  switch (car_state) {


    case IDLE_STATE:
      CarStop();
      CarTStop();
      break;
    case DRIVE_STATE:

      if (FrontSensor < 25 or LeftSensor < 25 or RightSensor < 25) {
        CarStop();
        car_state = TURN_STATE;
        turnTime = TURN_TIME;
      } else {
        CarDrive(CRUISE_POWER);
      }
      break;

    case TURN_STATE:
    //      if (--turnTime <= 0) {
    //
    //        CarTStop();
    //        car_state = REVERSE_STATE;
    //        reversTime = REVERS_TIME;
    //
    //      } else {
    //        CarTLeft(TURN_POWER);
    //      }
    //      break;

    case REVERSE_STATE:
      if (--reversTime <= 0)
      {
        CarStop();
        car_state = RETURN_STATE;
        turnTime = TURN_TIME;
      }
      else
      {
        CarReverse(CRUISE_POWER);
      }
      break;

    case RETURN_STATE:
      if (--turnTime <= 0) {

        CarTStop();
        car_state = DRIVE_STATE;


      } else {
        CarTRight(TURN_POWER);
      }
      break;
    //////////////////////////regim manual////////////////////////
    case FRW_STATE:

      CarDrive(CRUISE_POWERMAN);
      //Serial.println("+++HELO FROM FRW_STATE+++");
      break;

    case BK_STATE:


      CarReverse(CRUISE_POWERMAN);
      //Serial.println("+++HELO FROM FRW_STATE+++");
      break;

    case LEFT_STATE:
      CarTLeft(TURN_POWER);
      timeLastTransition_volan_left = millis();
      car_state = LEFT_STATE_DELAY;
      break;

    case LEFT_STATE_DELAY:
      if (millis() - timeLastTransition_volan_left >= turn_volan_time)
      {
        car_state = IDLE_STATE;
      }
      break;

    case RIGHT_STATE:
      CarTRight(TURN_POWER);
      timeLastTransition_volan_right = millis();
      car_state = RIGHT_STATE_DELAY;
      break;

    case RIGHT_STATE_DELAY:
      if (millis() - timeLastTransition_volan_right >= turn_volan_time)
      {
        car_state = IDLE_STATE;
      }
      break;

    case CAR_TSTOP:

      CarTStop();
      break;

    case GO_LEFT_STATE:
      CarTLeft(TURN_POWER);
      //Serial.println("++++++HELLO from LEFT STATE+++++");
      break;
    case GO_RIGHT_STATE:
      CarTRight(TURN_POWER);
      //Serial.println("+++++++HELLO _FROM RIGHT_STATE++++");
      break;




  }

}

// SYS TICK
void SysTick(void) {
  // run task 1
  // Scheduler
  //TaskScheduler();
  carDrive();

}

//HCSR04 sonic_1_front(trigPin1,echoPin1);
//HCSR04 sonic_2_front(trigPin2,echoPin2);
//HCSR04 sonic_3_front(trigPin3,echoPin3);
//HCSR04 sonic_4_front(trigPin4,echoPin4);
//HCSR04 sonic_5_front(trigPin5,echoPin5);
long vsonic_1_front,vsonic_2_front,vsonic_3_front,vsonic_4_front,vsonic_5_front;

void setup()
{
  
  Serial.begin (9600);
  ///////////////////////// rotile
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  pinMode(enal, OUTPUT);
  pinMode(inl1, OUTPUT);
  pinMode(inl2, OUTPUT);

  pinMode(inr1, OUTPUT);
  pinMode(inr2, OUTPUT);
  pinMode(enar, OUTPUT);
  

  MsTimer2::set(SYS_TICK_TIME, SysTick); // 1ms period
  MsTimer2::start();



}

void loop() {
//vsonic_1_front = sonic_1_front.dist();
//vsonic_2_front = sonic_2_front.dist();
//vsonic_3_front = sonic_3_front.dist();
//vsonic_4_front = sonic_4_front.dist();
//vsonic_5_front = sonic_5_front.dist();


  ////////////////////////////////////////
  currentMillis = millis();
  Serial.print(vsonic_1_front);
  Serial.print(" - ");
  Serial.print(vsonic_2_front);
  Serial.print(" - ");
  Serial.print(vsonic_3_front);
  Serial.print(" - ");
  Serial.print(vsonic_4_front);
  Serial.print(" - ");
  Serial.print(vsonic_5_front);
  Serial.println();
  Serial.print("________STATE_ACTUAL->");
  Serial.println(car_state);
  char cmd;
  if (Serial.available()) {
    cmd = Serial.read();

    switch (cmd) {
      case 'f':
        car_state = DRIVE_STATE;
        break;

      case 'S':
        car_state = IDLE_STATE;
        Serial.println("ati tastat i");
        break;
      ////////////////////////////////////////////manual_STATE///////////////////////////
      case 'F':
        Serial.println("========F========");
        car_state = FRW_STATE;
        break;
      case 'B':
        Serial.println("========B========");
        car_state = BK_STATE;
        break;
      case 'L':
        Serial.println("========L========");

        car_state = LEFT_STATE;
        //CarTLeft(TURN_POWER);
        break;
      case 'R':
        Serial.println("========D========");

        car_state = RIGHT_STATE;
        //CarTRight(TURN_POWER);
        break;

    }
  }
  delay(100);

}

int  SonarSensor(int trigPin, int echoPin)
{ long duration;
  int distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;
  return distance;
}
