#include <Arduino.h>
#include <TimerOne.h>
#include <MsTimer2.h>

/*


  This sketch uses the TimerOne library downloadable from here:
  http://code.google.com/p/arduino-timerone/downloads/detail?name=TimerOne-v9.zip&can=2&q=

  Install the library using the following guide:
  http://arduino.cc/en/Guide/Libraries

  Please read Information about how works with interrupt on arduino by clicking this link
  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
*/

#include <TimerOne.h> // Header file for TimerOne library
/*
   =====================================ULTRASONIC_SENSORS_DEFINE_BLOCK====================================
*/
#define trigPinCommon 12 // Pin 12 trigger output
#define sechoPin 2       // Pin 2 Echo input
#define sechoPin2 3      // Pin 2 Echo input
#define sechoPin3 18     // Pin 2 Echo input
#define sechoPin4 19     // Pin 2 Echo input
#define sechoPin5 20     // Pin 2 Echo input

#define ISR_ID_echoPin1 0 //ISR_ID_FOR_EACH ECHO_PIN read more   https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
#define ISR_ID_echoPin2 1
#define ISR_ID_echoPin3 2
#define ISR_ID_echoPin4 3
#define ISR_ID_echoPin5 4

#define onBoardLED 13    // Pin 13 onboard LED
#define echo_int 0       // Interrupt id for echo pulse
#define TIMER_US 50      // 50 uS timer duration
#define TICK_COUNTS 4000 // 200 mS worth of timer ticks

volatile long echo_start = 0; // Records start of echo pulse
volatile long echo_end = 0;   // Records end of echo pulse

volatile long echo_duration = 0;  // Duration - difference between end and start
volatile long echo2_duration = 0; // Duration - difference between end and start
volatile long echo3_duration = 0; // Duration - difference between end and start
volatile long echo4_duration = 0; // Duration - difference between end and start
volatile long echo5_duration = 0; // Duration - difference between end and start

volatile int trigger_time_count = 0;     // Count down counter to trigger pulse time
volatile long range_flasher_counter = 0; // Count down counter for flashing distance LED
int sonic_state = 0;
long vsonic_1_front, vsonic_2_front, vsonic_3_front, vsonic_4_front, vsonic_5_front;
/*
   ==========================================================================================================================
*/

/*
   -----------------------------------------DEFINIREA_PINURILOR_DRIVERE_MOTOARE-----------------------------
*/
#define ena 28   //enable
#define in1 4
#define in2 5
#define DISTANCE_BREAK_VALUE 40
#define enal 36   //enable
#define inl1 6
#define inl2 7
//////////////////////////////
#define enar 50    // enable ruli
#define inr1 8
#define inr2 9  
/////////////////////////////
/*
   ---------------------------------------------------------------------------------------------------------
*/

/*
   -----------------------------------DEFINIREA_STARILOR_PENTRU_STATE_MACHINE--------------------------------
*/
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
#define SET_VOLAN_INAINTE 14
#define SET_VOLAN_INAINTE_DELAY 15

#define AVOID_OBSTACLE_START_STATE 16
#define AVOID_OBSTACLE_FIND_OBTSACLE_STATE 17
#define AVOID_OBSTACLE_DIRECTION 18
#define AVOID_OBSTACLE_COTIRE_DELAY 19
#define AVOID_OBSTACLE_COTIRE_DELAY_STOP 20
#define AVOID_OBSTACLE_IESI_DIN_COTITURA_STANGA_VOLAN 21
#define AVOID_OBSTACLE_IESI_DIN_COTITURA_DREAPTA_VOLAN 22
#define AVOID_OBSTACLE_INDREAPTA_ROTILE_INAINTE 23
#define AVOID_OBSTACLE_RETURN_BASE_WAY_GO_LEFT 24
#define AVOID_OBSTACLE_RETURN_BASE_WAY_GO_RIGHT 25
#define AVOID_OBSTACLE_RETURN_BASE_WAY_GO_INAINTE 26
#define AVOID_OBSTACLE_RETURN_BASE_WAY_DONE 27
/*
   ---------------------------------------------------------------------------------------------------------
*/

/*
   --------------------------------------VARIABILILE_ADAUGATOARE_PENTRU_STATE_MACHINE-----------------------
*/
#define TURN_TIME 500 / SYS_TICK_TIME
#define REVERS_TIME 2800 / SYS_TICK_TIME
#define CRUISE_POWER 140
#define TURN_POWER 250
#define MANUAL_DRIVE_POWER 20
#define CRUISE_POWERMAN 230
#define interval 100
#define SECOND_TURN 10
/*
   ---------------------------------------------------------------------------------------------------------
*/

int putere;
int car_state = IDLE_STATE;
int turnTime = 0;
int reversTime = 0;
unsigned long previousMillis = 0;
unsigned long currentMillis;
int a, b;
//VAriabilile pentru Rotirea Volanului                                         //==============ROTIREA_VOLANULUI================
int time_switch_position_volan = 500;
#define VOLAN_INAINTE 1
#define VOLAN_LEFT 2
#define VOLAN_RIGHT 3
#define UN 0
int actual_position_volan = UN;
//Variabilile pentru tusk: AVOID_OBSTACLE                                       //===========AVOID_OBSTACLE====================
#define AVOID_OBSTACLE_DIRECTION_VOLAN_RIGHT 3
#define AVOID_OBSTACLE_DIRECTION_VOLAN_LEFT 2

int avoid_obstacle_direction_volan_var = AVOID_OBSTACLE_DIRECTION_VOLAN_RIGHT; //======================SE POATE DE SCHIMBAT DIRECTIA COTIRII===========

int manual_speed = 80;
int volan_state = 0;

int volan_direction_to_go = 0; // 1 -este in centru , 3 -spre dreapta, 2 -spre stanga

////////////////////////////////////////////////////////////////////////////////////////////TIMP_AVOID_OBSTACLE///////////////////////////////////////////////////////////////
#define PRIMA_COTIRE_TIME 200
#define TIME_MERGI_INAINTE 1200
////////////////////////////////////////////////////////////////////////////////////////////TIMP_AVOID_OBSTACLE///////////////////////////////////////////////////////////////

/*
   -------------------------------------------DECLARATION_APRINDEREA_FARURILOR_TASK------------------------------------
*/
#define LIGHT_SENSOR_PIN A0         //pinul A0(ANALOG_INPUT)
#define TURN_OFF_FARURI_VALUE 200   //valoarea de la senzor
#define RELEU_CONTROL_SIGNAL_PIN 45 //pinul

//INFORMATIE DESPRE UN TUTORIAL  https://create.arduino.cc/projecthub/iasonas-christoulakis/set-up-a-5v-relay-on-the-arduino-0b37ff
/*
   ----------------------------------------------------------------------------------------------------------------------
*/

/*
   =================================================DEFINIREA_PROTOTIPURILOR_FUNCTIILOR===================================
*/
void CarTLeft_ALGO(int pov);
void CarStop(void);
void CarDrive(int pov);
void CarReverse(int pov);
void CarTLeft(int pov);
void CarTRight(int pov);
void CarTStop();
void ultrasonic_pind_init();
void ultrasonic_interrupt_attach();

int SonarSensor(int trigPin, int echoPin);
void get_ultrasonic_value_read();

void check_light_value();
void control_volan(int directia);
void trigger_pulse();
void echo_interrupt();
void distance_flasher();
void CarTRight_ALGO(int pov);
/*
  =======================================================================================================================
*/

void carDrive(void)
{

  const uint32_t turn_volan_time = 900; //==============================VERIFICAT_IN_REALITATE===================
  static uint32_t timeLastTransition_volan_right = 0;
  static uint32_t timeLastTransition_volan_left = 0;

  static uint32_t timeLastTransition_volan_front = 0;
  switch (car_state)
  {

  case IDLE_STATE:
    CarStop();
    CarTStop();
    break;
  case DRIVE_STATE:

    if (vsonic_1_front < 40 || vsonic_3_front < 40 || vsonic_4_front < 40 || vsonic_5_front < 40 || vsonic_2_front<40)
    {
      //Serial.println("mai mic<=*");
      CarStop();
      //car_state = TURN_STATE;
      //turnTime = TURN_TIME;
    }
    else 
    {
      CarDrive(CRUISE_POWER);
      //Serial.println("mai mare");
    }
    break;

  case TURN_STATE:
         if (--turnTime <= 0) {
    
           CarTStop();
           car_state = DRIVE_STATE;
           reversTime = REVERS_TIME;
    
         } else {
           CarTLeft(TURN_POWER);
         }
         break;

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
    if (--turnTime <= 0)
    {

      CarTStop();
      car_state = DRIVE_STATE;
    }
    else
    {
      CarTRight(TURN_POWER);
    }
    break;
  //////////////////////////regim manual////////////////////////
  case FRW_STATE:

    CarDrive(manual_speed);
    //Serial.println("+++HELO FROM FRW_STATE+++");
    break;

  case BK_STATE:

    CarReverse(manual_speed);
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
    break;
  case GO_RIGHT_STATE:
    CarTRight(TURN_POWER);
    break;
  case SET_VOLAN_INAINTE:
    if (actual_position_volan == VOLAN_LEFT)
    {
      volan_direction_to_go = VOLAN_RIGHT;
    }
    else if (actual_position_volan == VOLAN_RIGHT)
    {
      volan_direction_to_go = VOLAN_LEFT;
    }
    timeLastTransition_volan_front = millis();
    car_state = SET_VOLAN_INAINTE_DELAY;
    break;

  case SET_VOLAN_INAINTE_DELAY:
    actual_position_volan = VOLAN_INAINTE;
    if (volan_direction_to_go == VOLAN_LEFT)
    {
      CarTRight_ALGO(240);
    }
    else if (volan_direction_to_go == VOLAN_RIGHT)
    {
      CarTLeft_ALGO(240);
    }
    if (millis() - timeLastTransition_volan_front >= turn_volan_time / 2) //==============================TREBUIE DE IMPARTIT LA 2==============
    {
      car_state = IDLE_STATE;
    }

    break;

  case AVOID_OBSTACLE_START_STATE: //================AVOID_OBSTACLE_START===================== //1
    CarDrive(CRUISE_POWER);
    car_state = AVOID_OBSTACLE_FIND_OBTSACLE_STATE;
    break;

  case AVOID_OBSTACLE_FIND_OBTSACLE_STATE: //================ //2
    if (vsonic_1_front < DISTANCE_BREAK_VALUE or vsonic_2_front < DISTANCE_BREAK_VALUE or vsonic_3_front < DISTANCE_BREAK_VALUE or vsonic_4_front < DISTANCE_BREAK_VALUE or vsonic_5_front < DISTANCE_BREAK_VALUE)
    {
      car_state = AVOID_OBSTACLE_FIND_OBTSACLE_STATE;
      CarStop();
    }
    else
    {CarDrive(CRUISE_POWER);
      if (vsonic_1_front < 80 or vsonic_2_front < 80 or vsonic_3_front < 80 or vsonic_4_front < 80 or vsonic_5_front < 80)
      {
        car_state = AVOID_OBSTACLE_DIRECTION;
      }
    }
    break;

  case AVOID_OBSTACLE_DIRECTION: //3
    if (vsonic_1_front < DISTANCE_BREAK_VALUE or vsonic_2_front < DISTANCE_BREAK_VALUE or vsonic_3_front < DISTANCE_BREAK_VALUE or vsonic_4_front < DISTANCE_BREAK_VALUE or vsonic_5_front < DISTANCE_BREAK_VALUE)
    {
      car_state = AVOID_OBSTACLE_DIRECTION;
      CarStop();
    }
    else
    {CarDrive(CRUISE_POWER);
      if (avoid_obstacle_direction_volan_var == AVOID_OBSTACLE_DIRECTION_VOLAN_LEFT)
      {
        CarTLeft_ALGO(240);
      }
      else if (avoid_obstacle_direction_volan_var == AVOID_OBSTACLE_DIRECTION_VOLAN_RIGHT)
      {
        CarTRight_ALGO(240);
      }
      timeLastTransition_volan_front = millis();
      car_state = AVOID_OBSTACLE_COTIRE_DELAY;
    }

    break;
  case AVOID_OBSTACLE_COTIRE_DELAY: //4
   if (vsonic_1_front < DISTANCE_BREAK_VALUE or vsonic_2_front < DISTANCE_BREAK_VALUE or vsonic_3_front < DISTANCE_BREAK_VALUE or vsonic_4_front < DISTANCE_BREAK_VALUE or vsonic_5_front < DISTANCE_BREAK_VALUE)
    {
      car_state = AVOID_OBSTACLE_DIRECTION;
      CarStop();
    }
    else
    {CarDrive(CRUISE_POWER);
      if (millis() - timeLastTransition_volan_front >= turn_volan_time + PRIMA_COTIRE_TIME) //==============================TREBUIE DE IMPARTIT LA 2==============//experimentat prima cotire
      {
        CarTStop();
        car_state = AVOID_OBSTACLE_COTIRE_DELAY_STOP;
      }
    }
    break;
  case AVOID_OBSTACLE_COTIRE_DELAY_STOP:
     if (vsonic_1_front < DISTANCE_BREAK_VALUE or vsonic_2_front < DISTANCE_BREAK_VALUE or vsonic_3_front < DISTANCE_BREAK_VALUE or vsonic_4_front < DISTANCE_BREAK_VALUE or vsonic_5_front < DISTANCE_BREAK_VALUE)
    {
      car_state = AVOID_OBSTACLE_DIRECTION;
      CarStop();
    }
    else
    {CarDrive(CRUISE_POWER);
      if (avoid_obstacle_direction_volan_var == AVOID_OBSTACLE_DIRECTION_VOLAN_LEFT)
      {
        CarTRight_ALGO(240);
        //avoid_obstacle_direction_volan_var = AVOID_OBSTACLE_DIRECTION_VOLAN_RIGHT;
      }
      else if (avoid_obstacle_direction_volan_var == AVOID_OBSTACLE_DIRECTION_VOLAN_RIGHT)
      {
        CarTLeft_ALGO(240);
        //avoid_obstacle_direction_volan_var = AVOID_OBSTACLE_DIRECTION_VOLAN_LEFT;
      }
      timeLastTransition_volan_front = millis();
      car_state = AVOID_OBSTACLE_IESI_DIN_COTITURA_STANGA_VOLAN;
    }

    break;

  case AVOID_OBSTACLE_IESI_DIN_COTITURA_STANGA_VOLAN:

     if (vsonic_1_front < DISTANCE_BREAK_VALUE or vsonic_2_front < DISTANCE_BREAK_VALUE or vsonic_3_front < DISTANCE_BREAK_VALUE or vsonic_4_front < DISTANCE_BREAK_VALUE or vsonic_5_front < DISTANCE_BREAK_VALUE)
    {
      car_state = AVOID_OBSTACLE_DIRECTION;
      CarStop();
    }
    else
    {CarDrive(CRUISE_POWER);
      if (millis() - timeLastTransition_volan_front >= turn_volan_time + 1500) //==============================START_INDREAPTA_ROTILE_INAINTE============== //scoatere din zanos din stanga 
      {
        if (avoid_obstacle_direction_volan_var == AVOID_OBSTACLE_DIRECTION_VOLAN_LEFT)
        {

          CarTLeft_ALGO(240);
          //avoid_obstacle_direction_volan_var = AVOID_OBSTACLE_DIRECTION_VOLAN_RIGHT;
        }
        else if (avoid_obstacle_direction_volan_var == AVOID_OBSTACLE_DIRECTION_VOLAN_RIGHT)
        {
          CarTRight_ALGO(240);
          //avoid_obstacle_direction_volan_var = AVOID_OBSTACLE_DIRECTION_VOLAN_LEFT;
        }

        timeLastTransition_volan_front = millis();
        car_state = AVOID_OBSTACLE_INDREAPTA_ROTILE_INAINTE;
      } ////////////////////////////////////////////////////////////
    }

    /////////////////MASINAM MERGE DREPT INAINTE
    break;
  case AVOID_OBSTACLE_INDREAPTA_ROTILE_INAINTE:
    if (vsonic_1_front < DISTANCE_BREAK_VALUE or vsonic_2_front < DISTANCE_BREAK_VALUE or vsonic_3_front < DISTANCE_BREAK_VALUE or vsonic_4_front < DISTANCE_BREAK_VALUE or vsonic_5_front < DISTANCE_BREAK_VALUE)
    {
      car_state = AVOID_OBSTACLE_DIRECTION;
      CarStop();
    }
    else
    {CarDrive(CRUISE_POWER);
      if (millis() - timeLastTransition_volan_front >= turn_volan_time /2) //==============================STOP_INDREAPTA_ROTILE_INAINTE============== //#volan_inainte
      {
        CarTStop();
        timeLastTransition_volan_front = millis();
        car_state = AVOID_OBSTACLE_RETURN_BASE_WAY_GO_LEFT;
      }
    }

    break;
  case AVOID_OBSTACLE_RETURN_BASE_WAY_GO_LEFT: //iesire din cotitura
    if (vsonic_1_front < DISTANCE_BREAK_VALUE or vsonic_2_front < DISTANCE_BREAK_VALUE or vsonic_3_front < DISTANCE_BREAK_VALUE or vsonic_4_front < DISTANCE_BREAK_VALUE or vsonic_5_front < DISTANCE_BREAK_VALUE)
    {
      car_state = AVOID_OBSTACLE_DIRECTION;
      CarStop();
    }
    else
    {CarDrive(CRUISE_POWER);
      if (millis() - timeLastTransition_volan_front >= turn_volan_time+TIME_MERGI_INAINTE ) //==============================START_MERGE_INAINTE=============#merge inainte
      {
        if (avoid_obstacle_direction_volan_var == AVOID_OBSTACLE_DIRECTION_VOLAN_LEFT)
        {
          CarTRight_ALGO(240);
          //avoid_obstacle_direction_volan_var = AVOID_OBSTACLE_DIRECTION_VOLAN_RIGHT;
        }
        else if (avoid_obstacle_direction_volan_var == AVOID_OBSTACLE_DIRECTION_VOLAN_RIGHT)
        {
          CarTLeft_ALGO(240);
          //avoid_obstacle_direction_volan_var = AVOID_OBSTACLE_DIRECTION_VOLAN_LEFT;
        }

        timeLastTransition_volan_front = millis();
        car_state = AVOID_OBSTACLE_RETURN_BASE_WAY_GO_RIGHT;
      } /////////////////////////////////////////////IESIRE DIN COTITURA SPRE BANDA DE BAZA LA STANGA
    }

    break;
  case AVOID_OBSTACLE_RETURN_BASE_WAY_GO_RIGHT:
    if (vsonic_1_front < DISTANCE_BREAK_VALUE or vsonic_2_front < DISTANCE_BREAK_VALUE or vsonic_3_front < DISTANCE_BREAK_VALUE or vsonic_4_front < DISTANCE_BREAK_VALUE or vsonic_5_front < DISTANCE_BREAK_VALUE)
    {
      car_state = AVOID_OBSTACLE_DIRECTION;
      CarStop();
    }
    else
    {CarDrive(CRUISE_POWER);
      if (millis() - timeLastTransition_volan_front >= turn_volan_time+PRIMA_COTIRE_TIME ) //==============================START_RETURN_ROTILE_DREAPTA==============
      {
        if (avoid_obstacle_direction_volan_var == AVOID_OBSTACLE_DIRECTION_VOLAN_LEFT)
        {
          CarTLeft_ALGO(240);
          //avoid_obstacle_direction_volan_var = AVOID_OBSTACLE_DIRECTION_VOLAN_RIGHT;
        }
        else if (avoid_obstacle_direction_volan_var == AVOID_OBSTACLE_DIRECTION_VOLAN_RIGHT)
        {
          CarTRight_ALGO(240);

          //avoid_obstacle_direction_volan_var = AVOID_OBSTACLE_DIRECTION_VOLAN_LEFT;
        }
        CarTRight(240);
        timeLastTransition_volan_front = millis();
        car_state = AVOID_OBSTACLE_RETURN_BASE_WAY_GO_INAINTE;
      }
    }

    break; ////////////////////////////////////////ROTESTE ROTILE DE La STANGA SPRE DREAPTA SCOATE MASINA DIN CURBA

  case AVOID_OBSTACLE_RETURN_BASE_WAY_GO_INAINTE:
   if (vsonic_1_front < DISTANCE_BREAK_VALUE or vsonic_2_front < DISTANCE_BREAK_VALUE or vsonic_3_front < DISTANCE_BREAK_VALUE or vsonic_4_front < DISTANCE_BREAK_VALUE or vsonic_5_front < DISTANCE_BREAK_VALUE)
    {
      car_state = AVOID_OBSTACLE_DIRECTION;
      CarStop();
    }
    else
    {CarDrive(CRUISE_POWER);
      if (millis() - timeLastTransition_volan_front >= turn_volan_time * 2) //==============================START_RETURN_ROTILE_INAINTE==============
      {
        if (avoid_obstacle_direction_volan_var == AVOID_OBSTACLE_DIRECTION_VOLAN_LEFT)
        {
          CarTRight_ALGO(240);
          //avoid_obstacle_direction_volan_var = AVOID_OBSTACLE_DIRECTION_VOLAN_RIGHT;
        }
        else if (avoid_obstacle_direction_volan_var == AVOID_OBSTACLE_DIRECTION_VOLAN_RIGHT)
        {
          CarTLeft_ALGO(240);

          //avoid_obstacle_direction_volan_var = AVOID_OBSTACLE_DIRECTION_VOLAN_LEFT;
        }
        timeLastTransition_volan_front = millis();
        car_state = AVOID_OBSTACLE_RETURN_BASE_WAY_DONE;
      }
    }

    break;
  case AVOID_OBSTACLE_RETURN_BASE_WAY_DONE:
     if (vsonic_1_front < DISTANCE_BREAK_VALUE or vsonic_2_front < DISTANCE_BREAK_VALUE or vsonic_3_front < DISTANCE_BREAK_VALUE or vsonic_4_front < DISTANCE_BREAK_VALUE or vsonic_5_front < DISTANCE_BREAK_VALUE)
    {
      car_state = AVOID_OBSTACLE_DIRECTION;
      CarStop();
    }
    else
    {CarDrive(CRUISE_POWER);
      if (millis() - timeLastTransition_volan_front >= turn_volan_time / 2) //==============================STOP_RETURN_ROTILE_INAINTE==============
      {
        CarTStop();

        car_state = AVOID_OBSTACLE_START_STATE;
      } //////////////////////////////ROTILE SUNT INAINTE SI MASINA MERGE INAINTE
    }

    break;
  }
}

/*
   ========================================ISR_APEL================================
*/

// SYS TICK
void SysTick(void)
{
  carDrive();
}
/*
   ==================================================================================
*/

void setup()
{

  //configurarea  pinurilor la care sunt atasate driverelel motoarelor
  pinMode(ena, OUTPUT); //right roata
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enal, OUTPUT); //left roata
  pinMode(inl1, OUTPUT);
  pinMode(inl2, OUTPUT);

  pinMode(inr1, OUTPUT); //volan
  pinMode(inr2, OUTPUT);
  pinMode(enar, OUTPUT);

  //Configurarea pinului pentru aprinderea/stingerea releului in dependenta de luminozitate
  pinMode(RELEU_CONTROL_SIGNAL_PIN, OUTPUT);

  //Ultrasoni_sensors_init_pin_attached_interrupt
  pinMode(onBoardLED, OUTPUT); // Onboard LED pin set to output
  ultrasonic_pind_init();
  ultrasonic_interrupt_attach();

  //Setarea timerului pentru  apelul la fiecare 1ms a functiei Car_Drive(state_machine)
  MsTimer2::set(SYS_TICK_TIME, SysTick); // 1ms period
  MsTimer2::start();
  Serial.begin(9600);
}

void loop()
{

  check_light_value();

  get_ultrasonic_value_read();
  //Serial.println(echo_duration / 58);
  Serial.print("__STATE_ACTUAL->");
  Serial.print(car_state);
  Serial.print("__SPEED->");
  Serial.print(manual_speed);
  Serial.print("__VOLAN_POSITION->");
  Serial.println(actual_position_volan);
  char cmd;
  if (Serial.available())
  {
    cmd = Serial.read();

    switch (cmd)
    {
    case 'W':
      car_state = DRIVE_STATE;
      break;

    case 'S':
      car_state = IDLE_STATE;
      Serial.println("ati tastat i");
      break;
    ////////////////////////////////////////////manual_STATE///////////////////////////
    case 'F':
      Serial.println("========F========");
      //CarDrive(manual_speed);
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
      Serial.println("========R========");

      car_state = RIGHT_STATE;
      //CarTRight(TURN_POWER);
      break;

    case '1':
      manual_speed = 120;
      break;
    case '2':
      manual_speed = 150;
      break;
    case '3':
      manual_speed = 180;
      break;
    case '4':
      manual_speed = 210;
      break;
    case '5':
      manual_speed = 230;
      break;
    case '6':
      manual_speed = 250;
      break;
    case 'l': // 'W" mare
      car_state = SET_VOLAN_INAINTE;
      Serial.println("========W========");
      break;
    case 'U': //'w' mic
      car_state = AVOID_OBSTACLE_START_STATE;
      break;
    }
  }
  delay(150);
}
/*
   ===========================================================FUNCTION_ULTRASONIC_USING_TIMMERS_INTERRUPT======================================
*/
// --------------------------
// timerIsr() 50uS second interrupt ISR()
// Called every time the hardware timer 1 times out.
// --------------------------
void timerIsr()
{
  trigger_pulse();    // Schedule the trigger pulses
  distance_flasher(); // Flash the onboard LED distance indicator
}

// --------------------------
// trigger_pulse() called every 50 uS to schedule trigger pulses.
// Generates a pulse one timer tick long.
// Minimum trigger pulse width for the HC-SR04 is 10 us. This system
// delivers a 50 uS pulse.
// --------------------------
void trigger_pulse()
{
  static volatile int state = 0; // State machine variable

  if (!(--trigger_time_count))        // Count to 200mS
  {                                   // Time out - Initiate trigger pulse
    trigger_time_count = TICK_COUNTS; // Reload
    state = 1;                        // Changing to state 1 initiates a pulse
  }

  switch (state) // State machine handles delivery of trigger pulse
  {
  case 0: // Normal state does nothing
    break;

  case 1:                              // Initiate pulse
    digitalWrite(trigPinCommon, HIGH); // Set the trigger output high
    state = 2;                         // and set state to 2
    break;

  case 2: // Complete the pulse
  default:
    digitalWrite(trigPinCommon, LOW); // Set the trigger output low
    state = 0;                        // and return state to normal 0
    break;
  }
}

// --------------------------
// echo_interrupt() External interrupt from HC-SR04 echo signal.
// Called every time the echo signal changes state.
//
// Note: this routine does not handle the case where the timer
//       counter overflows which will result in the occassional error.
// --------------------------
void echo1_interrupt()
{
  switch (digitalRead(sechoPin)) // Test to see if the signal is high or low
  {
  case HIGH:               // High so must be the start of the echo pulse
    echo_end = 0;          // Clear the end time
    echo_start = micros(); // Save the start time
    break;

  case LOW:                                // Low so must be the end of hte echo pulse
    echo_end = micros();                   // Save the end time
    echo_duration = echo_end - echo_start; // Calculate the pulse duration
    break;
  }
}
void echo2_interrupt()
{
  switch (digitalRead(sechoPin2)) // Test to see if the signal is high or low
  {
  case HIGH:               // High so must be the start of the echo pulse
    echo_end = 0;          // Clear the end time
    echo_start = micros(); // Save the start time
    break;

  case LOW:                                 // Low so must be the end of hte echo pulse
    echo_end = micros();                    // Save the end time
    echo2_duration = echo_end - echo_start; // Calculate the pulse duration
    break;
  }
}

void echo3_interrupt()
{
  switch (digitalRead(sechoPin3)) // Test to see if the signal is high or low
  {
  case HIGH:               // High so must be the start of the echo pulse
    echo_end = 0;          // Clear the end time
    echo_start = micros(); // Save the start time
    break;

  case LOW:                                 // Low so must be the end of hte echo pulse
    echo_end = micros();                    // Save the end time
    echo3_duration = echo_end - echo_start; // Calculate the pulse duration
    break;
  }
}

void echo4_interrupt()
{
  switch (digitalRead(sechoPin4)) // Test to see if the signal is high or low
  {
  case HIGH:               // High so must be the start of the echo pulse
    echo_end = 0;          // Clear the end time
    echo_start = micros(); // Save the start time
    break;

  case LOW:                                 // Low so must be the end of hte echo pulse
    echo_end = micros();                    // Save the end time
    echo4_duration = echo_end - echo_start; // Calculate the pulse duration
    break;
  }
}
void echo5_interrupt()
{
  switch (digitalRead(sechoPin5)) // Test to see if the signal is high or low
  {
  case HIGH:               // High so must be the start of the echo pulse
    echo_end = 0;          // Clear the end time
    echo_start = micros(); // Save the start time
    break;

  case LOW:                                 // Low so must be the end of hte echo pulse
    echo_end = micros();                    // Save the end time
    echo5_duration = echo_end - echo_start; // Calculate the pulse duration
    break;
  }
}

void distance_flasher()
{
  if (--range_flasher_counter <= 0) // Decrement and test the flash timer
  {                                 // Flash timer time out
    if (echo_duration < 25000)      // If the echo duration is within limits
    {
      range_flasher_counter = echo_duration * 2; // Reload the timer with the current echo duration
    }
    else
    {
      range_flasher_counter = 25000; // If out of range use a default
    }

    digitalWrite(onBoardLED, digitalRead(onBoardLED) ^ 1); // Toggle the onboard LED
  }
}

void ultrasonic_interrupt_attach()
{
  Timer1.initialize(TIMER_US);                        // Initialise timer 1
  Timer1.attachInterrupt( timerIsr );                 // Attach interrupt to the timer service routine
  attachInterrupt(digitalPinToInterrupt(sechoPin), echo1_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
  attachInterrupt(digitalPinToInterrupt(sechoPin2), echo2_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
  attachInterrupt(digitalPinToInterrupt(sechoPin3), echo3_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
  attachInterrupt(digitalPinToInterrupt(sechoPin4), echo4_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
  attachInterrupt(digitalPinToInterrupt(sechoPin5), echo5_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
}

void ultrasonic_pind_init()
{
  pinMode(trigPinCommon, OUTPUT); // Trigger pin set to output
  pinMode(sechoPin, INPUT);       // Echo pin set to input
  pinMode(sechoPin2, INPUT);
  pinMode(sechoPin3, INPUT); // Echo pin set to input
  pinMode(sechoPin4, INPUT);
  pinMode(sechoPin5, INPUT);
}

/*
   =====================================================================================================================================
*/

void get_ultrasonic_value_read()
{
   vsonic_1_front = (echo_duration / 58);
  vsonic_2_front = (echo2_duration / 58);
   vsonic_3_front = (echo3_duration / 58);
    vsonic_4_front = (echo4_duration / 58);
   vsonic_5_front = (echo5_duration / 58);
//  vsonic_1_front = (echo_duration / 58);
//  vsonic_2_front = 38;
//  vsonic_3_front = 38;
//  vsonic_4_front = 38;
//  vsonic_5_front = 38;
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
}
int SonarSensor(int trigPin, int echoPin)
{
  long duration;
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

void CarStop(void)
{
  digitalWrite(inl2, LOW);
  digitalWrite(inl1, LOW);
  analogWrite(enal, 0);

  digitalWrite(in2, LOW);
  digitalWrite(in1, LOW);
  analogWrite(ena, 0);
  /////////////////////////////////
}
void CarDrive(int pov)
{
  digitalWrite(inl2, LOW);
  digitalWrite(inl1, HIGH);
  analogWrite(enal, pov);

  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
  analogWrite(ena, pov);
}

void CarReverse(int pov)
{

  digitalWrite(inl2, HIGH);
  digitalWrite(inl1, LOW);
  analogWrite(enal, 250);

  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
  analogWrite(ena, 250);
}

void CarTLeft(int pov)
{
  actual_position_volan = VOLAN_LEFT;
  digitalWrite(inr1, LOW);
  digitalWrite(inr2, HIGH);
  analogWrite(enar, pov);
  //Serial.println("____ENAR_LEFT___");
}
void CarTLeft_ALGO(int pov)
{ //actual_position_volan = VOLAN_LEFT;
  digitalWrite(inr1, LOW);
  digitalWrite(inr2, HIGH);
  analogWrite(enar, pov);
  //Serial.println("____ENAR_LEFT___");
}
void CarTRight(int pov)
{
  actual_position_volan = VOLAN_RIGHT;
  digitalWrite(inr1, HIGH);
  digitalWrite(inr2, LOW);
  analogWrite(enar, pov);
}
void CarTRight_ALGO(int pov)
{ //actual_position_volan = VOLAN_RIGHT;
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
void check_light_value()
{
  int sensor_value = analogRead(LIGHT_SENSOR_PIN);
  if (sensor_value > TURN_OFF_FARURI_VALUE)
  {
    digitalWrite(RELEU_CONTROL_SIGNAL_PIN, LOW);
  }
  else
  {
    digitalWrite(RELEU_CONTROL_SIGNAL_PIN, HIGH);
  }
}
