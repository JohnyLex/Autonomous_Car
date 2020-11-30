/*
  

  This sketch uses the TimerOne library downloadable from here:
  http://code.google.com/p/arduino-timerone/downloads/detail?name=TimerOne-v9.zip&can=2&q=

  Install the library using the following guide:
  http://arduino.cc/en/Guide/Libraries

  Please read Information about how works with interrupt on arduino by clicking this link
  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
*/

#include <TimerOne.h>                                 // Header file for TimerOne library
/*
 * =====================================ULTRASONIC_SENSORS_DEFINE_BLOCK====================================
 */
#define trigPin 12                                    // Pin 12 trigger output
#define echoPin 2                                     // Pin 2 Echo input
#define echoPin2 3                                     // Pin 2 Echo input
#define echoPin3 4                                     // Pin 2 Echo input
#define echoPin4 5                                     // Pin 2 Echo input
#define echoPin5 6                                     // Pin 2 Echo input

#define ISR_ID_echoPin1 0
#define ISR_ID_echoPin2 1
#define ISR_ID_echoPin3 2
#define ISR_ID_echoPin4 3
#define ISR_ID_echoPin5 4

#define onBoardLED 13                                 // Pin 13 onboard LED
#define echo_int 0                                    // Interrupt id for echo pulse
#define TIMER_US 50                                   // 50 uS timer duration 
#define TICK_COUNTS 4000                              // 200 mS worth of timer ticks

volatile long echo_start = 0;                         // Records start of echo pulse
volatile long echo_end = 0;                           // Records end of echo pulse

volatile long echo_duration = 0;                      // Duration - difference between end and start
volatile long echo2_duration = 0;                      // Duration - difference between end and start
volatile long echo3_duration = 0;                      // Duration - difference between end and start
volatile long echo4_duration = 0;                      // Duration - difference between end and start
volatile long echo5_duration = 0;                      // Duration - difference between end and start

volatile int trigger_time_count = 0;                  // Count down counter to trigger pulse time
volatile long range_flasher_counter = 0;              // Count down counter for flashing distance LED
int sonic_state = 0;
long vsonic_1_front, vsonic_2_front, vsonic_3_front, vsonic_4_front, vsonic_5_front;
/*
 * ==========================================================================================================================
 */
// ----------------------------------
// setup() routine called first.
// A one time routine executed at power up or reset time.
// Used to initialise hardware.
// ----------------------------------
void setup()
{


pinMode(onBoardLED, OUTPUT);                        // Onboard LED pin set to output
ultrasonic_pind_init();
ultrasonic_interrupt_attach();
  
  Serial.begin (9600);                                // Initialise the serial monitor output
}

// ----------------------------------
// loop() Runs continuously in a loop.
// This is the background routine where most of the processing usualy takes place.
// Non time critical tasks should be run from here.
// ----------------------------------
void loop()
{

  vsonic_1_front = (echo_duration / 58);
  vsonic_2_front = (echo2_duration / 58);
  vsonic_3_front = (echo3_duration / 58);
  vsonic_4_front = (echo4_duration / 58);
    vsonic_5_front = (echo5_duration / 58);
  //Serial.println((echo_duration / 58));
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
  Serial.println(echo_duration / 58);               // Print the distance in centimeters
  delay(100);                                       // every 100 mS
}


/*
 * ===========================================================FUNCTION_ULTRASONIC_USING_TIMMERS_INTERRUPT======================================
 */
// --------------------------
// timerIsr() 50uS second interrupt ISR()
// Called every time the hardware timer 1 times out.
// --------------------------
void timerIsr()
{
  trigger_pulse();                                 // Schedule the trigger pulses
  distance_flasher();                              // Flash the onboard LED distance indicator
}

// --------------------------
// trigger_pulse() called every 50 uS to schedule trigger pulses.
// Generates a pulse one timer tick long.
// Minimum trigger pulse width for the HC-SR04 is 10 us. This system
// delivers a 50 uS pulse.
// --------------------------
void trigger_pulse()
{
  static volatile int state = 0;                 // State machine variable

  if (!(--trigger_time_count))                   // Count to 200mS
  { // Time out - Initiate trigger pulse
    trigger_time_count = TICK_COUNTS;           // Reload
    state = 1;                                  // Changing to state 1 initiates a pulse
  }

  switch (state)                                 // State machine handles delivery of trigger pulse
  {
    case 0:                                      // Normal state does nothing
      break;

    case 1:                                      // Initiate pulse
      digitalWrite(trigPin, HIGH);              // Set the trigger output high
      state = 2;                                // and set state to 2
      break;

    case 2:                                      // Complete the pulse
    default:
      digitalWrite(trigPin, LOW);               // Set the trigger output low
      state = 0;                                // and return state to normal 0
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
  switch (digitalRead(echoPin))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();                          // Save the end time
      echo_duration = echo_end - echo_start;        // Calculate the pulse duration
      break;
  }
}
void echo2_interrupt()
{
  switch (digitalRead(echoPin2))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();                          // Save the end time
      echo2_duration = echo_end - echo_start;        // Calculate the pulse duration
      break;
  }
}

void echo3_interrupt()
{
  switch (digitalRead(echoPin3))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();                          // Save the end time
      echo3_duration = echo_end - echo_start;        // Calculate the pulse duration
      break;
  }
}

void echo4_interrupt()
{
  switch (digitalRead(echoPin4))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();                          // Save the end time
      echo4_duration = echo_end - echo_start;        // Calculate the pulse duration
      break;
  }
}
void echo5_interrupt()
{
  switch (digitalRead(echoPin5))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();                          // Save the end time
      echo5_duration = echo_end - echo_start;        // Calculate the pulse duration
      break;
  }
}

void distance_flasher()
{
  if (--range_flasher_counter <= 0)                // Decrement and test the flash timer
  { // Flash timer time out
    if (echo_duration < 25000)                    // If the echo duration is within limits
    {
      range_flasher_counter = echo_duration * 2;  // Reload the timer with the current echo duration
    }
    else
    {
      range_flasher_counter = 25000;              // If out of range use a default
    }

    digitalWrite( onBoardLED, digitalRead( onBoardLED ) ^ 1 );   // Toggle the onboard LED
  }
}



void ultrasonic_interrupt_attach()
{
  Timer1.initialize(TIMER_US);                        // Initialise timer 1
  Timer1.attachInterrupt( timerIsr );                 // Attach interrupt to the timer service routine
  attachInterrupt(ISR_ID_echoPin1, echo1_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
  attachInterrupt(ISR_ID_echoPin2, echo2_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
  attachInterrupt(ISR_ID_echoPin3, echo3_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
  attachInterrupt(ISR_ID_echoPin4, echo4_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
  attachInterrupt(ISR_ID_echoPin5, echo5_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
}

void ultrasonic_pind_init()
{
    pinMode(trigPin, OUTPUT);                           // Trigger pin set to output
  pinMode(echoPin, INPUT);                            // Echo pin set to input
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);                            // Echo pin set to input
  pinMode(echoPin4, INPUT);
  pinMode(echoPin5, INPUT);
}

/*
 * =====================================================================================================================================
 */
