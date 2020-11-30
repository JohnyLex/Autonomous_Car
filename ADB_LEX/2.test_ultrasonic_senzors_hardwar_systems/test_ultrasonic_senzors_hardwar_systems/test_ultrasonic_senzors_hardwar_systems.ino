#include <HCSR04.h>

#define trigPin1 11
#define echoPin1 10

#define trigPin2 24 
#define echoPin2 25

#define trigPin3 26
#define echoPin3 27

#define trigPin4 28
#define echoPin4 29

#define trigPin5 30
#define echoPin5 31
//HCSR04 ultrasonic_senzors(new int[5]{trigPin1,trigPin2,trigPin3,trigPin4,trigPin5},new int[5]{echoPin1,echoPin2,echoPin3,echoPin4,echoPin5},5);

HCSR04 sonic_1_front(trigPin1,echoPin1);
//HCSR04 sonic_2_front(trigPin2,echoPin2);
//HCSR04 sonic_3_front(trigPin3,echoPin3);
//HCSR04 sonic_4_front(trigPin4,echoPin4);
//HCSR04 sonic_5_front(trigPin5,echoPin5);

long vsonic_1_front,vsonic_2_front,vsonic_3_front,vsonic_4_front,vsonic_5_front;
void setup()
{
  Serial.begin(9600);
}

void loop()
{
vsonic_1_front = sonic_1_front.dist();
//vsonic_2_front = sonic_2_front.dist();
//vsonic_3_front = sonic_3_front.dist();
//vsonic_4_front = sonic_4_front.dist();
//vsonic_5_front = sonic_5_front.dist();
Serial.println("--------------------ULTRAS_VALUE----------------");
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
  Serial.println("--------------------END_VALUE_ULTRAS_PRINT----------------");

}
