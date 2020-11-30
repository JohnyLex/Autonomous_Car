#define trigPin1 22
#define echoPin1 23
#define trigPin2 24 
#define echoPin2 25
#define trigPin3 26
#define echoPin3 27
#define trigPin4 28
#define echoPin4 29
#define trigPin5 30
#define echoPin5 31
//////////////////////////////////

#define ena 2   //enable
#define in1 4
#define in2 3

#define enal 8   //enable
#define inl1 10
#define inl2 9
//////////////////////////////
#define enar 5    // enable ruli
#define inr1 6
#define inr2 7  
/////////////////////////////


void CarStop(void);
void CarDrive(int pov);
void CarTLeft(int pov);
void CarTRight(int pov);
void CarTStop();
void CarReverse(int pov);
/*
 * -------------------------------------SETUP---------------------------------------
 */
void setup()
{
  ///////////////////////// rotile
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enal, OUTPUT);
  pinMode(inl1, OUTPUT);
  pinMode(in2, OUTPUT);
  //volan
  pinMode(inr1, OUTPUT);
  pinMode(inr2, OUTPUT);
  pinMode(enar, OUTPUT);
}

/*
 * ---------------------------------MAIN_LOOP-------------------------------------------
 */
void loop()
{
 CarDrive(250);
 delay(2000);
 CarReverse(250);
 delay(2000);
 CarStop();                        //stop rotile din spate
 CarTRight(240);
 delay(1000);
 CarTLeft(240);
 delay(1000);
}

/*
 * --------------------------------FUNCTION---------------------------------------------
 */
void CarReverse(int pov) {

  digitalWrite(inl2, HIGH);
  digitalWrite(inl1,LOW );
  analogWrite(enal, pov);

  digitalWrite(in2,HIGH );
  digitalWrite(in1, LOW);
  analogWrite(ena, pov );
}
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
void CarTLeft(int pov)
{
  digitalWrite(inr1, LOW);
  digitalWrite(inr2, HIGH);
  analogWrite(enar, pov);

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
