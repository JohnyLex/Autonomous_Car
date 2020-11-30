#define LIGHT_SENSOR_PIN A0
#define TURN_OFF_FARURI_VALUE 200
#define RELEU_CONTROL_SIGNAL_PIN 52
/*
 * INFORMATIE DESPRE UN TUTORIAL  https://create.arduino.cc/projecthub/iasonas-christoulakis/set-up-a-5v-relay-on-the-arduino-0b37ff
 */
void check_light_value();

void setup()
{
  pinMode(RELEU_CONTROL_SIGNAL_PIN, OUTPUT);
}

void loop()
{
  check_light_value();
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
