// Base Shield V2
#include <Servo.h>


Servo klo;
Servo albuge;
Servo skulle;
Servo base;

/*
const int klo_pin = 5;  //?
const int albuge_pin = 3; // ok
const int skuller_pin = 6; //?
const int base_pin = 5;
*/
int pwm_pungt = 100;
void setup()
{
  skulle.attach(2); // pwm 25-190 -> vinkel -45 - 70 
  albuge.attach(3); // pwm 0 -150 -> vinkel  30 - 150
  klo.attach(4);    // pwm 40-190 -> vinkel   0 - 80
  
  skulle.write(110);
  albuge.write(110);
  klo.write(110);

  Serial.begin(115200);

}

void loop()
{
  for(int i = 0; i < 255; i++)
  {
    albuge.write(i);
    Serial.println(i);
    delay(100);

  }
  /*
  for(int i = 0; i < 255; i++)
  {
    skulle.write(i);
    albuge.write(i);
    klo.write(i);
    delay(20);
  }
  for(int i = 255; i > 1; i--)
  {
    skulle.write(i);
    albuge.write(i);
    klo.write(i);
    delay(20);
  }
  */
}
