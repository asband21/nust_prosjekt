// Base Shield V2
#include <Servo.h>
#include "TinyMatrixMath.hpp"

Servo klo;
Servo albuge;
Servo skulle;
Servo base;
tmm::Matrix<4,4> l;

float m_base_ra[4][4] = {{1, 0, 0, 0},
                          {0, 1, 0, 0},
                          {0, 0, 1, 0.4},
                          {0, 0, 0, 1}};
tmm::Matrix<4,4> rot_x(float v){
  float m_rot_x_ra[4][4] = {{1,      0,      0, 0},
                            {0, cos(v),-sin(v), 0},
                            {0, sin(v), cos(v), 0},
                            {0,      0,      0, 1}};
  tmm::Matrix<4,4> m_rot_x(m_rot_x_ra);
  return m_rot_x;
}

tmm::Matrix<4,4> rot_y(float v){
  float m_rot_y_ra[4][4] = {{cos(v), 0, -sin(v), 0},
                            {     0, 1,       0, 0},
                            {sin(v), 0,  cos(v), 0},
                            {     0, 0,       0, 1}};
  tmm::Matrix<4,4> m_rot_y(m_rot_y_ra);
  return m_rot_y;
}

tmm::Matrix<4,4> rot_z(float v){
  float m_rot_z_ra[4][4] = {{cos(v),-sin(v), 0, 0},
                           { sin(v), cos(v), 0, 0},
                           {      0,      0, 1, 0},
                           {      0,      0, 0, 1}};
  tmm::Matrix<4,4> m_rot_z(m_rot_z_ra);
  return m_rot_z;
}

tmm::Matrix<4,4> trans_m(float x, float y, float z){
  float m_trans_ra[4][4] = {{1, 0, 0, x},
                           { 0, 1, 0, y},
                           { 0, 0, 1, z},
                           { 0, 0, 0, 1}};
  tmm::Matrix<4,4> m_trans(m_trans_ra);
  return m_trans;
}

int pwm_pungt = 100;
void setup()
{
  tmm::Matrix<4,4> m_base(m_base_ra);
  skulle.attach( 2, 25, 190); // pwm 25-190 -> vinkel -45 - 70 
  albuge.attach( 3,  0, 150); // pwm 0 -150 -> vinkel  30 - 150
  klo.attach   ( 4, 40, 80 );    // pwm 40-190 -> vinkel   0 - 80
  
  skulle.write(110);
  albuge.write(110);
  klo.write(110);

  Serial.begin(115200);
  tmm::Matrix<4,4> E = m_base * m_base;
  tmm::Matrix<4,4> rr = rot_x(0.1);
  E.printTo(Serial);
  l = tmm::Identity<4>();
}
void loop()
{
  tmm::Matrix<4,4> m_base(m_base_ra);
  tmm::Matrix<4,4> rr = rot_x(0.1);
  l.printTo(Serial);

  l = l*m_base*rr;

  /*
  for(int i = 0; i < 255; i++)
  {
    albuge.write(i);
    Serial.println(i);
    delay(100);

  }
  
  

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
