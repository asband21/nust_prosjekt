// Base Shield V2
#include <Servo.h>
#include "TinyMatrixMath.hpp"

float base_link = 0.05;
float uper_arm_link = 0.1;
float under_arm_link = 0.05;

Servo klo;
Servo albuge;
Servo skulle;
Servo base;
tmm::Matrix<4,4> l;

float m_base_ra[4][4] = {{1, 0, 0, 0  },
			 {0, 1, 0, 0  },
			 {0, 0, 1, 0.4},
			 {0, 0, 0, 1  }};

tmm::Matrix<4,4> rot_x(float v)
{
	float m_rot_x_ra[4][4] = {{1,      0,      0, 0},
				  {0, cos(v),-sin(v), 0},
				  {0, sin(v), cos(v), 0},
				  {0,      0,      0, 1}};
	tmm::Matrix<4,4> m_rot_x(m_rot_x_ra);
	return m_rot_x;
}

tmm::Matrix<4,4> rot_y(float v)
{
	float m_rot_y_ra[4][4] = {{cos(v), 0, -sin(v), 0},
				  {     0, 1,       0, 0},
				  {sin(v), 0,  cos(v), 0},
				  {     0, 0,       0, 1}};
	tmm::Matrix<4,4> m_rot_y(m_rot_y_ra);
	return m_rot_y;
}

tmm::Matrix<4,4> rot_z(float v){
	float m_rot_z_ra[4][4] = {{ cos(v),-sin(v), 0, 0},
				  { sin(v), cos(v), 0, 0},
				  {      0,      0, 1, 0},
				  {      0,      0, 0, 1}};
	tmm::Matrix<4,4> m_rot_z(m_rot_z_ra);
	return m_rot_z;
}

tmm::Matrix<4,4> trans_m(float x, float y, float z){
	float m_trans_ra[4][4] = {{ 1, 0, 0, x},
				  { 0, 1, 0, y},
				  { 0, 0, 1, z},
				  { 0, 0, 0, 1}};
	tmm::Matrix<4,4> m_trans(m_trans_ra);
	return m_trans;
}


struct arm_fon_fig
{
  float base;
  float skuller;
  float albuge;
  float klo;
};

//tmm::Matrix<4,4> forvert_kinmatik(float base, float skuller, float albuge)
tmm::Matrix<4,4> forvert_kinmatik(struct arm_fon_fig arm)
{
  //The negative rotation and first Y rotation are defined this way because, without an initial positive rotation, you can directly use atan2 to recover the angle.
  //This aligns with the intuition you get when physically rotating the joint arm.
	return trans_m(0, 0, base_link) * rot_z(arm.base) * rot_y(-arm.skuller) * trans_m(0, 0, uper_arm_link) * rot_y(-arm.albuge) * trans_m(0, 0, under_arm_link) * rot_y(arm.albuge+arm.skuller);
}

float extrag_skaler(tmm::Matrix<4,4> m, int x, int y)
{
	float x_raw[1][4] = {0, 0, 0, 0};
	float y_raw[4][1] = {{0}, {0}, {0}, {0}};
	x_raw[0][x] = 1;
	y_raw[y][0] = 1;
	tmm::Matrix<1,4> x_m(x_raw);
	tmm::Matrix<4,1> y_m(y_raw);
	tmm::Matrix<1,1> s = x_m * m * y_m;
	return (float)s; 
}

struct arm_fon_fig invers_kinmatik(tmm::Matrix<4,4> end_pos)
{
	float x = extrag_skaler(end_pos, 0, 3);
	float y = extrag_skaler(end_pos, 1, 3);
	float z = extrag_skaler(end_pos, 2, 3) - base_link;

	// finde base vinkel atan2 
	struct arm_fon_fig arm;
	arm.base = atan2(y, x);
	// finde albuge vinkel ved Cosinussetningen
	//float k = sqrt(x^2+y^2);
	float pre_acos = (x*x + y*y + z*z - uper_arm_link*uper_arm_link - under_arm_link*under_arm_link)/(-2*uper_arm_link*under_arm_link);
	float tre_albuge = acos(pre_acos);
	arm.albuge = PI- tre_albuge;
	// find skuller vinkel ved Sinussetningen
	float tre_skuller = asin(under_arm_link*sin(tre_albuge)/(sqrt(x*x +y*y +z*z)));
	// finde lige vinkel til end efter;
	arm.skuller = atan2(z, sqrt(x*x + y*y)) + tre_skuller;
  if(isnan(arm.albuge)) arm.albuge = 0;
  if(isnan(arm.skuller)) arm.skuller = 0;
	return arm;
} 

int pwm_pungt = 100;
void setup()
{
	tmm::Matrix<4,4> m_base(m_base_ra);
	skulle.attach( 2, 25, 190); // pwm 25-190 -> vinkel -45 - 70 
	albuge.attach( 3,  0, 150); // pwm 0 -150 -> vinkel  30 - 150
	klo.attach   ( 4, 40, 80 ); // pwm 40-190 -> vinkel   0 - 80

	skulle.write(110);
	albuge.write(110);
	klo.write(110);

	Serial.begin(115200);
	tmm::Matrix<4,4> E = m_base * m_base;
	tmm::Matrix<4,4> rr = rot_x(0.1);
	E.printTo(Serial);
	l = tmm::Identity<4>();
}

void print_arm(struct arm_fon_fig a, String pra_fix)
{
	Serial.print(pra_fix);
	//Serial.print("b:");
	//Serial.print(a.base);
	//Serial.print("\ts:");
	Serial.print(a.skuller);
	//Serial.print("\ta:");
	//Serial.print(a.albuge);
	//Serial.print("\tk:");
	//Serial.print(a.klo);
}


void loop()
{
  /*
   */
  for(float b = -PI; b < PI; b = b +0.01)
  {
            struct arm_fon_fig arm = {0,b,0,0};
            tmm::Matrix<4,4> e = forvert_kinmatik(arm);
            struct arm_fon_fig arm_2 = invers_kinmatik(e);
            print_arm(arm, "\n1:");
            print_arm(arm_2, "\t2:");

    
   }
  
	/*
	Serial.println("-----------------------------");
	for(float b = -PI; b < PI; b = b +0.2)
		for(float s = -PI; s < PI; s = s +0.2)
			for(float a = -PI; a < PI; a = a +0.2)
			{
				struct arm_fon_fig arm = {b,s,a,0};
				tmm::Matrix<4,4> e = forvert_kinmatik(arm);
				struct arm_fon_fig arm_2 = invers_kinmatik(e);
				print_arm(arm, "\n1:");
				print_arm(arm_2, "\t2:");
			}
	delay(100000);

	   tmm::Matrix<4,4> m_base(m_base_ra);
	   tmm::Matrix<4,4> rr = rot_x(0.1);
	   l.printTo(Serial);

	   l = l*m_base*rr;

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
