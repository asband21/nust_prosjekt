/*
 * arm_kinematics_pure_cpp.cpp  (Version 3)
 * ------------------------------------------------
 * Pure‑standard‑C++ rewrite of the original Arduino sketch.
 * – No Servo / Arduino dependencies
 * – Uses the Eigen header‑only linear‑algebra library
 *
 * Build (Linux / macOS / Windows‑MSYS / WSL):
 *     g++ -std=c++17 arm_kinematics_pure_cpp.cpp -I/path/to/eigen -o arm_kinematics
 *
 * Eigen can be obtained from https://eigen.tuxfamily.org (unzip and point the
 * include path to the root folder that contains the “Eigen” directory).
 * ------------------------------------------------*/


#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <cmath>

#define PI 3.14159

// -----------------------------------------------------------------------------
// Robot geometry constants (SI units: metres)
// -----------------------------------------------------------------------------
constexpr float base_link       = 0.05f;  // distance from base frame to shoulder (z)
constexpr float upper_arm_link  = 0.10f;  // shoulder → elbow
constexpr float lower_arm_link  = 0.05f;  // elbow    → wrist / tool‑centre

using Mat4f = Eigen::Matrix4f;

// -----------------------------------------------------------------------------
// Convenience helpers: 4×4 homogeneous transforms
// -----------------------------------------------------------------------------
Mat4f rotX(float v) {
    Mat4f m = Mat4f::Identity();
    const float c = std::cos(v);
    const float s = std::sin(v);
    m(1,1) =  c;  m(1,2) = -s;
    m(2,1) =  s;  m(2,2) =  c;
    return m;
}

Mat4f rotY(float v) {
    Mat4f m = Mat4f::Identity();
    const float c = std::cos(v);
    const float s = std::sin(v);
    m(0,0) =  c;  m(0,2) =  s;
    m(2,0) = -s;  m(2,2) =  c;
    return m;
}

Mat4f rotZ(float v) {
    Mat4f m = Mat4f::Identity();
    const float c = std::cos(v);
    const float s = std::sin(v);
    m(0,0) =  c;  m(0,1) = -s;
    m(1,0) =  s;  m(1,1) =  c;
    return m;
}

Mat4f translate(float x, float y, float z) {
    Mat4f m = Mat4f::Identity();
    m(0,3) = x;
    m(1,3) = y;
    m(2,3) = z;
    return m;
}

// -----------------------------------------------------------------------------
// Joint‑space representation
// -----------------------------------------------------------------------------
struct arm_config {
    float base   = 0.0f; // rotation about Z (yaw)
    float shoulder = 0.0f; // rotation about Y
    float elbow  = 0.0f; // rotation about Y
    float gripper = 0.0f; // unused here
};

// -----------------------------------------------------------------------------
// Forward kinematics – returns the homogeneous transform of the tool frame
// relative to the robot base.
// -----------------------------------------------------------------------------
Mat4f forwardKinematics(const arm_config &q)
{
    // The sign convention follows the comments in the original sketch.
    return translate(0, 0, base_link) *
           rotZ(q.base) *
           rotY(-q.shoulder) *
           translate(0, 0, upper_arm_link) *
           rotY(-q.elbow) *
           translate(0, 0, lower_arm_link) *
           rotY(q.elbow + q.shoulder);
}

struct arm_config invers_kinematics(Mat4f m)
{
	double x = m(0, 3);
	double y = m(1, 3);
	double z = m(2, 3);
	struct arm_config arm {0,0,0,0};
	
	arm.base = atan2(y, x);
	double pre_acos = (x*x + y*y + z*z - upper_arm_link*upper_arm_link - lower_arm_link*lower_arm_link)/(-2*upper_arm_link*lower_arm_link);
	double tre_albuge = acos(pre_acos);
	arm.elbow = PI - tre_albuge;
	double tre_skuller = asin(lower_arm_link*sin(tre_albuge)/(sqrt(x*x +y*y +z*z)));
	arm.shoulder = atan2(z, sqrt(x*x + y*y)) + tre_skuller;
	if(std::isnan(arm.elbow)) arm.elbow = 0;
	if(std::isnan(arm.shoulder)) arm.shoulder = 0;

	//printf("x:%f\ty:%f\tz:%f\n",x,y,z);
	//printf("%f\t%f\t%f\n",x,y,z);
	return arm;
}

// -----------------------------------------------------------------------------
// Demo program – sweeps the base joint and prints the tool‑centre pose.
// -----------------------------------------------------------------------------
int main()
{
    std::cout << std::fixed << std::setprecision(3);

    for (float b = -static_cast<float>(M_PI); b < static_cast<float>(M_PI); b += 0.01f) {
        arm_config q{b, 1.0f, 0.0f, 0.0f};
        Mat4f T = forwardKinematics(q);
	struct arm_config h = invers_kinematics(T);
        std::cout << q.base << '\t' << h.base << '\n';
        //std::cout << "Tool frame transform:\n" << T << "\n\n";
    }

    return 0;
}


