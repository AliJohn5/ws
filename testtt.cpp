#include "ros/ros.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Bool.h>
#include <math.h>

#define R2_L1 .224
#define R2_L2 .250
#define R2_L3 .230
#define R2_L4 .171

#define  R1_L1 .120
#define  R1_L2 .150
#define  R1_L3 .110
#define  R1_L5 .021

#define WR1 0 
#define LR1 .310 
#define HR1 .120 

#define WR2 .390 
#define LR2 0 
#define HR2 .275


#define ori 0

#define increment 30

 

float R1_q1 = M_PI / 2, R1_q2 = M_PI / 2, R1_q3 = -M_PI / 2, R1_q4 = 0;
float R1_q1_dot = 0, R1_q2_dot = 0, R1_q3_dot = 0, R1_q4_dot = 0;
float R1_Wx = 0, R1_Wy = 0, R1_Wz = 0;

float R2_q1 = M_PI / 2, R2_q2 = M_PI / 2, R2_q3 = -M_PI / 2, R2_q4 = 0;
float R2_q1_dot = 0, R2_q2_dot = 0, R2_q3_dot = 0, R2_q4_dot = 0;
float R2_Wx = 0, R2_Wy = 0, R2_Wz = 0;

float px = 0, py = R2_L3 + R2_L4, pz = R2_L1 + R2_L2;

float px_R1=py+WR1;
float py_R1=px+LR1;
float pz_R1=pz+HR1;

float px_R2=px+WR2;
float py_R2=py+LR2;
float pz_R2=pz+HR2;

float rad = .1;
float theta = 0;

float X_dot = 0.0;
float Y_dot = 0.0;
float Z_dot = 0.0;
float q3_old = 0;

std_msgs::Float32MultiArray angles_msg; // this msg contain angles of tow arm R1 & R2.

void init_msg();
void calculate_R2_q_dot();
void calculate_R1_q_dot();
void inverse();
void rad2steps();

void update_angles();
int main(int argc, char **argv) {
  ros::init(argc, argv, "T");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("angles_1", 1);
  init_msg();

  ros::Rate loop_rate(increment);

  inverse();
  while (ros::ok()) {
   // calculate_R1_q_dot();
    calculate_R2_q_dot();
    update_angles();

    R2_q3_dot =R2_q3_dot - R2_q2_dot;

   /* X_dot = rad * cos(theta);
    Y_dot = rad * sin(theta);
    theta += M_PI / increment;
*/
    rad2steps();

    angles_msg.data.clear(); // clear data of last angles smg.
    angles_msg.data.push_back(R1_q1_dot);
    angles_msg.data.push_back(-R1_q2_dot);
    angles_msg.data.push_back(R1_q3_dot);
    angles_msg.data.push_back(R1_q4_dot);

    angles_msg.data.push_back(R2_q1_dot);
    angles_msg.data.push_back(R2_q2_dot);
    angles_msg.data.push_back(R2_q3_dot);
    angles_msg.data.push_back(R2_q4_dot);

    chatter_pub.publish(angles_msg);


    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
void init_msg() {
  std_msgs::MultiArrayDimension msg_dim;
  msg_dim.label = "angles";
  msg_dim.size = 8; // size is [11] 'cause we have 11 angles of tow arms.
  msg_dim.stride = 8;
  angles_msg.layout.dim.clear(); // clean layout.
  angles_msg.layout.dim.push_back(msg_dim); // sets layout of angles msg.
}
void calculate_R2_q_dot() {


 R2_q1_dot = (Y_dot*cos(R2_q1) - X_dot*sin(R2_q1))/(R2_L3*cos(R2_q2 + R2_q3) + R2_L2*cos(R2_q2) + R2_L4*cos(R2_q2 + R2_q3 + R2_q4));

  R2_q2_dot = (Y_dot*cos(R2_q2)*cos(R2_q3) - Y_dot*sin(R2_q2)*sin(R2_q3) + R2_L4*R2_Wx*sin(R2_q4) - Y_dot*cos(R2_q1)*cos(R2_q1)*cos(R2_q2)*cos(R2_q3) + Y_dot*cos(R2_q1)*cos(R2_q1)*sin(R2_q2)*sin(R2_q3) + Z_dot*cos(R2_q2)*sin(R2_q1)*sin(R2_q3) + Z_dot*cos(R2_q3)*sin(R2_q1)*sin(R2_q2) + X_dot*cos(R2_q1)*cos(R2_q2)*cos(R2_q3)*sin(R2_q1) - X_dot*cos(R2_q1)*sin(R2_q1)*sin(R2_q2)*sin(R2_q3))/(R2_L2*sin(R2_q1)*sin(R2_q3));

 R2_q3_dot = -(R2_L2*Y_dot*cos(R2_q2) + R2_L3*R2_L4*R2_Wx*sin(R2_q4) + R2_L3*Y_dot*cos(R2_q2)*cos(R2_q3) + R2_L2*Z_dot*sin(R2_q1)*sin(R2_q2) - R2_L3*Y_dot*sin(R2_q2)*sin(R2_q3) - R2_L2*Y_dot*cos(R2_q1)*cos(R2_q1)*cos(R2_q2) + R2_L2*X_dot*cos(R2_q1)*cos(R2_q2)*sin(R2_q1) + R2_L3*Z_dot*cos(R2_q2)*sin(R2_q1)*sin(R2_q3) + R2_L3*Z_dot*cos(R2_q3)*sin(R2_q1)*sin(R2_q2) - R2_L3*Y_dot*cos(R2_q1)*cos(R2_q1)*cos(R2_q2)*cos(R2_q3) + R2_L3*Y_dot*cos(R2_q1)*cos(R2_q1)*sin(R2_q2)*sin(R2_q3) + R2_L2*R2_L4*R2_Wx*cos(R2_q3)*sin(R2_q4) + R2_L2*R2_L4*R2_Wx*cos(R2_q4)*sin(R2_q3) - R2_L3*X_dot*cos(R2_q1)*sin(R2_q1)*sin(R2_q2)*sin(R2_q3) + R2_L3*X_dot*cos(R2_q1)*cos(R2_q2)*cos(R2_q3)*sin(R2_q1))/(R2_L2*R2_L3*sin(R2_q1)*sin(R2_q3));

 R2_q4_dot = (Y_dot*cos(R2_q2) + Z_dot*sin(R2_q1)*sin(R2_q2) - Y_dot*cos(R2_q1)*cos(R2_q1)*cos(R2_q2) + R2_L3*R2_Wx*sin(R2_q3) + R2_L4*R2_Wx*cos(R2_q3)*sin(R2_q4) + R2_L4*R2_Wx*cos(R2_q4)*sin(R2_q3) + X_dot*cos(R2_q1)*cos(R2_q2)*sin(R2_q1))/(R2_L3*sin(R2_q1)*sin(R2_q3));

}
void calculate_R1_q_dot() {
  R1_q1_dot = (Y_dot * cos(R1_q1) - X_dot * sin(R1_q1)) / (R1_L3 * cos(R1_q2 + R1_q3) + R1_L2 * cos(R1_q2) + R1_L5 * cos(R1_q2 + R1_q3 + R1_q4));

  R1_q2_dot = (Y_dot * cos(R1_q2) * cos(R1_q3) - Y_dot * sin(R1_q2) * sin(R1_q3) + R1_Wx * R1_L5 * sin(R1_q4) + Z_dot * cos(R1_q2) * sin(R1_q1) * sin(R1_q3) + Z_dot * cos(R1_q3) * sin(R1_q1) * sin(R1_q2) - Y_dot * cos(R1_q1) * cos(R1_q1) * cos(R1_q2) * cos(R1_q3) + Y_dot * cos(R1_q1) * cos(R1_q1) * sin(R1_q2) * sin(R1_q3) + X_dot * cos(R1_q1) * cos(R1_q2) * cos(R1_q3) * sin(R1_q1) - X_dot * cos(R1_q1) * sin(R1_q1) * sin(R1_q2) * sin(R1_q3)) / (R1_L2 * sin(R1_q1) * sin(R1_q3));

  R1_q3_dot = -(Y_dot * R1_L2 * cos(R1_q2) - Y_dot * R1_L2 * cos(R1_q1) * cos(R1_q1) * cos(R1_q2) + R1_Wx * R1_L3 * R1_L5 * sin(R1_q4) + Y_dot * R1_L3 * cos(R1_q2) * cos(R1_q3) - Y_dot * R1_L3 * sin(R1_q2) * sin(R1_q3) + Z_dot * R1_L2 * sin(R1_q1) * sin(R1_q2) + Y_dot * R1_L3 * cos(R1_q1) * cos(R1_q1) * sin(R1_q2) * sin(R1_q3) + R1_Wx * R1_L2 * R1_L5 * cos(R1_q3) * sin(R1_q4) + R1_Wx * R1_L2 * R1_L5 * cos(R1_q4) * sin(R1_q3) + X_dot * R1_L2 * cos(R1_q1) * cos(R1_q2) * sin(R1_q1) + Z_dot * R1_L3 * cos(R1_q2) * sin(R1_q1) * sin(R1_q3) + Z_dot * R1_L3 * cos(R1_q3) * sin(R1_q1) * sin(R1_q2) - Y_dot * R1_L3 * cos(R1_q1) * cos(R1_q1) * cos(R1_q2) * cos(R1_q3) + X_dot * R1_L3 * cos(R1_q1) * cos(R1_q2) * cos(R1_q3) * sin(R1_q1) - X_dot * R1_L3 * cos(R1_q1) * sin(R1_q1) * sin(R1_q2) * sin(R1_q3)) / (R1_L2 * R1_L3 * sin(R1_q1) * sin(R1_q3));

  R1_q4_dot = (Y_dot * cos(R1_q2) + Z_dot * sin(R1_q1) * sin(R1_q2) - Y_dot * cos(R1_q1) * cos(R1_q1) * cos(R1_q2) + R1_Wx * R1_L3 * sin(R1_q3) + X_dot * cos(R1_q1) * cos(R1_q2) * sin(R1_q1) + R1_Wx * R1_L5 * cos(R1_q3) * sin(R1_q4) + R1_Wx * R1_L5 * cos(R1_q4) * sin(R1_q3)) / (R1_L3 * sin(R1_q1) * sin(R1_q3));
}
void update_angles() {
  R1_q1 = R1_q1 + R1_q1_dot / increment;
  R1_q2 = R1_q2 + R1_q2_dot / increment;
  R1_q3 = R1_q3 + R1_q3_dot / increment;
  R1_q4 = R1_q4 + R1_q4_dot / increment;

  R2_q1 = R2_q1 + R2_q1_dot / increment;
  R2_q2 = R2_q2 + R2_q2_dot / increment;
  R2_q3 = R2_q3 + R2_q3_dot / increment;
  R2_q4 = R2_q4 + R2_q4_dot / increment;
}
void rad2steps() {

  R2_q1_dot = R2_q1_dot * 6400.0 / (2 * M_PI); // steps/s
  R2_q2_dot = -R2_q2_dot * 6400.0 * 4.5 / (2 * M_PI);
  R2_q3_dot = R2_q3_dot * 6400.0 * 4.5 / (2 * M_PI);

  R1_q1_dot = R1_q1_dot * 6400.0 * 3.3 / (2 * M_PI); // steps/s
  R1_q2_dot = -R1_q2_dot * 6400.0 * 5.0 / (2 * M_PI);
  R1_q3_dot = R1_q3_dot * 6400.0 * 3.5 / (2 * M_PI);
}
void inverse() {
  float A, B, C, a, b, r;

  /**** R2 ****/

  R2_q1 = atan2(py, px); // theta 1
  if (!py && !px)
    R2_q1 = 0;

  A = px - R2_L4 * cos(R2_q1) * cos(ori);
  B = py - R2_L4 * sin(R2_q2) * cos(ori);
  C = pz - R2_L1 - R2_L4 * sin(ori);

  R2_q3 = -acos((A * A + B * B + C * C - R2_L2 * R2_L2 - R2_L3 * R2_L3) / (2 * R2_L2 * R2_L3)); // theta 3

  a = R2_L3 * sin(R2_q3);
  b = R2_L2 + R2_L3 * cos(R2_q3);
  r = sqrt(a * a + b * b);

  R2_q2 = atan2(C, sqrt(r * r - C * C)) - atan2(a, b); // theta 2

  R2_q4 = ori - R2_q2 - R2_q3; // theta 4

  R2_q3 = R2_q3 - atan2(R2_L2 * cos(R2_q2), R2_L2 * sin(R2_q2));
  R2_q2 = M_PI - R2_q2;
}
