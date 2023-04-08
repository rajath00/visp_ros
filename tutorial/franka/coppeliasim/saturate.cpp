#include <iostream>
#include <visp/vpRobot.h>

int main()
{
  // Set a velocity skew vector
  vpColVector v(6);
  v[0] = 0.1;               // vx in m/s
  v[1] = 0.2;               // vy
  v[2] = 0.3;               // vz
  v[3] = vpMath::rad(10);   // wx in rad/s
  v[4] = vpMath::rad(-10);  // wy
  v[5] = vpMath::rad(20);   // wz
  // Set the maximal allowed velocities
  vpColVector v_max(6);
  for (int i=0; i<3; i++)
    v_max[i] = 0.3;             // in translation (m/s)
  for (int i=3; i<6; i++)
    v_max[i] = vpMath::rad(10); // in rotation (rad/s)
  // Compute the saturated velocity skew vector
  vpColVector v_sat = vpRobot::saturateVelocities(v, v_max, true);
  std::cout << "v    : " << v.t() << std::endl;
  std::cout << "v max: " << v_max.t() << std::endl;
  std::cout << "v sat: " << v_sat.t() << std::endl;
  return 0;
}
