/*
 * Utilities.h
 *
 *  Created on: 28/09/2015
 *      Author: paco
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <math.h>

namespace wm_navigation {
inline double
randn (double mu, double sigma)
{
  double U1, U2, W, mult;
  static double X1, X2;
  static int call = 0;

  if (call == 1)
    {
      call = !call;
      return (mu + sigma * (double) X2);
    }

  do
    {
      U1 = -1 + ((double) rand () / RAND_MAX) * 2;
      U2 = -1 + ((double) rand () / RAND_MAX) * 2;
      W = pow (U1, 2) + pow (U2, 2);
    }
  while (W >= 1 || W == 0);

  mult = sqrt ((-2 * log (W)) / W);
  X1 = U1 * mult;
  X2 = U2 * mult;

  call = !call;

  return (mu + sigma * (double) X1);
}

inline double toRadians(double degrees){return degrees * (M_PI/180.0);}

inline double normalizePi(double data)
{
  if (data < M_PI && data >= -M_PI) return data;
  double ndata = data - ((int )(data / (M_PI*2.0)))*(M_PI*2.0);
  while (ndata >= M_PI)
  {
    ndata -= (M_PI*2.0);
  }
  while (ndata < -M_PI)
  {
    ndata += (M_PI*2.0);
  }
  return ndata;
}

inline float distance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
	return sqrt((p1.position.x-p2.position.x)*(p1.position.x-p2.position.x)+
			(p1.position.y-p2.position.y)*(p1.position.y-p2.position.y));
}

inline float angleZ(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
	double ro1, ro2, pi1, pi2, ya1, ya2;
	tf::Quaternion q1(p1.orientation.x,
			p1.orientation.y,
			p1.orientation.z,
			p1.orientation.w);
	tf::Quaternion q2(p2.orientation.x,
			p2.orientation.y,
			p2.orientation.z,
			p2.orientation.w);

	tf::Matrix3x3(q1).getRPY(ro1, pi1, ya1);
	tf::Matrix3x3(q2).getRPY(ro2, pi2, ya2);

	return normalizePi(ya1-ya2);
}
}
#endif /* UTILITIES_H_ */
