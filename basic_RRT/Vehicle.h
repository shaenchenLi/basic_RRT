#ifndef VEHICLE_H
#define VEHICLE_H

#include <cv.h>
#include <Dense>
using namespace Eigen;

namespace Vehicle
{
	// key value of vehicle	
	const float PI = CV_PI;// 3.141592653589793;
	const float TWO_PI = 2 * PI;
	const float L = 3.569f;
	const float W = 1.551f;
	const float LA = 2.305f;
	const float DL = 0.5f; // the distance between the rear axle center and end of vehicle
	const float H = 1.5f;
	const float XMAX = 100.0f;
	const float XMIN = 0.0f;
	const float YMIN = 0.0f;
	const float YMAX = 15.0f;
	const float THETAMAX = PI;
	const float THETAMIN = -PI;
	const float KMAX = 0.26f;
	const float KMIN = -0.26f;
	const float ALATER = 0.4f*9.8f;
	const float VMAX = 20.0f;
	
	struct Node
	{
		Node() = default;
		Node(const float &x0, const float &y0, const float &theta0, const float &k0) :x(x0), y(y0), theta(theta0), k(k0) {}
		Node(const VectorXf &X) :x(X[0]), y(X[1]), theta(X[2]), k(X[3]) {}
		Node(const Node &n) :x(n.x), y(n.y), theta(n.theta), k(n.k) {}
		Node(const Node *n)
		{
			x = n->x;
			y = n->y;
			theta = n->theta;
			k = n->k;
		}

		inline void reset(const float &x0, const float &y0)
		{
			x = x0;
			y = y0;
		}
		inline void reset(const float &x0, const float &y0, const float &theta0)
		{
			reset(x0, y0);
			reset(theta0);
		}
		inline void reset(const float &theta0)
		{
			theta = theta0;
		}
		bool operator==(Node &n)
		{
			return (x == n.x) && (y = n.y) && (theta = n.theta) && (k = n.k);
		}
		bool operator!=(Node &n)
		{
			return !(*this == n);
		}

		float x, y, theta, k;
	};
}

#endif