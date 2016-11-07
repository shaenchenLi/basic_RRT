#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
using std::cout; using std::endl;

#include <cv.h>
#include <Dense>
using namespace Eigen;

#define PI CV_PI
#define TWO_PI (2*PI)
#define L 3.569f
#define W 1.551f
#define H 1.5f
#define LA 2.305f
#define DL 0.5f
#define XMAX 100.0f
#define XMIN 0.0f
#define YMAX 15.0f
#define YMIN 0.0f
#define THETAMAX PI
#define THETAMIN (-PI)
#define KMAX 0.26f
#define KMIN (-0.26f)
#define ALATER 0.4f*9.8f
#define VMAX 20.f
#define WMAX 1.56 //rad/s

namespace Vehicle
{
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

	bool is_node_effect(const Vehicle::Node &new_node);

}

#endif