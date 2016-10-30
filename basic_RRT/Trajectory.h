#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <vector>

#include <Dense>
using namespace Eigen;

#include "tinysplinecpp.h"

#include "Collision_check.h"
#include "RRT.h"
#include "Vehicle.h"

namespace Trajectory
{
	void norm_theta_2pi(float *theta);
	void norm_theta_pi(float *theta);
	float dist(const Vehicle::Node &n1, const Vehicle::Node &n2);
	float dist(const float &x1, const float &y1, const float &x2, const float &y2);
//	bool find_theta(const float &l) {return }

	struct State
	{
		State() = default;
		State(const VectorXf &Xi)
		{
			node.x = Xi[0];
			node.y = Xi[1];
			node.theta = Xi[2];
			node.k = Xi[3];
			v = Xi[4];
			s_init = 0;
		}
		State(const float &x0, const float &y0, const float &t0, const float &k0)
		{
			node.x = x0;
			node.y = y0;
			node.theta = t0;
			node.k = k0;
		}
		State(const float &x0, const float &y0, const float &t0, const float &k0, const float &s)
		{
			node.x = x0;
			node.y = y0;
			node.theta = t0;
			node.k = k0;
			s_init = s;
		}
		State(const float &x0, const float &y0)
		{
			node.x = x0;
			node.y = y0;
		}
		State(const State &s) :node(s.node), v(s.v), s_init(s.s_init) {}

		Vehicle::Node* _node() { return &node; }
		float _v() const { return v; }
		float _s_init() const { return s_init; }
		float _theta() const { return node.theta; }
		float _k() const { return node.k; }

		void _theta(const float &t) { node.theta = t; }
		void _k(const float &k0) { node.k = k0; }
		void _node(const float &x0, const float &y0, const float &t0, const float &k0)
		{
			node.x = x0;
			node.y = y0;
			node.theta = t0;
			node.k = k0;
		}
		void _node(const Vehicle::Node &n)
		{
			node.x = n.x;
			node.y = n.y;
			node.theta = n.theta;
			node.k = n.k;
		}
		void _v(const float &v0) { v = v0; }
		void _s_init(const float &s) { s_init = s; }

	private:
		Vehicle::Node node;
		float v, s_init;
	};

	struct traj
	{
		traj()
		{
 			knots = { 1.f, 0.7f, 0.7f, 0.4f, 0.4f, 0.1f, 0.1f, 0.f };
		}
		traj(const VectorXf &Xi, const VectorXf &Xg)
		{
			ctrl_points.emplace_back(Xi[0], Xi[1]);
			//knots = { 1., 0.7, 0.7, 0.4, 0.4, 0.1, 0.1, 0. };
			knots = { 1., 0. };
			bound.emplace_back(Xi);
			bound.emplace_back(Xg);
			v_tmp_dest.emplace_back(Xg[4]);
			_v_tmp_dest(Xi[5]);
		}
		
		void _ctrl_points(const std::vector<Vehicle::Node> &route_tree, const float &step, Collision::collision *collimap);
		void _bspline();
		void _state(float *accel);
		std::vector<State>* _state(const float &a0, const float &sg); //in practical when sg<state.end-10, calculate state_future
		void _v_tmp_dest(const float &accel);

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		std::vector<State>* _state_now() { return &state_now; }
		std::vector<State>* _state_future() { return &state_future; }

	private:
		struct Point
		{
			float x, y;

			Point() = default;
			Point(const float &x0, const float &y0) :x(x0), y(y0) {}
			Point(const Vehicle::Node &node) :x(node.x), y(node.y) {}
		};
		std::vector<Point> ctrl_points;
		std::vector<float> v_tmp_dest;// in practical 
		std::vector<State> state_now;
		std::vector<State> state_future; //in practical
		std::vector<float> knots; //in practical
		ts::BSpline bspline;
		std::vector<State> bound;

	public:
		std::vector<Point>* _ctrl_points() { return &ctrl_points; }
	};
}

#endif