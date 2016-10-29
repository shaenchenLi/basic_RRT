#include "Trajectory.h"

float eps = 1e-6;

void Trajectory::norm_theta_2pi(float *theta)
{
	*theta = std::fmod(*theta, Vehicle::TWO_PI);
	if (*theta < -Vehicle::PI)
		*theta += Vehicle::TWO_PI;
	else if (*theta>Vehicle::PI)
		*theta -= Vehicle::TWO_PI;
}

void Trajectory::norm_theta_pi(float *theta)
{
	norm_theta_2pi(theta);
	if (*theta > 0)
		*theta = Vehicle::PI - *theta;
	else
		*theta = Vehicle::PI + *theta;
}

void Trajectory::traj::_ctrl_points(const std::vector<Vehicle::Node> &route_tree, const float &step, Collision::collision *collimap)
{
	float theta_min = atanf(Vehicle::LA*(std::min(Vehicle::KMAX, std::abs(Vehicle::KMIN)))) + 0.01*Vehicle::PI;

	float x, y;

	std::vector<Vehicle::Node> tree_tmp = route_tree;
	std::vector<Vehicle::Node> route = { *tree_tmp.begin() };
	for (auto node = tree_tmp.begin(); node != tree_tmp.end(); node++)
	{
		if (node == tree_tmp.begin())
		{
			x = tree_tmp.begin()->x + 0.5f*step*cos(tree_tmp.begin()->theta);
			y = tree_tmp.begin()->y + 0.5f*step*sin(tree_tmp.begin()->theta);
			while (collimap->iscollision(x, y, tree_tmp.begin()->theta))
			{
				x = 0.5f*(x + tree_tmp.begin()->x);
				y = 0.5f*(y + tree_tmp.begin()->y);
			}
			route.emplace_back(x, y, tree_tmp.begin()->theta, 0.f);
			(tree_tmp.begin() + 1)->reset(atan2f(node->y - y, node->x - x));
			continue;
		}
		else if (node == tree_tmp.end() - 1)
		{
			x = tree_tmp.rbegin()->x - 0.5f*step*cos(tree_tmp.rbegin()->theta);
			y = tree_tmp.rbegin()->y - 0.5f*step*sin(tree_tmp.rbegin()->theta);
			float theta = atan2f(y - route.rbegin()->y, x - route.rbegin()->x);
			while (collimap->iscollision(x, y, theta))
			{
				x = 0.5f*(x + tree_tmp.rbegin()->x);
				y = 0.5f*(y + tree_tmp.rbegin()->y);
				theta = atan2f(y - (node - 1)->theta, x - (node - 1)->theta);
			}

			float unit_dis = 2.f; //distance between two sampled used to detect collision
			float dis = sqrtf(powf(route.rbegin()->x - x, 2) + powf(route.rbegin()->y - y, 2));
			for (int s = unit_dis; s < dis; s += unit_dis)
			{
				if (collimap->iscollision(route.rbegin()->x + s*cosf(theta), route.rbegin()->y + s*sinf(theta), theta))
					route.emplace_back(*(node - 1));
			}
			route.emplace_back(x, y, theta, 0.f);
			route.emplace_back(*tree_tmp.rbegin());
			continue;
		}
		
		float unit_dis = 2.f; //distance between two sampled used to detect collision
		float dis = sqrtf(powf(route.rbegin()->x - node->x, 2) + powf(route.rbegin()->y - node->y, 2));
		float theta = atan2f(node->y - route.rbegin()->y, node->x - route.rbegin()->x);
		for (int s = unit_dis; s < dis; s += unit_dis)
		{
			if (collimap->iscollision(route.rbegin()->x + s*cosf(theta), route.rbegin()->y + s*sinf(theta), theta))
			{
				if (*(node - 1) != *route.rbegin())
					route.emplace_back(*(node - 1));
				route.emplace_back(*node);
			}
		}
	}
	
	for (auto node = route.begin() + 1; node != route.end() - 1; node++)
	{
		float theta = node->theta - (node + 1)->theta;
		Trajectory::norm_theta_pi(&theta);

		if (theta < theta_min)
		{
			float delta;
			delta = theta_min - theta;
			Vehicle::Node reference_node = *(node - 1);
			//int flag, action;
			if (node->x>reference_node.x)
			{
				x = (reference_node.x*cos(delta) - reference_node.y*sin(delta)) / 5;
				y = (reference_node.x*sin(delta) + reference_node.y*cos(delta)) / 5;
			} //reverse clock
			else
			{
				x = (reference_node.x*cos(delta) + reference_node.y*sin(delta)) / 5;
				y = (-reference_node.x*sin(delta) + reference_node.y*cos(delta)) / 5;
			}
			int insert=0; //0: failed 1:success
			for (int i = 1; i < 10; i++)
			{
				float theta1 = atan2f(y - reference_node.y, x - reference_node.x);
				float theta2 = atan2f(node->y - y, node->x - x);
				float alfa1 = theta1 - theta2;
				float alfa2 = reference_node.theta - theta1;
				Trajectory::norm_theta_pi(&alfa1);
				Trajectory::norm_theta_pi(&alfa2);
				if (alfa1 > theta_min && alfa2 > theta_min && (!collimap->iscollision(x, y, theta1)))
				{
					insert = 1;
					ctrl_points.emplace_back(x, y);
					ctrl_points.emplace_back(*node);
					break;
				}
				x = 0.5f*(x + node->x);
				y = 0.5f*(y + node->y);
			}			
			if (insert == 0)
				(node + 1)->theta = atan2f((node + 1)->y - reference_node.y, (node + 1)->x - reference_node.x);
		}
		else
			ctrl_points.emplace_back(*node);
	}

	ctrl_points.emplace_back(*route_tree.rbegin());
}

void Trajectory::traj::_bspline()
{

	auto n = ctrl_points.size();

	if (n > 5)
		bspline = ts::BSpline(3, 2, 3 * n, TS_CLAMPED);
	else
		bspline = ts::BSpline(n - 3, 2, (n - 3)*n, TS_CLAMPED);

	std::vector<ts::rational> ctrlp = bspline.ctrlp();
	for (std::vector<Point>::size_type i = 1; i < n;i++)
	{
		ctrlp[2 * i] = ctrl_points[i].x;
		ctrlp[2 * i + 1] = ctrl_points[i].y;		
	}
	bspline.setCtrlp(ctrlp);
}

void Trajectory::traj::_state(float *accel)
{
	float u0[2];
	for (auto &i : u0)
	{
		i = *knots.rbegin();
		knots.pop_back();
	}

	State old;
	for (auto u = u0[0]; u < u0[1]; u += 0.001f)
	{
		if (state_future.empty())
		{
			if (state_now.empty())
			{
				state_future.emplace_back(*bound.begin());
				continue;
			}
			else
				old = *state_now.rbegin();
		}
		else
			old = *state_future.rbegin();

		auto result = bspline.evaluate(u).result();
		float theta = atan2f(result[1] - old._node()->y, result[0] - old._node()->x);
		float k = tanf(theta);
		float s = sqrtf(pow((result[1] - old._node()->y), 2) + pow((result[0] - old._node()->x), 2));
		state_future.emplace_back(result[0], result[1], theta, k, s + old._s_init());
	}
	
	float s = state_future.rbegin()->_s_init() - state_future.begin()->_s_init();
	float vg = *v_tmp_dest.rbegin();
	v_tmp_dest.pop_back();
	Vector4f coeff_v;

	coeff_v[0] = state_now.empty() ? bound.begin()->_v() : state_now.rbegin()->_v();
	coeff_v[1] = *accel;

	float vv = 2 * coeff_v(0) + vg;
	float delta = vv*vv + 6 * coeff_v[1]*s;
	if (delta > -eps)
	{
		if (abs(coeff_v[1]) <= eps)
		{
			if (vv == 0)
			{
				if (coeff_v[1] > eps)
					coeff_v[3] = sqrtf(6 * s / coeff_v[1]);
			}
			else
				coeff_v[3] = 3 * s / vv;
		}

		else
			coeff_v[3] = (sqrtf(std::max(0.f, delta)) - vv) / coeff_v[1];
	}

	if (coeff_v[3] > 0)
		coeff_v[2] = (vg - coeff_v[0] - coeff_v[3] * coeff_v[1]) / (coeff_v[3] * coeff_v[3]);
	else
	{
		for (auto &i : state_future)
			i._v(std::min(coeff_v[0], std::min(sqrtf(Vehicle::ALATER / abs(i._k())), Vehicle::VMAX)));
			
		*accel = 0;
		return;
	}

	int N = state_future.size();

	ArrayXXf times = VectorXf::LinSpaced(N, 0.f, coeff_v[3]);
	ArrayXXf lengths = times*(coeff_v[0] + times*(coeff_v[1] / 2.f + times*coeff_v[2] / 3.f));

	int j = 1;
	float t = 0.f;
	for (auto &i:state_future)
	{
		while (i._s_init() >= lengths(j, 0) && j < N - 1)
			j++;

		t = (times(j - 1, 0) + (i._s_init() - lengths(j - 1, 0))*(times(j, 0) - times(j - 1, 0)) / (lengths(j, 0) - lengths(j - 1, 0)));
		i._v(std::min(coeff_v[0] + coeff_v[1] * t + coeff_v[2] * pow(t, 2), std::min(sqrtf(Vehicle::ALATER / abs(i._k())), Vehicle::VMAX)));
	}

	*accel = 2 * coeff_v[0] * t + coeff_v[1];
}

void Trajectory::traj::_v_tmp_dest(const float &accel)
{
	float s = sqrtf(pow((bound[0]._node()->x - bound[1]._node()->x), 2) + pow((bound[0]._node()->y - bound[1]._node()->y), 2));
	auto N = knots.size();

	Vector4f coeff_v;

	coeff_v[0] = bound[0]._v();
	coeff_v[1] = accel;

	float vv = 2 * coeff_v(0) + bound[1]._v();
	float delta = vv*vv + 6 * coeff_v[1] * s;
	if (delta > -eps)
	{
		if (abs(coeff_v[1]) <= eps)
		{
			if (vv == 0)
			{
				if (coeff_v[1] > eps)
					coeff_v[3] = sqrtf(6 * s / coeff_v[1]);
			}
			else
				coeff_v[3] = 3 * s / vv;
		}

		else
			coeff_v[3] = (sqrtf(std::max(0.f, delta)) - vv) / coeff_v[1];
	}

	if (coeff_v[3] > 0)
		coeff_v[2] = (bound[1]._v() - coeff_v[0] - coeff_v[3] * coeff_v[1]) / (coeff_v[3] * coeff_v[3]);
	else
	{
		for (int i = 1; i <= N / 2 - 1; i++)
			v_tmp_dest.emplace_back(coeff_v[0]);
		return;
	}

	ArrayXXf times = VectorXf::LinSpaced(N, 0., coeff_v[3]);
	ArrayXXf lengths = times*(coeff_v[0] + times*(coeff_v[1] / 2. + times*coeff_v[2] / 3.));

	int j = 1;
	float t = 0;
	for (int i = N - 1; i > 0; i -= 2)
	{
		while (knots[i] * s >= lengths(j, 0) && j < N - 1)
			j++;

		t = (times(j - 1, 0) + (knots[i] * s - lengths(j - 1, 0))*(times(j, 0) - times(j - 1, 0)) / (lengths(j, 0) - lengths(j - 1, 0)) + t);
		v_tmp_dest.push_back(coeff_v[0] + coeff_v[1] * t + coeff_v[2] * pow(t, 2));
	}
	std::reverse(v_tmp_dest.begin(), v_tmp_dest.end());
}

std::vector<Trajectory::State>* Trajectory::traj::_state(const float &a0, const float &sg)
{
	float accel = a0;
	if (sg < (state_now.cbegin() + 9)->_s_init())
		_state(&accel);
	if (sg == (state_now.cbegin() + 1)->_s_init())
	{
		state_now.clear();
		state_now.resize(state_future.size());
		state_now = state_future;
		state_future.clear();
	}
	return &state_now;
}