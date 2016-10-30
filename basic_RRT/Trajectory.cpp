#include "Trajectory.h"

float eps = 1e-6f;

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

float Trajectory::dist(const Vehicle::Node &n1, const Vehicle::Node &n2)
{
	return sqrtf(powf(n1.x - n2.x, 2) + powf(n1.y - n2.y, 2));
}

float Trajectory::dist(const float &x1, const float &y1, const float &x2, const float &y2)
{
	return sqrtf(powf(x1 - x2, 2) + powf(y1 - y2, 2));
}
void Trajectory::traj::_ctrl_points(const std::vector<Vehicle::Node> &route_tree, const float &step, Collision::collision *collimap)
{
	Matrix<float, 6, 2> L_theta;
	for (int i = 0; i < 6;i++) //float t = Vehicle::PI / 3; t <= 5 * Vehicle::PI / 6; t += 0.1*Vehicle::PI)
	{
		L_theta(i, 1) = Vehicle::PI / 3 + i*0.1f*Vehicle::PI;
		L_theta(i, 0) = sinf(L_theta(i, 1)) / std::min(Vehicle::KMAX, std::abs(Vehicle::KMIN))*powf((1 - cosf(L_theta(i, 1))) / 8, -1.5) / 6;
		
	}// to satisfy the maximum 

	float x, y, theta;

	std::vector<Vehicle::Node> tree_tmp = route_tree;
	std::vector<Vehicle::Node> route = { *tree_tmp.begin() };
	for (auto node = tree_tmp.begin(); node != tree_tmp.end(); node++)
	{
		if (node == tree_tmp.begin())
		{
			x = tree_tmp.begin()->x + 5 * cos(tree_tmp.begin()->theta);
			y = tree_tmp.begin()->y + 5 * sin(tree_tmp.begin()->theta);
			route.emplace_back(x, y, tree_tmp.begin()->theta, 0.f);
			(tree_tmp.begin() + 1)->reset(atan2f(node->y - y, node->x - x));
			continue;
		}
		/*else if (node == route_tree.begin() + 1)
			route.emplace_back(node->x, node->y, atan2f(node->y - y, node->x - x),node->k);*/
		else if (node == tree_tmp.end() - 1)
		{
			x = tree_tmp.rbegin()->x - 5 * cos(tree_tmp.rbegin()->theta);
			y = tree_tmp.rbegin()->y - 5 * sin(tree_tmp.rbegin()->theta);
			theta = atan2f(y - route.rbegin()->y, x - route.rbegin()->x);
			float unit_dis = 2.f; //distance between two sampled used to detect collision
			float dis = dist(route.rbegin()->x, route.rbegin()->y, x, y);
			for (float s = unit_dis; s < dis; s += unit_dis)
			{
				if (collimap->iscollision(route.rbegin()->x + s*cosf(theta), route.rbegin()->y + s*sinf(theta), theta))
					route.emplace_back(*(node - 1));
			}
			route.emplace_back(x, y, theta, 0.f);
			route.emplace_back(*tree_tmp.rbegin());
			continue;
		}
		
		float unit_dis = 2.f; //distance between two sampled used to detect collision
		float dis = dist(*route.rbegin(), *node);
		float theta = atan2f(node->y - route.rbegin()->y, node->x - route.rbegin()->x);
		for (float s = unit_dis; s < dis; s += unit_dis)
		{
			if (collimap->iscollision(route.rbegin()->x + s*cosf(theta), route.rbegin()->y + s*sinf(theta), theta))
			{
				if (*(node - 1) != *route.rbegin())
					route.emplace_back(*(node - 1));
				route.emplace_back(*node);
			}
		}
	}

	Vehicle::Node reference_node = *(route.begin());
	float theta_min/*, L_min*/;
	for (auto node = route.begin() + 1; node != route.end() - 1; node++)
	{
		float theta = node->theta - (node + 1)->theta;
		norm_theta_pi(&theta);
		float L = std::min(dist(reference_node, *node), dist(*node, *(node + 1)));
		for (int i = 0; i < 6; i++)
		{
			if (L_theta(i, 0) < L)
			{
				theta_min = L_theta(i, 1);
				//L_min = L_theta(i, 0);
				break;
			}
		}
		if (theta < theta_min)
		{
			float delta;
			delta = L_theta(5,1) - theta;
			//float L = dist(reference_node, *node);
			//int flag, action;
			if (node->x>reference_node.x)
			{
				x = (reference_node.x*cos(delta) - reference_node.y*sin(delta)) *L_theta(5, 0) / 5 + (L - L_theta(5, 0)) / L*node->x;
				y = (reference_node.x*sin(delta) + reference_node.y*cos(delta)) *L_theta(5, 0) / 5 + (L - L_theta(5, 0)) / L*node->y;
			} //reverse clock
			else
			{
				x = (reference_node.x*cos(delta) + reference_node.y*sin(delta)) *L_theta(5, 0) / 5 + (L - L_theta(5, 0)) / L*node->x;
				y = (-reference_node.x*sin(delta) + reference_node.y*cos(delta)) *L_theta(5, 0) / 5 + (L - L_theta(5, 0)) / L*node->y;
			}			
			//if (/*alfa1 > theta_min && alfa2 > theta_min && */(!collimap->iscollision(x, y, atan2f(y - reference_node.y, x - reference_node.x))))
			if ((!collimap->iscollision(x, y, atan2f(y - reference_node.y, x - reference_node.x))))
			{
				ctrl_points.emplace_back(0.5*(x + reference_node.x), 0.5*(y + reference_node.y));
				ctrl_points.emplace_back(x, y);
				ctrl_points.emplace_back(0.5*(x + node->x), 0.5*(y + node->y));
				ctrl_points.emplace_back(*node);
				reference_node = *node;
			}
		}
		else
		{
			ctrl_points.emplace_back(0.5*(node->x + reference_node.x), 0.5*(node->y + reference_node.y));
			ctrl_points.emplace_back(*node);
			reference_node = *node;
		}
	}

	ctrl_points.emplace_back(*route_tree.rbegin());
}

void Trajectory::traj::_bspline()
{

	auto n = ctrl_points.size();

	if (n > 5)
		bspline = ts::BSpline(3, 2, n, TS_CLAMPED);
	else
		bspline = ts::BSpline(n - 3, 2, n, TS_CLAMPED);

	std::vector<ts::rational> ctrlp = bspline.ctrlp();
	
	for (std::vector<Point>::size_type i = 0; i < n;i++)
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
		float s = dist(result[0], result[1], old._node()->x, old._node()->y);
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
	float s = dist(bound[0]._node(), bound[1]._node());
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
	ArrayXXf lengths = times*(coeff_v[0] + times*(coeff_v[1] / 2.f + times*coeff_v[2] / 3.f));

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