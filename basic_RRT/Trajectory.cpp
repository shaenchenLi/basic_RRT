#include "Trajectory.h"

float eps = 1e-6f;

void Trajectory::norm_theta_0_2pi(float *theta)
{
	*theta = std::fmod(*theta, Vehicle::TWO_PI);
	if (*theta < -Vehicle::PI)
		*theta += Vehicle::TWO_PI;
	else if (*theta>Vehicle::PI)
		*theta -= Vehicle::TWO_PI;
}

void Trajectory::norm_theta_2pi(float *theta)
{
	norm_theta_0_2pi(theta);
	if (*theta > Vehicle::PI)
		*theta -= 2 * Vehicle::PI;
}

void Trajectory::norm_theta_pi(float *theta)
{
	norm_theta_0_2pi(theta);
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

float Trajectory::dist(const Point &p)
{
	return sqrtf(powf(p.x, 2) + powf(p.y, 2));
}

//Trajectory::Point diff(Trajectory::State *s1, Trajectory::State *s2)
//{
//	return Trajectory::Point(s1->_node()->x - s2->_node()->x, s1->_node()->y - s2->_node()->y);
//}
//
//Trajectory::Point diff(Trajectory::State *s1, Trajectory::State *s2, Trajectory::State *s3)
//{
//	Trajectory::Point p1 = diff(s1, s2);
//	Trajectory::Point p2 = diff(s2, s3);
//	return Trajectory::Point(p1.x - p2.x, p1.y - p2.y);
//}

void Trajectory::traj::_ctrl_points(const std::vector<Vehicle::Node> &route_tree, const float &step, Collision::collision *collimap)
{
	Matrix<float, 6, 2> L_theta;
	for (int i = 1; i < 6;i++) //float t = Vehicle::PI / 3; t <= 5 * Vehicle::PI / 6; t += 0.1*Vehicle::PI)
	{
		L_theta(i - 1, 1) = Vehicle::PI / 3 + i*0.1f*Vehicle::PI;
		L_theta(i - 1, 0) = sinf(L_theta(i - 1, 1)) / std::min(Vehicle::KMAX, std::abs(Vehicle::KMIN))*powf((1 - cosf(L_theta(i - 1, 1))) / 8, -1.5) / 6;
		//theta_min_candidate.emplace_back(t);
		//L_min_candidate.emplace_back(sinf(t) / std::min(Vehicle::KMAX, std::abs(Vehicle::KMIN))*powf((1 - cosf(t)) / 8, -1.5) / 6);
	}// to satisfy the maximum */
	L_theta(5, 0) = 0;
	L_theta(5, 1) = Vehicle::PI;	

	float x, y, theta;

	std::vector<Vehicle::Node> tree_tmp = route_tree;
	std::vector<Vehicle::Node> route = { *tree_tmp.begin() };
	for (auto node = tree_tmp.begin(); node != tree_tmp.end(); node++)
	{
		if (node == tree_tmp.begin())
		{
			x = tree_tmp.begin()->x + 3.f * cos(tree_tmp.begin()->theta);
			y = tree_tmp.begin()->y + 3.f * sin(tree_tmp.begin()->theta);
			route.emplace_back(x, y, tree_tmp.begin()->theta, 0.f);
			(tree_tmp.begin() + 1)->reset(atan2f(node->y - y, node->x - x));
			continue;
		}
		/*else if (node == route_tree.begin() + 1)
			route.emplace_back(node->x, node->y, atan2f(node->y - y, node->x - x),node->k);*/
		else if (node == tree_tmp.end() - 1)
		{
			x = tree_tmp.rbegin()->x - 3.f * cos(tree_tmp.rbegin()->theta);
			y = tree_tmp.rbegin()->y - 3.f * sin(tree_tmp.rbegin()->theta);
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
	//float theta_min/*, L_min*/;
	int flag = 0; // 0:node++ 1:insert_node 2:end
	auto node = route.begin();
	Vehicle::Node *insert_node_tmp;
	Vehicle::Node *node_behind = &*(route.rbegin());
	Vehicle::Node *node_tmp = &*node;
	Vehicle::Node *insert_node = &*node;
	std::vector<Vehicle::Node> before_end_insert_node;
	while (flag!=2)
	//for (auto node = route.begin() + 1; node != route.end() - 1; node++)
	{
		if (node == route.end() - 3 || node == route.end() - 2) // the end of the ctrlpoints
		{
			if (flag == 0)
				node_tmp = &*(++node);

			float theta = node_tmp->theta - (node_behind)->theta;
			norm_theta_pi(&theta);
			float L = std::min(dist(reference_node, *node_tmp), dist(*node_tmp, *(node_behind)));
			if (L < (sinf(theta) / std::min(Vehicle::KMAX, std::abs(Vehicle::KMIN))*powf((1 - cosf(theta)) / 8, -1.5) / 6))
			{
				flag = -1;
				float L_tmp, L_diff_tmp, L_diff = HUGE_VALF;
				//float dist_refer_to_node = dist(*node_tmp, *(node + 1));
				for (int i = 0; i < 5; i++)
				{
					if (L_theta(i, 0) > dist(*node_tmp, *(node_behind)))
					{
						if (i == 4)
						{
							flag = 1;
							insert_node = &Vehicle::Node(node_tmp->x + L_theta(i, 0)*cosf(node_tmp->theta), node_tmp->y + L_theta(i, 0)*sinf(node_tmp->theta), node_tmp->theta, 0.f);
							break;
						}
						else
							continue;
					}
					theta = node_tmp->theta - (node_behind)->theta;
					norm_theta_2pi(&theta);
					float insert_theta = (node_behind)->theta - theta / std::abs(theta)*L_theta(i, 1);
					float insert_x = node_tmp->x + L_theta(i, 0)*cosf(insert_theta);
					float insert_y = node_tmp->y + L_theta(i, 0)*sinf(insert_theta);
					insert_node_tmp = &Vehicle::Node(insert_x, insert_y, atan2f(insert_y - reference_node.y, insert_x - reference_node.x), 0.f);
					float new_theta = insert_node_tmp->theta - insert_theta;
					norm_theta_pi(&new_theta); // insert node's theta
					if (Vehicle::is_node_effect(insert_node_tmp) && (!collimap->iscollision(*node_tmp, insert_node_tmp)) && (!collimap->iscollision(insert_node_tmp, *node_behind)))
					{
						L_tmp = sinf(new_theta) / std::min(Vehicle::KMAX, std::abs(Vehicle::KMIN))*powf((1 + cosf(new_theta)) / 8, -1.5) / 6;
						if (std::min(L_theta(i, 0), dist(insert_x, insert_y, reference_node.x, reference_node.y)) < L_tmp)
						{
							flag = 1;
							L_diff_tmp = L_tmp - std::min(L_theta(i, 0), dist(insert_x, insert_y, node_behind->x, node_behind->y));
							if (L_diff_tmp < L_diff)
							{
								insert_node = insert_node_tmp;
								L_diff = L_diff_tmp;
							}
						}
						else
						{
							flag = 2;
							insert_node = insert_node_tmp;
							break;
						}
					}
				}
				if (flag == -1)
					break;
				else if (flag == 2)
				{
					before_end_insert_node.emplace_back(insert_node);
					node_behind->reset(atan2f(node_behind->y - insert_node->y, node_behind->x - insert_node->x));
					break;
				}
				else
				{
					before_end_insert_node.emplace_back(insert_node);
					node_behind->reset(atan2f(node_behind->y - insert_node->y, node_behind->x - insert_node->x));
					node_behind = node_tmp;
					node_tmp = insert_node;
					continue;
				}
			}
			else
			{
				if (before_end_insert_node.empty())
				{
					ctrl_points.emplace_back(0.5f*(reference_node.x + node_tmp->x), 0.5f*(reference_node.y + node_tmp->y));
					ctrl_points.emplace_back(*node_tmp);
				}
				else
				{					
					for (auto n = before_end_insert_node.rbegin(); n != before_end_insert_node.rend(); n++)
					{
						ctrl_points.emplace_back(0.5*(reference_node.x + n->x), 0.5*(reference_node.y + n->y));
						ctrl_points.emplace_back(*n);
						reference_node = *n;
					}
					ctrl_points.emplace_back(0.5*(before_end_insert_node.begin()->x + (route.end() - 2)->x), 0.5*(before_end_insert_node.begin()->y + (route.end() - 2)->y));
					ctrl_points.emplace_back(*(route.end() - 2));
				}
				flag = 2;
				break;
			}
		}

		if (flag == 0)
		{
			node_tmp = &*(++node);
			ctrl_points.emplace_back(0.5f*(reference_node.x + node_tmp->x), 0.5f*(reference_node.y + node_tmp->y));
			ctrl_points.emplace_back(*node_tmp);
		}			
		else
			node_tmp = insert_node;
				
		float theta = node_tmp->theta - (node + 1)->theta;
		norm_theta_pi(&theta);
		float L = std::min(dist(reference_node, *node_tmp), dist(*node_tmp, *(node + 1)));
		if (L < (sinf(theta) / std::min(Vehicle::KMAX, std::abs(Vehicle::KMIN))*powf((1 - cosf(theta)) / 8, -1.5) / 6))
		{
			flag = -1;
			float L_tmp, L_diff_tmp, L_diff = HUGE_VALF;
			//float dist_refer_to_node = ;
			for (int i = 0; i < 5; i++)
			{
				if (L_theta(i, 0) > dist(reference_node, *node_tmp))
				{
					if (i == 4)
					{
						insert_node = &Vehicle::Node(node_tmp->x + L_theta(i, 0)*cosf(node_tmp->theta), node_tmp->y + L_theta(i, 0)*sinf(node_tmp->theta), node_tmp->theta, 0.f);
						flag = 1;
						break;
					}
					else
						continue;
				}
				float insert_theta = (node_tmp->theta - Vehicle::PI) + theta / std::abs(theta)* L_theta(i, 1);
				norm_theta_0_2pi(&insert_theta);
				float insert_x = node_tmp->x + L_theta(i, 0)*cosf(insert_theta);
				float insert_y = node_tmp->y + L_theta(i, 0)*sinf(insert_theta);
				float new_theta = insert_theta - atan2f((node + 1)->y - insert_y, (node + 1)->x - insert_x); // insert node's theta
				insert_node_tmp = &Vehicle::Node(insert_x, insert_y, insert_theta, 0.f);
				if (Vehicle::is_node_effect(insert_node_tmp) && (!collimap->iscollision(*node_tmp, insert_node_tmp)) && (!collimap->iscollision(insert_node_tmp, *(node + 1))))
				{
					L_tmp = sinf(new_theta) / std::min(Vehicle::KMAX, std::abs(Vehicle::KMIN))*powf((1 - cosf(new_theta)) / 8, -1.5) / 6;
					if (std::min(L_theta(i, 0), dist(insert_x, insert_y, (node + 1)->x, (node + 1)->y)) < L_tmp)
					{
						flag = 1;
						L_diff_tmp = L_tmp - std::min(L_theta(i, 0), dist(insert_x, insert_y, (node + 1)->x, (node + 1)->y));
						if (L_diff_tmp < L_diff)
						{
							insert_node = insert_node_tmp;
							L_diff = L_diff_tmp;
						}
					}
					else
					{
						flag = 0;
						insert_node = insert_node_tmp;
						break;
					}
				}
			}
			if (flag == -1)
			{
				flag = 0;
				reference_node = *node;
			}
			else
			{
				(node + 1)->reset(atan2f((node + 1)->y - insert_node->y, (node + 1)->x - insert_node->x));
				ctrl_points.emplace_back(0.5f*(node_tmp->x + insert_node->x), 0.5f*(node_tmp->y + insert_node->y));
				ctrl_points.emplace_back(insert_node);
				reference_node = insert_node;
			}
		}
		//ctrl_points.emplace_back(0.5f*(ctrl_points.rbegin()->x + (node + 1)->x), 0.5f*(ctrl_points.rbegin()->y + (node + 1)->y));
		//ctrl_points.emplace_back(*(node + 1));
		else
		{
			reference_node = *node_tmp;
			flag = 0;
		}
	}

	ctrl_points.emplace_back(0.5f*(route_tree.rbegin()->x + ctrl_points.rbegin()->x), 0.5f*(route_tree.rbegin()->y + ctrl_points.rbegin()->y));
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

	float du = 0.001f;
	State old_1, old_2;
	Point diff_1, diff_2, old_diff;
	for (auto u = u0[0]; u < u0[1]; u += du)
	{
		if (state_future.empty())
		{
			if (state_now.empty())
			{
				state_future.emplace_back(*bound.begin());
				old_2 = State(bound.begin()->_node()->x - du*cosf(bound.begin()->_node()->theta), bound.begin()->_node()->y - du*sinf(bound.begin()->_node()->theta));
				old_diff.reset(du*cosf(bound.begin()->_node()->theta), du*sinf(bound.begin()->_node()->theta));
				continue;
			}
			else
			{
				old_1 = *state_now.rbegin();
				old_2 = *(state_now.rbegin() + 1);
				old_diff.reset(old_1._node()->x - old_2._node()->x, old_1._node()->y - old_2._node()->y);
			}
		}

		auto result = bspline.evaluate(u).result();
		old_1 = *state_future.rbegin();
		diff_1.reset(result[0] - old_1._node()->x, result[1] - old_1._node()->y);
		diff_2.reset(diff_1.x - old_diff.x, diff_1.y - old_diff.y);
		float theta = atan2f(diff_1.x, diff_1.y);
		float k = (diff_1.x*diff_2.y - diff_2.x*diff_1.y) / powf(dist(diff_1), 1.5);
		float s = dist(result[0], result[1], old_1._node()->x, old_1._node()->y);
		state_future.emplace_back(result[0], result[1], theta, k, s + old_1._s_init());
		old_diff = diff_1;
		old_2 = old_1;
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