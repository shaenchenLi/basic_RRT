#include "RRT.h"
using std::cout;
using std::endl;

void RRT::basic_RRT::kd_tree(Index* kd_father, const Vehicle::Node &new_node)
{
	*kd_father = 0;
	Index old = 0;
 	while (*kd_father != max_size+1)
	{
		old = *kd_father;
		switch (tree[*kd_father]._deep())
		{
		case 0:
			//cout << norm(new_node.x, Vehicle::XMAX, Vehicle::XMIN) << " " << norm(tree[*kd_father]._node()->x, Vehicle::XMAX, Vehicle::XMIN) << endl;
			if (norm(new_node.x, Vehicle::XMAX, Vehicle::XMIN) > norm(tree[*kd_father]._node()->x, Vehicle::XMAX, Vehicle::XMIN))
				*kd_father = tree[*kd_father]._right();
			else
				*kd_father = tree[*kd_father]._left();
			break;
		case 1:
			//cout << norm(new_node.y, dest._node()->y, 0) << " " << norm(tree[*kd_father]._node()->y, dest._node()->y, 0) << endl;
			if (norm(new_node.y, dest._node()->y, 0) > norm(tree[*kd_father]._node()->y, dest._node()->y, 0))
				*kd_father = tree[*kd_father]._right();
			else
				*kd_father = tree[*kd_father]._left();
			break;
		/*case 2:
			//cout << norm(new_node.theta, Vehicle::THETAMAX, Vehicle::THETAMIN) << " " << norm(tree[*kd_father]._node()->theta, Vehicle::THETAMAX, Vehicle::THETAMIN) << endl;
			if (norm(new_node.theta, Vehicle::THETAMAX, Vehicle::THETAMIN) > norm(tree[*kd_father]._node()->theta, Vehicle::THETAMAX, Vehicle::THETAMIN))
				*kd_father = tree[*kd_father]._right();
			else
				*kd_father = tree[*kd_father]._left();
			break;*/
		}
	}
	*kd_father = old;
}

void RRT::basic_RRT::kd_tree(RRT_Node *new_node, const Index &insert_index) 
{
	new_node->_kd_father(insert_index);

	switch (tree[insert_index]._deep())
	{
	case 0:
		//cout << norm(new_node->_node()->x, Vehicle::XMAX, Vehicle::XMIN) << " " << norm(tree[insert_index]._node()->x, Vehicle::XMAX, Vehicle::XMIN) << endl;
		if (norm(new_node->_node()->x, Vehicle::XMAX, Vehicle::XMIN) > norm(tree[insert_index]._node()->x, Vehicle::XMAX, Vehicle::XMIN))
			tree[insert_index]._right(new_node->_index());
		else
			tree[insert_index]._left(new_node->_index());
		break;
	case 1:
		//cout << norm(new_node->_node()->y, dest._node()->y, 0) << " " << norm(tree[insert_index]._node()->y, dest._node()->y, 0) << endl;
		if (norm(new_node->_node()->y, dest._node()->y, 0) > norm(tree[insert_index]._node()->y, dest._node()->y, 0))
			tree[insert_index]._right(new_node->_index());
		else
			tree[insert_index]._left(new_node->_index());
		break;
	/*case 2:
		//cout << norm(tree[insert_index]._node()->theta, Vehicle::THETAMAX, Vehicle::THETAMIN) << " " << norm(tree[insert_index]._node()->theta, Vehicle::THETAMAX, Vehicle::THETAMIN) << endl;
		if (norm(new_node->_node()->theta, Vehicle::THETAMAX, Vehicle::THETAMIN) > norm(tree[insert_index]._node()->theta, Vehicle::THETAMAX, Vehicle::THETAMIN))
			tree[insert_index]._right(new_node->_index());
		else
			tree[insert_index]._left(new_node->_index());
		break;*/
	}

	//new_node->_kd_father(insert_index);
	new_node->_deep(tree[insert_index]._deep() + 1);
}

void RRT::basic_RRT::nearest_research(Index* near_node, const Vehicle::Node &node)
{
	if (!tree.empty())
		kd_tree(near_node, node);
	else
		*near_node = HUGE_VAL;
}

void RRT::basic_RRT::rand_select(Vehicle::Node *rand_node, const int &iter)
{
	std::uniform_real_distribution<float> rand_x, rand_y;
	if (Vehicle::XMIN < /*Vehicle::XMAX*/dest._node()->x < (1 / Vehicle::KMAX) ? Vehicle::XMAX : dest._node()->x)
		rand_x = std::uniform_real_distribution<float>(Vehicle::XMIN, /*Vehicle::XMAX*/dest._node()->x < (1 / Vehicle::KMAX) ? Vehicle::XMAX : dest._node()->x);
	else
		rand_x = std::uniform_real_distribution<float>(/*Vehicle::XMAX*/dest._node()->x < (1 / Vehicle::KMAX) ? Vehicle::XMAX : dest._node()->x, Vehicle::XMIN);
		
	if (0 < dest._node()->y)
		rand_y = std::uniform_real_distribution<float>(0, dest._node()->y);
	else
		rand_y = std::uniform_real_distribution<float>(dest._node()->y, 0);

	std::default_random_engine e(iter*time(0));
	Vector4f X;
	//if (dim == 2)
		X << rand_x(e), rand_y(e), /*rand_theta(e)*/0.f, 0.f;
	/*else
	{
		std::uniform_real_distribution<float> rand_k(-0.26, 0.26);
		X << rand_x(e), rand_y(e), rand_theta(e), rand_k(e);
	}*/
	*rand_node = Vehicle::Node(X);
}

bool RRT::basic_RRT::is_new_effect(Vehicle::Node *new_node)
{
	// estimate point A
	cout << new_node->x << " " << new_node->y << " " << new_node->theta << " " << endl;
	cout << "estimate point A:" /*<< endl*/;
	float x = new_node->x - Vehicle::DL*cosf(new_node->theta) - Vehicle::W*sinf(new_node->theta) / 2;
	cout << x << " ";
	if (x <= Vehicle::XMIN || x >= Vehicle::XMAX)
		return false;
	float y = new_node->y - Vehicle::DL*sinf(new_node->theta) + Vehicle::W*cosf(new_node->theta) / 2;
	cout << y << endl;
	if (y <= Vehicle::YMIN || y >= Vehicle::YMAX)
		return false;
	
	//estimate point B
	cout << "estimate point B:" /*<< endl*/;
	x += Vehicle::W*sinf(new_node->theta);
	cout << x << " ";
	if (x <= Vehicle::XMIN || x >= Vehicle::XMAX)
		return false;
	y -= Vehicle::W*cosf(new_node->theta);
	cout << y << endl;
	if (y <= Vehicle::YMIN || y >= Vehicle::YMAX)
		return false;

	// estimate point C
	cout << "estimate point C:" /*<< endl*/;
	x += Vehicle::L*cosf(new_node->theta);
	cout << x << " ";
	if (x <= Vehicle::XMIN || x >= Vehicle::XMAX)
		return false;
	y += Vehicle::L*sinf(new_node->theta);
	cout << y << endl;
	if (y <= Vehicle::YMIN || y >= Vehicle::YMAX)
		return false;

	// estimate point D
	cout << "estimate point D:" /*<< endl*/;
	x -= Vehicle::W*sinf(new_node->theta);
	cout << x << " ";
	if (x <= Vehicle::XMIN || x >= Vehicle::XMAX)
		return false;
	y += Vehicle::W*cosf(new_node->theta);
	cout << y << endl;
	if (y <= Vehicle::YMIN || y >= Vehicle::YMAX)
		return false;

	cout << "effective" << endl;
	return true;
}

std::string RRT::basic_RRT::grow(Vehicle::Node *new_node, Collision::collision *collimap)
{
	std::string result = "failed";

	Index near_node_index;
	nearest_research(&near_node_index, *new_node);
	RRT_Node near_node = tree[near_node_index];

	float delta = atan2f(new_node->y - near_node._node()->y, new_node->x - near_node._node()->x);
	new_node->reset(near_node._node()->x + step*cos(delta), near_node._node()->y + step*sin(delta), delta);
	
	cout << "near_node:" << near_node._node()->x << "  " << near_node._node()->y << "  " << near_node._node()->theta << "  " << endl;
	cout << "new_node:" << new_node->x << "  " << new_node->y << "  " << new_node->theta << "  " << endl;
	
	if (is_new_effect(new_node))
	{
		if (new_node->x >= dest._node()->x - 1 && new_node->x <= dest._node()->x + 1 && new_node->y >= dest._node()->y - 1 && new_node->y <= dest._node()->y + 1)
		{
			new_node->reset(dest._node()->x, dest._node()->y);
			if (!collimap->iscollision(near_node._node(), *new_node))
			{
				dest._index(tree.size());
				tree[near_node_index]._successor(dest._index());
				dest._predecessor(near_node_index);
				kd_tree(&dest, near_node_index);
				tree.push_back(dest);
				result = "succeeded";
				root = tree[0];
			}
		}
		else
		{
			if (!collimap->iscollision(near_node._node(), *new_node))
			{				
				Index new_index = tree.size();
				tree.emplace_back(*new_node, new_index, near_node_index, max_size + 1);
				tree[near_node_index]._successor(new_index);
				kd_tree(&tree[new_index], near_node_index);
				result = "extended";
			}
		}
	}
	
	return result;
}

bool RRT::basic_RRT::RRT_search(Collision::collision *collimap)
{
	int iter = 1;
	Vehicle::Node rand_node;

	for (; iter < max_size; iter++)
	{
		//cout << endl;
		//cout << "iter:" << iter << endl;
		rand_select(&rand_node, iter);
		//cout << endl;
		//cout << "sample:" << rand_node.x << "  " << rand_node.y << "  " << rand_node.theta << "  " << endl;
		std::string result = grow(&rand_node,collimap);
		if (result == "failed")
			;//traj_trun
		else if (result == "succeeded")
			break;
	}
	cout << iter << endl;
	if (iter == max_size)
		return false;
	else
		return true;
}

//float RRT::RRT_Node::edge_generate(const Vehicle::Node &node_i, const Vehicle::Node &node_g)
//{
//	//Trajectory::State state_i(&predecessor->node), state_g(&node);
//	edge = &Trajectory::Path(Trajectory::State(node_i), Trajectory::State(node_g));
//	edge->path_generate();
//	edge->path_discrete();
//	return edge->_coeff()[5];
//}

void RRT::basic_RRT::getpath()
{
	float sg = 0.;
	Vector6f coeff;
	for (auto i = dest._index(); i != 0;)
	{		
		route_tree.push_back(*(tree[i]._node()));
		i = tree[i]._predecessor();
	}
	route_tree.push_back(*(root._node()));
	std::reverse(route_tree.begin(), route_tree.end());
}

//void RRT::get_path(std::vector<Trajectory::State>* states, RRT_Node *state_i, RRT_Node*state_g, const Vector5f &coeff_v, bool flag, const Vector2d &ref, const float &unit_length)
//{
//	float eps = 1e-6;
//
//	Vehicle::Node node, old_node;
//	old_node = state_i->_node();
//	Vector6f coeff_path = state_g->_edge();
//
//	int evaluations;
//	float errorEstimate;
//	//Path_function::theta_s theta_g(x);
//	//Path_function::x_s x_g(coeff_path, state_i->_node()->theta);
//	//Path_function::y_s y_g(coeff_path, state_i->_node()->theta);
//	//ref=[s_init,t]
//	//states->emplace_back(*state_i,ref[0],ref[1]);
//
//	int N = (int)std::floor(coeff_path[5] / unit_length);
//	ArrayXXd times = VectorXf::LinSpaced(N, 0., coeff_v[3]);
//	ArrayXXd lengths = times*(coeff_v[0] + times*(coeff_v[1] / 2. + times*coeff_v[2] / 3.));
//	int j = 0;
//	float s_init, t, v;
//	for (float s = unit_length; s < coeff_path[5]; s += unit_length)
//	{
//		Path_function::x_s x_g(coeff_path, old_node.theta);
//		Path_function::y_s y_g(coeff_path, old_node.theta);
//		node.k = coeff_path[0] + coeff_path[1] * s + coeff_path[2] * pow(s, 2) + coeff_path[3] * pow(s, 3) + coeff_path[4] * pow(s, 4);
//		node.theta = state_i->_node()->theta + coeff_path[0] * s + coeff_path[1] * pow(s, 2) / 2 + coeff_path[2] * pow(s, 3) / 3 + coeff_path[3] * pow(s, 4) / 4 + coeff_path[4] * pow(s, 5) / 5;
//		node.x = old_node.x + DEIntegrator<Path_function::x_s>::Integrate(x_g, 0, unit_length, eps, evaluations, errorEstimate);
//		node.y = old_node.y + DEIntegrator<Path_function::y_s>::Integrate(y_g, 0, unit_length, eps, evaluations, errorEstimate);
//		/*node.x = old_node.x + 0.5*(cos(old_node.theta) + cos(node.theta))*unit_length;
//		node.y = old_node.y + 0.5*(sin(old_node.theta) + sin(node.theta))*unit_length;*/
//		s_init = s + ref[0];
//		while (s_init >= lengths(j, 0) && j < N - 1)
//			j++;
//		t = times(j - 1, 0) + (s_init - lengths(j - 1, 0))*(times(j, 0) - times(j - 1, 0)) / (lengths(j, 0) - lengths(j - 1, 0)) + ref[1];
//		v = coeff_v[0] + coeff_v[1] * t + coeff_v[2] * pow(t, 2);
//		states->emplace_back(node, v, s_init, t);
//		//cout << "x:" << node.x << " y:" << node.y << " k:" << node.k << " theta:" << node.theta << endl;
//		old_node = node;
//	}
//
//	state_g->_node()->k = coeff_path[0] + coeff_path[1] * coeff_path[5] + coeff_path[2] * pow(coeff_path[5], 2) + coeff_path[3] * pow(coeff_path[5], 3) + coeff_path[4] * pow(coeff_path[5], 4);
//	s_init = ref[0] + coeff_path[5];
//	while (s_init >= lengths(j, 0) && j < N - 1)
//		j++;
//	t = times(j - 1, 0) + (s_init - lengths(j - 1, 0))*(times(j, 0) - times(j - 1, 0)) / (lengths(j, 0) - lengths(j - 1, 0)) + ref[1];
//	v = coeff_v[0] + coeff_v[1] * t + coeff_v[2] * pow(t, 2);
//	states->emplace_back(node, v, s_init, t);
//}