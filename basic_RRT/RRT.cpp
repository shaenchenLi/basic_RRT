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
			if (norm(new_node.x, XMAX, XMIN) > norm(tree[*kd_father]._node()->x, XMAX, XMIN))
				*kd_father = tree[*kd_father]._right();
			else
				*kd_father = tree[*kd_father]._left();
			break;
		case 1:
			if (norm(new_node.y, dest._node()->y, 0) > norm(tree[*kd_father]._node()->y, dest._node()->y, 0))
				*kd_father = tree[*kd_father]._right();
			else
				*kd_father = tree[*kd_father]._left();
			break;
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
		if (norm(new_node->_node()->x, XMAX, XMIN) > norm(tree[insert_index]._node()->x, XMAX, XMIN))
			tree[insert_index]._right(new_node->_index());
		else
			tree[insert_index]._left(new_node->_index());
		break;
	case 1:
		if (norm(new_node->_node()->y, dest._node()->y, 0) > norm(tree[insert_index]._node()->y, dest._node()->y, 0))
			tree[insert_index]._right(new_node->_index());
		else
			tree[insert_index]._left(new_node->_index());
		break;
	}

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
	if (XMIN < dest._node()->x < (1 / KMAX) ? XMAX : dest._node()->x)
		rand_x = std::uniform_real_distribution<float>(XMIN, dest._node()->x < (1 / KMAX) ? XMAX : dest._node()->x);
	else
		rand_x = std::uniform_real_distribution<float>(dest._node()->x < (1 / KMAX) ? XMAX : dest._node()->x, XMIN);
		
	if (0 < dest._node()->y)
		rand_y = std::uniform_real_distribution<float>(0, dest._node()->y);
	else
		rand_y = std::uniform_real_distribution<float>(dest._node()->y, 0);

	std::default_random_engine e(iter*time(0));
	Vector4f X;
		X << rand_x(e), rand_y(e), 0.f, 0.f;
	*rand_node = Vehicle::Node(X);
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
	
	if (Vehicle::is_node_effect(*new_node))
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
		rand_select(&rand_node, iter);
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