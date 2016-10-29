#ifndef RRT_H
#define RRT_H

#include <algorithm>
#include <ctime>
#include <iostream>
#include <list>
#include <math.h>
#include <random>
#include <string>
#include <vector>

#include <Dense>
using namespace Eigen;
using Vector5f = Matrix<float, 5, 1>;
using Vector6f = Matrix<float, 6, 1>;

#include "Collision_check.h"
#include "Vehicle.h"

namespace RRT
{
	struct RRT_Node;

	using Index = std::vector<RRT_Node>::size_type;

	const int dim = 2;

	struct RRT_Node
	{
		RRT_Node() = default;
		RRT_Node(const RRT_Node &n) :
			node(n.node), predecessor(n.predecessor), successor(n.successor),
			left(n.left), right(n.right), kd_father(n.kd_father), deep(n.deep), index(n.index) {}
		RRT_Node(const Vehicle::Node &n, const Index &i, const Index &p, const Index &max) :node(n), index(i), predecessor(p)
		{
			left = right = kd_father = max; //max equals to the max_size of tree +1
			deep = 0;
			successor = std::vector<Index>();
		}
		RRT_Node(const Vehicle::Node &n) :node(n) {}

		void reset_node(float x, float y)
		{
			node.x = x;
			node.y = y;
		}
		void reset_node(float x, float y, float theta) { reset_node(x, y); node.theta = theta; }
		float edge_generate(const Vehicle::Node &node_i, const Vehicle::Node &node_g);

		//getdata
		Vehicle::Node* _node() { return &node; }
		Index _predecessor() const { return predecessor; }
		Index _left() const { return left; }
		Index _right() const { return right; }
		Index _kd_father() const { return right; }
		int _deep() const { return deep; }
		Index _index() { return index; }

		//setdata
		void _index(const Index i) { index = i; }
		void _left(const Index &l) { left = l; }
		void _right(const Index &r) { right = r; }
		void _kd_father(const Index &f) { kd_father = f; }
		void _predecessor(const Index &p) { predecessor = p; }
		void _successor(const Index &s) { successor.push_back(s); }
		void _deep(const int &d) { deep = d % dim; }

	private:
		Vehicle::Node node;
		Index index;
		Index predecessor;
		std::vector<Index> successor;
		Index left, right, kd_father;
		int deep;
	};

	struct basic_RRT
	{
		basic_RRT() = default;
		basic_RRT(const VectorXf xi, const VectorXf xg, const int m = 1000, const float s = 5) :step(s), max_size(m)
		{
			root = RRT_Node(Vehicle::Node(xi), 0, 0, m+1);
			dest = RRT_Node(Vehicle::Node(xg), m+1, m+1, m+1);
			tree.reserve(m + 1);
			tree.emplace_back(root);
		}

		virtual std::string grow(Vehicle::Node *new_node, Collision::collision *collimap);
		virtual void rand_select(Vehicle::Node *rand_node, const int &iter);
		virtual bool is_new_effect(Vehicle::Node *new_node);
		void kd_tree(Index* kd_father, const Vehicle::Node &node); //recursive 
		void kd_tree(RRT_Node *new_node, const Index &insert_index); //insert
		virtual void nearest_research(Index* near_node, const Vehicle::Node &node);
		virtual bool RRT_search(Collision::collision *collimap);
		void getpath();

		std::vector<RRT_Node>* _tree() { return &tree; }
		std::vector<Vehicle::Node>* _route_tree() { return &route_tree; }
		float _step() const { return step; }

	private:
		RRT_Node root, dest;
		std::vector<RRT_Node> tree;
		std::vector<Vehicle::Node> route_tree;
		float step;
		int max_size;
	};

	inline float norm(const float &raw, const float &max, const float &min) { return std::fabs((max - raw) / (max - min)); }
}

#endif