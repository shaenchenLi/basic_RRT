// Version 2.3 Li Yishan
//

#include <chrono>
#include <fstream>
#include <sstream>
using namespace std::chrono;

#include "Collision_check.h"
#include "Environment.h"
#include "RRT.h"
#include "Trajectory.h"
using namespace RRT;
using std::cout;
using std::endl;

int main()
{
	Environment::position xymin(XMIN, YMIN), xymax(XMAX, YMAX);
	Environment::EnvironMap environmap(xymin, xymax);
	Environment::map_construct(&environmap);
	
	std::ofstream outfile;
	outfile.open("E:\\postgraduate\\codes\\FRRT\\path1023\\path1023_2\\validate\\environment.txt");
	for (auto &e : (*environmap._environment()))
		outfile << e.first.x << " " << e.first.y << " " << e.second << endl;
	outfile.close();

	VectorXf xi(6), xg(5);
	xi << 0.f, 5.25f, 0.f, 0.f, 4.f, 0.f;
	xg << 50.f, 13.f, -PI, 0.f, 7.f;
	Vector3f bound;
	bound << xi[4], xi[5], xg[4];
	Trajectory::State XI(xi);
	Trajectory::State XG(xg);
	std::vector<Trajectory::State> *adjust_states_front = new std::vector<Trajectory::State>;
	std::vector<Trajectory::State> *adjust_states_end = new std::vector<Trajectory::State>;
	if (xi[4] != 0)
	{
		Trajectory::adjust_k(xi, &XI, adjust_states_front, -1);
		xi[5] = 0;
	}
	if (xg[4] != 0)
		Trajectory::adjust_k(xi, &XG, adjust_states_end, 1);// make the initial and final curvature 0

	Collision::collision* collimap = new Collision::collision(&environmap);
	basic_RRT rrttree(*XI._node(), *XG._node(), 1000);
	bool result = rrttree.RRT_search(collimap);
	Trajectory::traj trajectory(XI, XG, xi[5], adjust_states_front);
	if (result)
	{
		rrttree.getpath();	
		trajectory._ctrl_points(*rrttree._route_tree(), rrttree._step(), collimap);
		trajectory._bspline();
	}

	outfile.open("E:\\postgraduate\\codes\\FRRT\\path1023\\path1023_2\\validate\\collision_map.txt");
	for (int i = 0; i < collimap->_collision_map()->rows; i++)
	{
		for (int j = 0; j < collimap->_collision_map()->cols; j++)
			outfile << collimap->_collision_map()->at<float>(i, j) << "  ";
		outfile << endl;
	}
	outfile.close();
	collimap->~collision();

	if (result)
	{
		trajectory._state(&(xi[5]), adjust_states_end);
		std::vector<Trajectory::State> states = *trajectory._state_future();
		auto end = steady_clock::now();

		std::vector<RRT_Node> tree(*rrttree._tree());
		std::vector<Vehicle::Node>* route_tree(rrttree._route_tree());
		std::ofstream outfile;
		outfile.open("E:\\postgraduate\\codes\\FRRT\\path1023\\path1023_2\\validate\\treenode.txt");
		cout << outfile.is_open() << endl;
		outfile << "ALL NODES IN THE RRT " << endl;
		outfile << "x      y      theta      k" << endl;
		for (auto &it : tree)
			outfile << it._node()->x << " " << it._node()->y << " " << it._node()->theta << " " << it._node()->k << endl;
		outfile.close();

		outfile.open("E:\\postgraduate\\codes\\FRRT\\path1023\\path1023_2\\validate\\path.txt");
		outfile << "The Selected Nodes in the RRT, which are nodes of final path" << endl;
		outfile << "x      y      theta      k" << endl;
		for (auto &it : *route_tree)
			outfile << it.x << " " << it.y << " " << it.theta << " " << it.k << endl;
		outfile.close();

		outfile.open("E:\\postgraduate\\codes\\FRRT\\path1023\\path1023_2\\validate\\pathnode.txt");
		Vector2d ref = { 0, 0 };//ref=[s_init,t];
		for (auto it : states)
		{
			outfile << it._node()->x << " " << it._node()->y << " " << it._node()->theta << " " << it._node()->k << " " << it._v() << " " << endl;
		}
		outfile.close();

		outfile.open("E:\\postgraduate\\codes\\FRRT\\path1023\\path1023_2\\validate\\ctrlpoints.txt");
		for (auto it : *trajectory._ctrl_points())
		{
			outfile << it.x << " " << it.y << " " << endl;
		}
		//need to add initial and final node*/
		outfile.close();
	}
	system("pause");
}
