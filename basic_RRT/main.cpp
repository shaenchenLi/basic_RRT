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
	Environment::position xymin(Vehicle::XMIN, Vehicle::YMIN), xymax(Vehicle::XMAX, Vehicle::YMAX);
	Environment::EnvironMap environmap(xymin, xymax);
	//*environmap = Environment::EnvironMap(xymin, xymax);
	Environment::map_construct(&environmap);
	
	std::ofstream outfile;
	/**outfile.open("E:\\postgraduate\\codes\\FRRT\\path1023\\path1023_2\\validate\\environment.txt");
	for (auto &e : (*environmap._environment()))
		outfile << e.first.x << " " << e.first.y << " " << e.second << endl;
	outfile.close();*/

	/*Collision::collision collimap = Collision::collision(&environmap);
	imshow("collision_map", *collimap._collision_map());
	waitKey(1000);
	outfile.open("E:\\postgraduate\\codes\\FRRT\\path1023_2\\path1023_2\\validate\\collision_map.txt");
	for (int i = 0; i < collimap._collision_map()->rows; i++)
	{
		for (int j = 0; j < collimap._collision_map()->cols; j++)
			outfile << collimap._collision_map()->at<float>(i, j) << "  ";
		outfile << endl;
	}
	outfile.close();
	collimap.~collision();*/

	//int success = 0, fail = 0;
	//float time = 0;
	///*for (int iter = 1; iter <= 1000; iter++)
	//{*/
	VectorXf xi(6), xg(5);
	xi << 0.f, 5.25f, 0.f, 0.f, 4.f, 0.f;
	xg << 30.f, 6.5f, 0.5*Vehicle::PI, 0.f, 7.f;
	Vector3f bound;
	bound << xi[4], xi[5], xg[4];

	//	//time_point<steady_clock> start, end;
	auto start = steady_clock::now();
	Collision::collision* collimap = new Collision::collision(&environmap);
	basic_RRT rrttree(xi, xg, 1000);
	Trajectory::traj trajectory(xi, xg);
	bool result = rrttree.RRT_search(collimap);
	//bool flag;
	if (result)
	{
		//++success;
		rrttree.getpath();
	/*std::ifstream infile;
	infile.open("E:\\postgraduate\\codes\\FRRT\\path1023\\path1023_2\\validate\\path.txt");
	string line, tmp;
	std::vector<Vehicle::Node> route_tree;
	while (getline(infile,line))
	{
		Vehicle::Node node;
		std::istringstream record(line);
		record >> node.x;
		record >> node.y;
		record >> node.theta;
		record >> node.k;
		route_tree.emplace_back(node);
	}*/
		trajectory._ctrl_points(*rrttree._route_tree(), rrttree._step(), collimap);
		//trajectory._ctrl_points(route_tree, 5, collimap);
		trajectory._bspline();
	/*	auto end = steady_clock::now();
		duration<float> cost_time = end - start;
		time += cost_time.count();*/
	}
		/*else
			++fail;*/
	//}

	//cout << "the count of successful search:" << success << endl;
	//cout << "the count of unsuccessful search:" << fail << endl;
	//cout << "the total search costing time:" << time << endl;
	//cout << "the average costing time:" << time / success << endl;
	//// if false, create another rrttree, using the nearest node to xg as the initial node
	//
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
		start = steady_clock::now();
		trajectory._state(&(xi[5]));
		std::vector<Trajectory::State> states = *trajectory._state_future();
		auto end = steady_clock::now();
		auto cost_time = end - start;
		cout << duration_cast<microseconds>(cost_time).count() << endl;
		
		std::vector<RRT_Node> tree(*rrttree._tree());
		std::vector<Vehicle::Node>* route_tree(rrttree._route_tree());
		std::ofstream outfile;
		outfile.open("E:\\postgraduate\\codes\\FRRT\\path1023\\path1023_2\\validate\\treenode.txt");
		//cout << outfile.is_open() << endl;
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
		//Vector2d ref = { 0, 0 };//ref=[s_init,t];
		for (auto it : *trajectory._ctrl_points())
		{
			outfile << it.x << " " << it.y << " " << endl;
		}
		//need to add initial and final node*/
		outfile.close();
	}
	system("pause");
}

