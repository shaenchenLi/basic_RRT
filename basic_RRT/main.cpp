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
	Environment::map_construct(&environmap);
	
	VectorXf xi(6), xg(5);
	xi << 0.f, 5.25f, 0.f, 0.f, 4.f, 0.f;
	xg << 15.f, 10.f, -Vehicle::PI, 0.f, 7.f;
	Vector3f bound;
	bound << xi[4], xi[5], xg[4];

	auto start = steady_clock::now();
	Collision::collision* collimap = new Collision::collision(&environmap);
	basic_RRT rrttree(xi, xg, 10000);
	Trajectory::traj trajectory(xi, xg);
	bool result = rrttree.RRT_search(collimap);
	if (result)
	{
		rrttree.getpath();
		trajectory._ctrl_points(*rrttree._route_tree(), rrttree._step(), collimap);
		trajectory._bspline();
		start = steady_clock::now();
		trajectory._state(&(xi[5]));
		std::vector<Trajectory::State> states = *trajectory._state_future();
		auto end = steady_clock::now();
		auto cost_time = end - start;
		cout << duration_cast<microseconds>(cost_time).count() << endl;
		collimap->~collision();
	}
}