#include "Environment.h"

void Environment::EnvironMap::point_construct(const position &point)
{
	int i = (int)std::floor(point.x / interval);
	int j = (int)std::floor(point.y / interval);
	position key = { i*interval, j*interval };   
	auto occupy = environment.find(key);
	occupy->second = 1;
}

//shape=[length,width]
void Environment::EnvironMap::vehicle_construct(const position &center, const position &shape)// view as a rectangle 
{
	for (float i = center.x; i < center.x + shape.x + interval; i += interval)
	{
		for (float j = center.y - shape.y / 2; j < center.y + shape.y / 2 + interval; j += interval)
		{
			position key(i, j);
			point_construct(key);
		}
	}		
}

void Environment::EnvironMap::line_construct(const std::vector<position> &points)
{
	point_construct(*points.begin());
	for (auto p = points.begin() + 1; p != points.end(); p++)
	{
		float d = sqrtf(pow(p->x - (p - 1)->x, 2) + pow(p->y - (p - 1)->y, 2));
		if (d < interval)
			point_construct(*p);
		else
		{
			float theta = atan2(p->y - (p - 1)->y, p->x - (p - 1)->x);
			int N = std::floor(d / interval);
			for (int i = 1; i <= N; i++)
				point_construct(position((p - 1)->x + interval*i*cosf(theta), (p - 1)->y + interval*i*sinf(theta)));
			point_construct(*p);
		}
	}
}

void Environment::EnvironMap::guard_construct(const std::vector<position> &points, const float &width)
{
	for (float i = -width / 2; i < width / 2 + interval; i += interval)
	{
		std::vector<position> points_width = points;
		for (auto &p : points_width)
			p.y += i;
		line_construct(points_width);
	}
}

void Environment::EnvironMap::reset()
{
	for (auto i = environment.begin(); i != environment.end(); i++)
		i->second = 0;
}

void Environment::map_construct(EnvironMap *environmap)
{
	// construct guard 
	float lane_center = 7.5;
	float guard_width = 1;
	std::vector<position> points;
	for (float i = 0.; i <= 10.; i += environmap->_interval())
		points.emplace_back(i, lane_center);
	environmap->guard_construct(points, guard_width); 
	// construct obstacle
	position center(50, 1.5), shape(5, 2);
	environmap->vehicle_construct(center, shape);
}