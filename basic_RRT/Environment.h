#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

//#include <utility>
#include <unordered_map>
//#include <vector>
using std::unordered_map;

namespace Environment
{
	struct position
	{
		/*size_t hasher(const float &p)
		{
			return std::hash<float>()(p + 1.2);
		}
		using try_map = std::unordered_map<float, int, decltype(hasher)*>;*/

		float x, y;
		position(const float &a, const float &b) :x(a), y(b) {}
	};

	struct EnvironMap
	{
		//xy coordinates of all objects are relative to vehicle
		/*bool equal(const position &p1, const position &p2)
		{
			return p1.x == p2.x && p1.y == p2.y;
		}

		size_t hash_value(const position &p)
		{
			return std::hash<float>()(p.x);
		}*/

		struct HashFunc
		{
			size_t operator()(const position &p) const
			{
				return std::hash<float>()(p.x);
			}
		};

		struct Equal
		{
			bool operator()(const position &p1, const position &p2) const
			{
				return p1.x == p2.x && p1.y == p2.y;
			}
		};

		using en_map = unordered_map<position, int, HashFunc, Equal>;

		EnvironMap(const position &xymin, const position &xymax, const float &inter = 0.25) :interval(inter)
		{
			//int N = ((int)std::ceil((xymax.x - xymin.x) / inter) + 1)*((int)std::ceil((xymax.y - xymin.y) / inter) + 1);
			//environment.reserve(N);
			for (float i = xymin.x; i < xymax.x + interval; i += interval)
				for (float j = xymin.y; j < xymax.y + interval; j += interval)
				{
					range = { xymin, xymax };
					position key(i, j);
					//std::pair<position, int> element;
					environment.insert({ key, 0 });
				}
		}

		void point_construct(const position &point);
		void vehicle_construct(const position &center, const position &shape);
		void line_construct(const std::vector<position> &points); //the interval of points must equal to the map's interval
		void guard_construct(const std::vector<position> &points, const float &width);
		void reset();

		//get data
		std::vector<position>* _range() { return &range; }
		float _interval() const { return interval; }
		en_map* _environment() { return &environment; }

	private:
		en_map environment;
		float interval;
		std::vector<position> range;
	};

	void map_construct(EnvironMap *environmap);
}


#endif