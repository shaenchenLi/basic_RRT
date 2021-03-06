#include "Collision_check.h"

Mat* Collision::maptoMat(Environment::EnvironMap *environmap)
{
	int row = (int)std::ceil((environmap->_range()->crbegin()->y - environmap->_range()->cbegin()->y) / environmap->_interval());
	int col = (int)std::ceil((environmap->_range()->crbegin()->x - environmap->_range()->cbegin()->x) / environmap->_interval());
	
	Mat* environmat = new Mat(row, col, CV_32F, Scalar(0));

	for (int i = 0; i < environmat->rows; i++)
	{
		for (int j = 0; j < environmat->cols; j++)
		{
			Environment::position pos(environmap->_range()->begin()->x + j*environmap->_interval(), environmap->_range()->begin()->y + i*environmap->_interval());
			environmat->at<float>(i, j) = (float)((environmap->_environment()->find(pos))->second);
		}
	}
	return environmat;
}

Mat* Collision::collision::_kernel(const float &interval)
{
	float radius = sqrt(pow(L, 2) + 9 * pow(W, 2)) / 6;
	int row = (int)std::ceil(radius / interval);
	Mat* kernel = new Mat(2 * row, 2 * row, CV_32F, Scalar(0));
	float obs = (float)(1 / (4 * pow(row, 2)));

	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < row; j++)
		{
			float y = row*interval - sqrt(pow(radius, 2) - pow((j - row + 1)*interval, 2));
			if ((i + 1)*interval>y)
				kernel->at<float>(i, j) = obs;
		}
	}
	for (int i = 0; i < row; i++)
	{
		for (int j = row; j < 2 * row; j++)
		{
			float y = row*interval - sqrt(pow(radius, 2) - pow((j - row)*interval, 2));
			if ((i + 1)*interval>y)
				kernel->at<float>(i, j) = obs;
		}
	}
	for (int i = row; i < 2 * row; i++)
	{
		for (int j = 0; j < row; j++)
		{
			float y = row*interval + sqrt(pow(radius, 2) - pow((j - row + 1)*interval, 2));
			if (i*interval < y)
				kernel->at<float>(i, j) = obs;
		}
	}
	for (int i = row; i < 2 * row; i++)
	{
		for (int j = row; j < 2 * row; j++)
		{
			float y = row*interval + sqrt(pow(radius, 2) - pow((j - row)*interval, 2));
			if (i*interval < y)
				kernel->at<float>(i, j) = obs;
		}
	}
	return kernel;
}

void Collision::collision::_collision_map(Environment::EnvironMap *environmap)
{
	Mat* environmat = maptoMat(environmap);
	if (collision_map != nullptr)
		delete collision_map;
	collision_map = new Mat(environmat->rows, environmat->cols, CV_32F, Scalar(0));
	filter2D(*environmat, *collision_map, -1, *kernel);
	delete environmat;
}

bool Collision::collision::iscollision(const Vehicle::Node &begin_node, const Vehicle::Node &end_node)
{
	//bool judge[3] = { false, false, false }; // according to the length of the vehicle 	
	cout << "begin_node:" << begin_node.x << " " << begin_node.y << endl;
	cout << "end_node:" << end_node.x << " " << end_node.y << endl;
	if (iscollision(end_node.x, end_node.y, end_node.theta))
		return true;
	float s_total = sqrtf(powf(begin_node.x - end_node.x, 2) + powf(begin_node.y - end_node.y, 2));
	for (float s = space; s < s_total; s += space)
	{
		cout << begin_node.x + s*cosf(end_node.theta) << " " << begin_node.y + s*sinf(end_node.theta) << endl;
		if (iscollision(begin_node.x + s*cosf(end_node.theta), begin_node.y + s*sinf(end_node.theta), end_node.theta))
			return true;
	}
	return false;
}

bool Collision::collision::iscollision(const float &x, const float &y, const float &theta)
{
	std::vector<float> circle(6);
	std::vector<int> circle_map(6); //circle can be delete in the future
	float judge[3];
	for (int i = 0; i < 3; i++)
	{
		circle[2 * i] = y + (L / 6 + L*i / 3 - DL)*sin(theta);
		circle[2 * i + 1] = x + (L / 6 + L*i / 3 - DL)*cos(theta);
		circle_map[2 * i] = (int)std::floor((circle[2 * i] - origin[1]) / interval);
		circle_map[2 * i + 1] = (int)std::floor((circle[2 * i + 1] - origin[0]) / interval);
		cout << circle_map[2 * i] << " " << circle_map[2 * i + 1] << endl;
		judge[i] = collision_map->at<float>(circle_map[2 * i], circle_map[2 * i + 1]);
		if (judge[i] != 0)
			return true;			
	}
	return false;
}
