#include "pinhole_camera.h"

PinholeCamera::PinholeCamera(double width, double height,
	double fx, double fy,
	double cx, double cy,
	double k1, double k2, double p1, double p2, double k3) :
	width_(width), height_(height),
	fx_(fx), fy_(fy), cx_(cx), cy_(cy),
	distortion_(fabs(k1) > 0.0000001)
{
	// »û±ä²ÎÊý
	d_[0] = k1; d_[1] = k2; d_[2] = p1; d_[3] = p2; d_[4] = k3;
	
}

PinholeCamera::~PinholeCamera()
{}