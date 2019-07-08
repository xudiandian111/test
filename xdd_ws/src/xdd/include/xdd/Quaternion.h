#pragma once

#include "EulerAngle.h"

class Quaternion
{
public:
	double x , y , z , w;
public:
	Quaternion(void) : x(0.0) , y(0.0) , z(0.0) , w(1.0) {}
	~Quaternion(void) {}
	void SetEulerAngle(const EulerAngle& ea);
	EulerAngle GetEulerAngle() const;
};