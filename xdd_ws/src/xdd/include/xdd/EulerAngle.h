#pragma once

class EulerAngle
{
public:
	double m_fYaw , m_fPitch, m_fRoll;
public:
	EulerAngle(void): m_fYaw(0.0) , m_fPitch(0.0) , m_fRoll(0.0) {}
	~EulerAngle(void) {}
};