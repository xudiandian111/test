#include "xdd/PID.h"
#include <cmath>

PID::PID()
{
	
}

PID::~PID()
{

}

void PID::init(float64 _aimpose, float64 _Kp, float64 _Ki, float64 _Kd)
{
	aimpose = beginpose + _aimpose;
    Kp = _Kp;
	Ki = _Ki;
    Kd = _Kd;

	ierr = 0.0;
	

	vel_threshold = 0.2;
}

void PID::update()
{
    err = aimpose - curpose;

	/****
if(std::abs(err) >= vel_threshold)
	{
		if(err > 0)
		{
			err = vel_threshold;
		}
		else if(err < 0)
		{
			err = -vel_threshold;
		}
	}
******/
    derr = err - err_last;
    err_last = err;

	ierr += err;
	if(ierr >= 5)
	{
		ierr = 5;
	}
	else if(ierr <= -5)
	{
		ierr = -5;
	}
}

float64 PID::out()
{
	return (Kp*err + Ki*ierr + Kd*derr);
}

void PID::set_aimpose(float64 _aimpose)
{
    aimpose = _aimpose;
}

void PID::set_curpose(float64 _curpose)
{
    curpose = _curpose;
}

void PID::set_beginpose(float64 _beginpose)
{
    beginpose = _beginpose;
}

float64 PID::read_beginpose()
{
	return beginpose;
}

float64 PID::read_curpose()
{
	return curpose;
}

float64 PID::read_aimpose()
{
	return aimpose;
}

float64 PID::read_err()
{
	return err;
}

void PID::set_vel_threshold(float64 _vel_threshold) 
{
	vel_threshold = _vel_threshold;
}

void PID::set_Kp(float64 _Kp)
{
	Kp = _Kp;
}

void PID::set_Ki(float64 _Ki)
{
	Ki = _Ki;
}

void PID::set_Kd(float64 _Kd)
{
	Kd = _Kd;
}

