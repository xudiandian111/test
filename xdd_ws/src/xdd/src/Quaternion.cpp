#include "xdd/Quaternion.h"
#include <math.h>

#define CLAMP(x , min , max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : x))
//x只能取min,max之间的数值

void Quaternion::SetEulerAngle(const EulerAngle &ea)
{
	double fCosHRoll = cos(ea.m_fRoll * 0.50);
	double fSinHRoll = sin(ea.m_fRoll * 0.50);
	double fCosHPitch = cos(ea.m_fPitch * 0.50);
	double fSinHPitch = sin(ea.m_fPitch * 0.50);
	double fCosHYaw = cos(ea.m_fYaw * 0.50);
	double fSinHYaw = sin(ea.m_fYaw * 0.50);

	/// Cartesian coordinate System
	//w = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
	//x = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;
	//y = fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
	//z = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;

	w = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
	x = fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
	y = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;
	z = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;
}

EulerAngle Quaternion::GetEulerAngle() const
{
	EulerAngle ea;

	/// Cartesian coordinate System 
	//ea.m_fRoll  = atan2(2 * (w * x + y * z) , 1 - 2 * (x * x + y * y));
	//ea.m_fPitch = asin(2 * (w * y - z * x));
	//ea.m_fYaw   = atan2(2 * (w * z + x * y) , 1 - 2 * (y * y + z * z));

	ea.m_fRoll  = atan2(2 * (w * z + x * y) , 1 - 2 * (z * z + x * x));
	ea.m_fPitch = asin(CLAMP(2 * (w * x - y * z) , -1.0 , 1.0));
	ea.m_fYaw   = atan2(2 * (w * y + z * x) , 1 - 2 * (x * x + y * y));

	return ea;
}