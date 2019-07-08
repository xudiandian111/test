#ifndef __PID
#define __PID

typedef double float64;

class PID
{
public:
	PID();
	virtual ~PID();
	void update();
    void init(float64 _aimpose, float64 _Kp, float64 _Ki, float64 _Kd);
	float64 out();

	void set_curpose(float64 _curpose);
	void set_beginpose(float64 _beginpose);
	void set_aimpose(float64 _aimpose);
	float64 read_beginpose();
	float64 read_curpose();
	float64 read_aimpose();
	float64 read_err();

	void set_vel_threshold(float64 _vel_threshold);

	void set_Kp(float64 _Kp);
	void set_Ki(float64 _Ki);
	void set_Kd(float64 _Kd);

protected:


private:
	float64 beginpose;
    float64 aimpose;
    float64 curpose;
    
	float64 err;
    float64 err_last;
    
	float64 derr;
	float64 ierr;

    float64 Kp;
	float64 Ki;
    float64 Kd;

	float64 vel_threshold;
};

#endif
