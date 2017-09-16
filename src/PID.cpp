#include "PID.h"

using namespace std;


PID::PID() :
	p_error_(0),
	i_error_(0),
	d_error_(0),
	Kp_(0),
	Ki_(0),
	Kd_(0)
	{
	}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;
}

void PID::UpdateError(double cte) {
	d_error_  = cte - p_error_;
	p_error_  = cte;
	i_error_ += cte;
}

double PID::TotalError() const {
  return Kp_ * p_error_
       + Ki_ * i_error_
       + Kd_ * d_error_;
}

double PID::GetResponse() const {
	return -TotalError();	
}


ThrottleControl::ThrottleControl () :
		p_error_(0),
		d_error_(0),
		Kp_(0),
		Kd_(0)
	{
	}


void ThrottleControl::Init(double Kp, double Kd) {
	Kp_ = Kp;
	Kd_ = Kd;
}

double ThrottleControl::GetResponse() const {
	return -Kp_ * p_error_ -Kd_ * d_error_;
}



void ThrottleControl::UpdateError(double Error) {
	d_error_ = Error - p_error_;
	p_error_ = Error;
}



