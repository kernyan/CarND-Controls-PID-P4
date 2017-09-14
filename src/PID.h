#ifndef PID_H
#define PID_H

using namespace std;

class ControlInterface {
public:
	virtual double GetResponse () const = 0;
	virtual void UpdateError(double Value) = 0;

	ControlInterface() {};
	virtual ~ControlInterface() {};
};

class PID : protected ControlInterface {
public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  virtual void UpdateError(double cte) override final;

  /*
  * Calculate the total PID error.
  */
  double TotalError() const;

  virtual double GetResponse() const override final;
};


class ThrottleControl : protected ControlInterface {
public:
  /*
  * Errors
  */
  double p_error_;
  double d_error_;

  /*
  * Coefficients
  */
  double Kp_;
  double Kd_;

  ThrottleControl();

  void Init(double Kp, double Kd);
  virtual double GetResponse () const override final;
  virtual void UpdateError (double Error) override final;
};

#endif /* PID_H */
