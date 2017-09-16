#ifndef __PID_h__
#define __PID_h__

struct PID_Info {
  float desired;
  float P;
  float I;
  float D;
  float FF;
  float AFF;
};

class PID {
public:
  PID(const float initial_p = 0.0f, const float initial_i = 0.0f, const float initial_d = 0.0f, const int16_t initial_imax = 5000) {
    _kp = initial_p;
    _ki = initial_i;
    _kd = initial_d;
    _imax = initial_imax;
    _last_derivative = NAN;
  }

  float get_pid(float error, float scaler = 1.0);
  void reset_I();
  void operator () (const float p, const float i, const float d, const int16_t imaxval = 5000) {
    _kp = p;
    _ki = i;
    _kd = d;
    _imax = imaxval;
  }

  float kP() const {
    return _kp;
  }

  float kI() const {
    return _ki;
  }

  float kD() const {
    return _kd;
  }

  int16_t imax() const {
    return _imax;
  }

  void kP(const float v) {
    _kp = v;
  }

  void kI(const float v) {
    _ki = v;
  }

  void kD(const float v) {
    _kd = v;
  }

  void imax(const int16_t v) {
    _imax = abs(v);
  }

  float get_integrator() const {
    return _integrator;
  }

private:
  struct PID_Info _pid_info;
  float _kp;
  float _ki;
  float _kd;
  int16_t _imax;
  float _integrator;            // integrator value
  float _last_error;            // last error for derivative
  float _last_derivative;       // last derivative for low-pass filter
  uint32_t _last_t;             // last time get_pid() was called in millis
  float _get_pid(float error, uint16_t dt, float scaler);
  static const uint8_t _fCut = 20;
};

#endif
