#include "PID.h"
float PID::get_pid(float error, float scaler) {
  uint32_t tnow = millis();
  uint32_t dt = tnow - _last_t;
  float output = 0;
  float delta_time;

  if (_last_t == 0 || dt > 1000) {
    dt = 0;
    reset_I();
  }

  _last_t = tnow;
  delta_time = (float)dt / 1000.0f;
  _pid_info.P = error * _kp;
  output += _pid_info.P;

  if ((fabsf(_kd) > 0) && (dt > 0)) {
    float derivative;
    if (isnan(_last_derivative)) {
      derivative = 0;
      _last_derivative = 0;
    }
    else {
      derivative = (error - _last_error) / delta_time;
    }

    float RC = 1 / (2 * M_PI * _fCut);

    derivative = _last_derivative +
      ((delta_time / (RC + delta_time)) *
      (derivative - _last_derivative));
    _last_error = error;
    _last_derivative = derivative;
    _pid_info.D = _kd * derivative;
    output += _pid_info.D;
  }

  output *= scaler;
  _pid_info.D *= scaler;
  _pid_info.P *= scaler;

  if ((fabsf(_ki) > 0) && (dt > 0)) {
    _integrator += (error * _ki) * scaler * delta_time;

    if (_integrator < -_imax) {
      _integrator = -_imax;
    }
    else if (_integrator > _imax) {
      _integrator = _imax;
    }

    _pid_info.I = _integrator;
    output += _integrator;
  }

  _pid_info.desired = output;

  return output;
}

void PID::reset_I() {
  _integrator = 0;
  _last_derivative = NAN;
  _pid_info.I = 0;
}

