/*
* Copyright (C) Cross The Road Electronics.  All rights reserved.
* License information can be found in CTRE_LICENSE.txt
* For support and suggestions contact support@ctr-electronics.com or file
* an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
*/

#include "PWMTalonFXS.hpp"

#include "wpi/driverstation/RobotState.hpp"
#include "wpi/hal/UsageReporting.hpp"

using namespace wpi;

void PWMTalonFXS::SetThrottle(double throttle) {
  /* timer is running means we are configuring */
  if (!_timer.IsRunning()) {
    /* timer not running, we are not configurating */
    if (!RobotState::IsEnabled() || _configs.empty()) {
      /* turn off timer */
      _timer.Stop();

      /* do what the base class normally does */
      PWMMotorController::SetThrottle(throttle);
    } else {
      /* start timer */
      _timer.Restart();

      /* send PWM */
      m_pwm.SetPulseTime(_configs.front());
    }
  } else {
    /* timer running, we are applying a config */
    if (!RobotState::IsEnabled()) {
      /* turn off timer, abandon the pulse*/
      _timer.Stop();

      /* do what the base class normally does */
      PWMMotorController::SetThrottle(throttle);
    } else if (!IsTmrExpired()) {
      /* Still waiting on config pulses to finish */
    } else {
      /* tmr period expired */

      /* remove the config we've applied */
      _configs.erase(_configs.begin());

      /* next steps */
      if (_configs.empty()) {
        /* turn off timer */
        _timer.Stop();

        /* do what the base class normally does */
        PWMMotorController::SetThrottle(throttle);
      } else {
        /* start timer */
        _timer.Restart();

        /* send PWM */
        m_pwm.SetPulseTime(_configs.front());
      }
    }
  }
}

bool PWMTalonFXS::SetNeutralMode(bool bIsBrake)
{
  if (_configs.size() > 10) {
    return false;
  }
  _configs.push_back(bIsBrake ? 4000_us : 3500_us);

  if (RobotState::IsEnabled()) {
    SetThrottle(0);
  }
  return true;
}

bool PWMTalonFXS::SetMotorArrangement(MotorArrangement motorArrangement)
{
  if (_configs.size() > 10) {
    return false;
  }

  units::microsecond_t microseconds{};
  switch (motorArrangement) {
  case MotorArrangement::Minion_JST:
    microseconds = 3000_us;
    break;
  case MotorArrangement::NEO_JST:
    microseconds = 3100_us;
    break;
  case MotorArrangement::NEO550_JST:
    microseconds = 3200_us;
    break;
  case MotorArrangement::VORTEX_JST:
    microseconds = 3300_us;
    break;
  case MotorArrangement::Brushed_DC:
    microseconds = 3700_us;
    break;
  default:
    return false;
  }

  _configs.push_back(microseconds);

  if (RobotState::IsEnabled()) {
    SetThrottle(0);
  }
  return true;
}

bool PWMTalonFXS::IsTmrExpired()
{
  return _timer.Get() > 0.1_s;
}

PWMTalonFXS::PWMTalonFXS(int channel) : PWMMotorController("PWMTalonFXS", channel)
{
  SetBounds(2.004_ms, 1.52_ms, 1.5_ms, 1.48_ms, 0.997_ms);
  m_pwm.SetOutputPeriod(5_ms);
  PWMMotorController::SetThrottle(0.0);

  HAL_ReportUsage("IO", GetChannel(), "TalonFXS");
}
