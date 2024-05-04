
#include "stabilizer_types.h"

#include "pid.h"
#include "controller_mleso.h"
#include "platform_defaults.h"

#include "log.h"
#include "param.h"
#include "math3d.h"


#define UPDATE_DT (float)(1.0f / ATTITUDE_RATE)
static bool rateFiltEnable = ATTITUDE_RATE_LPF_ENABLE;
static float omxFiltCutoff = ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ;
static float omyFiltCutoff = ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ;
static float omzFiltCutoff = ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ;

static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

static bool isInit;

PidObject pidFeedbackRoll = {
    .kp = PID_FEEDBACK_ROLL_KP,
    .ki = PID_FEEDBACK_ROLL_KI,
    .kd = PID_FEEDBACK_ROLL_KD,
    .kff = PID_FEEDBACK_ROLL_KFF,
};
PidObject pidFeedbackPitch = {
    .kp = PID_FEEDBACK_PITCH_KP,
    .ki = PID_FEEDBACK_PITCH_KI,
    .kd = PID_FEEDBACK_PITCH_KD,
    .kff = PID_FEEDBACK_PITCH_KFF,
};
PidObject pidFeedbackYaw = {
    .kp = PID_FEEDBACK_YAW_KP,
    .ki = PID_FEEDBACK_YAW_KI,
    .kd = PID_FEEDBACK_YAW_KD,
    .kff = PID_FEEDBACK_YAW_KFF,
};

const Eigen::

void controllerMLESOInit(void)
{
  if (isInit)
    return;

  pidInit(&pidFeedbackRoll, 0, pidFeedbackRoll.kp, pidFeedbackRoll.ki, pidFeedbackRoll.kd,
          pidFeedbackRoll.kff, UPDATE_DT, ATTITUDE_RATE, omxFiltCutoff, rateFiltEnable);
  pidInit(&pidFeedbackPitch, 0, pidFeedbackPitch.kp, pidFeedbackPitch.ki, pidFeedbackPitch.kd,
          pidFeedbackPitch.kff, UPDATE_DT, ATTITUDE_RATE, omyFiltCutoff, rateFiltEnable);
  pidInit(&pidFeedbackYaw, 0, pidFeedbackYaw.kp, pidFeedbackYaw.ki, pidFeedbackYaw.kd,
          pidFeedbackYaw.kff, UPDATE_DT, ATTITUDE_RATE, omzFiltCutoff, rateFiltEnable);

  isInit = true;
}

bool controllerMLESOTest(void)
{

  return isInit;
}

static float capAngle(float angle)
{
  float result = angle;

  while (result > 180.0f)
  {
    result -= 360.0f;
  }

  while (result < -180.0f)
  {
    result += 360.0f;
  }

  return result;
}

void controllerMLESO(control_t *control, const setpoint_t *setpoint,
                     const sensorData_t *sensors,
                     const state_t *state,
                     const stabilizerStep_t stabilizerStep)
{
  control->controlMode = controlModeLegacy;

  Eigen::Vector4f setpoint = {setpoint->thrust, setpoint->attitude.roll,
      setpoint->attitude.pitch, setpoint->attitude_rate.yaw};

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    
    

    
  }


}

/**
 * Logging variables for the command and reference signals for the
 * altitude MLESO controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll, &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch, &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw, &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate, &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate, &rateDesired.yaw)
LOG_GROUP_STOP(controller)
