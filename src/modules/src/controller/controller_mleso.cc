#define DEBUG_MODULE "MLESO_DEBUG"

#include "stabilizer_types.h"

#include "pid.h"
#include "debug.h"
#include "controller_mleso.h"
#include "platform_defaults.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

#include <Eigen/Core>

#define UPDATE_DT (float)(1.0f / ATTITUDE_RATE)
#define NBR_STATES 9
#define ANGLE2ACT_CMD (float)UINT16_MAX/ 360.0f
#define THROTTLE2FREQ 20.0f

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

const Eigen::Matrix4f Kr {
    {MLESO_GAIN_THROTTLE, 0, 0, 0},
    {0, MLESO_GAIN_ROLL*ANGLE2ACT_CMD, 0, 0},
    {0, 0, MLESO_GAIN_PITCH*ANGLE2ACT_CMD, 0},
    {0, 0, 0, MLESO_GAIN_YAW*ANGLE2ACT_CMD}};

static const Eigen::MatrixXf Aref = Eigen::MatrixXf(A_REF);
static const Eigen::MatrixXf B = Eigen::MatrixXf(B_P);
static const Eigen::MatrixXf L0 = Eigen::MatrixXf(L0_EST);
static const Eigen::MatrixXf M0 = Eigen::MatrixXf(M0_EST);

static Eigen::VectorXf stateP_hat(9);

void controllerMLESOInit(void)
{
  if (isInit)
    return;
  DEBUG_PRINT("Initializing");

  pidInit(&pidFeedbackRoll, 0, pidFeedbackRoll.kp, pidFeedbackRoll.ki, pidFeedbackRoll.kd,
          pidFeedbackRoll.kff, UPDATE_DT, ATTITUDE_RATE, omxFiltCutoff, rateFiltEnable);
  pidInit(&pidFeedbackPitch, 0, pidFeedbackPitch.kp, pidFeedbackPitch.ki, pidFeedbackPitch.kd,
          pidFeedbackPitch.kff, UPDATE_DT, ATTITUDE_RATE, omyFiltCutoff, rateFiltEnable);
  pidInit(&pidFeedbackYaw, 0, pidFeedbackYaw.kp, pidFeedbackYaw.ki, pidFeedbackYaw.kd,
          pidFeedbackYaw.kff, UPDATE_DT, ATTITUDE_RATE, omzFiltCutoff, rateFiltEnable);
  pidSetDesired(&pidFeedbackRoll, 0.0f);
  pidSetDesired(&pidFeedbackPitch, 0.0f);
  pidSetDesired(&pidFeedbackYaw, 0.0f);

  stateP_hat << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  DEBUG_PRINT("controller initialized");

  isInit = true;
}

bool controllerMLESOTest(void)
{

  return isInit;
}

void getStateFeedback(Eigen::Vector4f& stateFeedback, const Eigen::VectorXf& stateP) {
  stateFeedback[0] = 0.0f;
  pidSetDesired(&pidFeedbackRoll, stateP[0]);
  stateFeedback[1] = pidUpdate(&pidFeedbackRoll, 0.0f, true);
  pidSetDesired(&pidFeedbackPitch, stateP[1]);
  stateFeedback[2] = pidUpdate(&pidFeedbackPitch, 0.0f, true);
  pidSetDesired(&pidFeedbackYaw, stateP[2]);
  stateFeedback[3] = pidUpdate(&pidFeedbackYaw, 0.0f, true);
  
  stateFeedback *= ANGLE2ACT_CMD;
}

void convertFromMotorCmd2ActCmd(Eigen::Vector4f& act_cmd) {
  act_cmd[0] = (act_cmd[0] - 0.5f*act_cmd[1])/(float)UINT16_MAX*THROTTLE2FREQ;
  act_cmd[1] = (act_cmd[0] + 0.5f*act_cmd[1])/(float)UINT16_MAX*THROTTLE2FREQ;
  act_cmd[2] /= -(float)UINT16_MAX;
  act_cmd[3] /= -(float)UINT16_MAX;
}

void convertFromActCmd2MotorCmd(Eigen::Vector4f& act_cmd){
  act_cmd[0] = 0.5f*(act_cmd[0] + act_cmd[1])*UINT16_MAX/THROTTLE2FREQ;
  act_cmd[1] = (act_cmd[1] - act_cmd[0])*UINT16_MAX/THROTTLE2FREQ;
  act_cmd[2] *= -UINT16_MAX;
  act_cmd[3] *= -UINT16_MAX;
}

void mlesoControllerResetRollFeedbackPID(void)
{
    pidReset(&pidFeedbackRoll);
}

void mlesoControllerResetPitchFeedbackPID(void)
{
    pidReset(&pidFeedbackPitch);
}

void mlesoControllerResetAllPID(void)
{
  pidReset(&pidFeedbackPitch);
  pidReset(&pidFeedbackYaw);
  pidReset(&pidFeedbackRoll);
}


Eigen::Vector4f mlesoModel(Eigen::Vector4f& actCmd, Eigen::VectorXf stateP) {
  Eigen::VectorXf stateP_hat_dot = Aref*stateP + B*actCmd + L0*(stateP_hat-stateP);
  Eigen::VectorXf disturbance = M0 * (stateP_hat-stateP);

  stateP_hat += stateP_hat_dot * UPDATE_DT;
  return Eigen::Vector4f(disturbance[0], disturbance[1], disturbance[2], disturbance[3]);
}


void controllerMLESO(control_t *control, const setpoint_t *setpoint,
                     const sensorData_t *sensors,
                     const state_t *state,
                     const stabilizerStep_t stabilizerStep)
{
  control->controlMode = controlModeLegacy;

  Eigen::Vector4f setpoint_eigen = {setpoint->thrust, setpoint->attitude.roll,
      setpoint->attitude.pitch, setpoint->attitudeRate.yaw};
  
  Eigen::VectorXf stateP(NBR_STATES);
  stateP << sensors->gyro.x, sensors->gyro.y, sensors->gyro.z,
      state->velocity.z, state->velocity.x, state->velocity.y,
      state->attitude.roll, state->attitude.pitch, state->attitude.yaw;

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    Eigen::Vector4f stateFeedback;
    getStateFeedback(stateFeedback, stateP);

    Eigen::Vector4f actCmd = Kr*setpoint_eigen - stateFeedback;
    convertFromMotorCmd2ActCmd(actCmd);

    Eigen::Vector4f disMatched = mlesoModel(actCmd, stateP);
    actCmd -= disMatched;

    convertFromActCmd2MotorCmd(actCmd);

    control->thrust = actCmd[0];
    control->roll = saturateSignedInt16(actCmd[1]);
    control->pitch = saturateSignedInt16(actCmd[2]);
    control->yaw = saturateSignedInt16(actCmd[3]);

    control->yaw = -control->yaw;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
    control->thrust = 0;
    if (control->thrust == 0)
    {
      control->thrust = 0;
      control->roll = 0;
      control->pitch = 0;
      control->yaw = 0;

      cmd_thrust = control->thrust;
      cmd_roll = control->roll;
      cmd_pitch = control->pitch;
      cmd_yaw = control->yaw;

      mlesoControllerResetAllPID();

    }
 
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
LOG_GROUP_STOP(controller)
