#define DEBUG_MODULE "MLESO_CONTROLLER"
#include "stabilizer_types.h"

#include "controller_mleso.h"
#include "math_nd.h"

#include "pid.h"
#include "debug.h"
#include "platform_defaults.h"

#include "log.h"
#include "param.h"


#define UPDATE_DT (float)(1.0f / (float)ATTITUDE_RATE)
#define NBR_STATES 9
#define NBR_ACT 4
#define ANGLE2ACT_CMD (float)UINT16_MAX/ 360.0f
#define PWM_THROTTLE2FREQ 20.0f/(float)UINT16_MAX

#define FLAPPER_MAX_THRUST 60000

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

static struct vecX stateP_hat = {.length = NBR_STATES};


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

void controllerMlesoInit(void)
{
  if (isInit)
    return;

  static bool rateFiltEnable = ATTITUDE_RATE_LPF_ENABLE;

  pidInit(&pidFeedbackRoll, 0, pidFeedbackRoll.kp, pidFeedbackRoll.ki, pidFeedbackRoll.kd,
          pidFeedbackRoll.kff, UPDATE_DT, ATTITUDE_RATE, ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ, rateFiltEnable);
  pidInit(&pidFeedbackPitch, 0, pidFeedbackPitch.kp, pidFeedbackPitch.ki, pidFeedbackPitch.kd,
          pidFeedbackPitch.kff, UPDATE_DT, ATTITUDE_RATE, ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ, rateFiltEnable);
  pidInit(&pidFeedbackYaw, 0, pidFeedbackYaw.kp, pidFeedbackYaw.ki, pidFeedbackYaw.kd,
          pidFeedbackYaw.kff, UPDATE_DT, ATTITUDE_RATE, ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ, rateFiltEnable);
  pidSetDesired(&pidFeedbackRoll, 0.0f);
  pidSetDesired(&pidFeedbackPitch, 0.0f);
  pidSetDesired(&pidFeedbackYaw, 0.0f);

  isInit = true;
}

bool controllerMlesoTest(void)
{

  return isInit;
}


struct vecX getStateFeedback(const struct vecX stateP) {
  struct vecX v = vzeroX(NBR_ACT);
  pidSetDesired(&pidFeedbackRoll, stateP.vec[0]);
  v.vec[1] = pidUpdate(&pidFeedbackRoll, 0.0f, true);
  pidSetDesired(&pidFeedbackPitch, stateP.vec[1]);
  v.vec[2] = pidUpdate(&pidFeedbackPitch, 0.0f, true);
  pidSetDesired(&pidFeedbackYaw, stateP.vec[2]);
  v.vec[3] = pidUpdate(&pidFeedbackYaw, 0.0f, true);
  
  return vXscl(ANGLE2ACT_CMD, v);
}

struct vecX convert2ActCmd(struct vecX act_cmd) {
  struct vecX v = {.length = act_cmd.length};
  v.vec[0] = fmin(act_cmd.vec[0], FLAPPER_MAX_THRUST);
  v.vec[0] = (v.vec[0] - 0.5f*act_cmd.vec[1])*PWM_THROTTLE2FREQ;
  v.vec[1] = (act_cmd.vec[0] + 0.5f*act_cmd.vec[1])*PWM_THROTTLE2FREQ;
  v.vec[2] = act_cmd.vec[2];
  v.vec[3] = act_cmd.vec[3];
  //act_cmd[2] /= -(float)UINT16_MAX; TODO: check input mleso
  //act_cmd[3] /= -(float)UINT16_MAX;
  return v;
}

struct vecX convert2PWMCmd(struct vecX act_cmd){
  struct vecX v = {.length = act_cmd.length};
  v.vec[0] = 0.5f*(act_cmd.vec[0] + act_cmd.vec[1])/PWM_THROTTLE2FREQ;
  v.vec[1] = (act_cmd.vec[1] - act_cmd.vec[0])/PWM_THROTTLE2FREQ;
  v.vec[2] = act_cmd.vec[2] * ANGLE2ACT_CMD;
  v.vec[3] = act_cmd.vec[3] * ANGLE2ACT_CMD;
  return v;
}

void mlesoControllerResetRollFeedbackPID(void)
{
    pidReset(&pidFeedbackRoll, 0.0f);
}

void mlesoControllerResetPitchFeedbackPID(void)
{
    pidReset(&pidFeedbackPitch, 0.0f);
}

void mlesoControllerResetAllPID(void)
{
  pidReset(&pidFeedbackPitch, 0.0f);
  pidReset(&pidFeedbackYaw, 0.0f);
  pidReset(&pidFeedbackRoll, 0.0f);
}

struct vecX mlesoModel(struct vecX actCmd, struct vecX stateP) {
  const struct matXX Aref = {.mat = A_REF, .rows = NBR_STATES, .colums = NBR_STATES};
  const struct matXX B = {.mat = B_P, .rows = NBR_STATES, .colums = NBR_ACT};
  const struct matXX L0 = {.mat = L0_EST, .rows = NBR_STATES, .colums = NBR_STATES};
  const struct matXX M0 = {.mat = M0_EST, .rows = NBR_STATES, .colums = NBR_STATES};

  struct vecX deltaState = vXsub(stateP_hat, stateP);
  struct vecX stateP_hat_dot =  vXadd3(mvXXmul(Aref, stateP), mvXXmul(B, actCmd), mvXXmul(L0, deltaState));
  struct vecX disturbance = mvXXmul(M0, deltaState);

  vcpyX(stateP_hat, vXadd(stateP_hat, vXscl(UPDATE_DT, stateP_hat_dot)));

  return vneltX(NBR_ACT, disturbance);
}

void controllerMleso(control_t *control, const setpoint_t *setpoint,
                     const sensorData_t *sensors,
                     const state_t *state,
                     const stabilizerStep_t stabilizerStep)
{
  const struct matXX Kr = {.mat = K_R, .rows = NBR_ACT, .colums = NBR_ACT};;
  control->controlMode = controlModeLegacy;
  
  // convert setpoint to vector
  //TODO: test is still yaw rate cmd
  float stpnt[NBR_STATES] = {setpoint->thrust, setpoint->attitude.roll, setpoint->attitude.pitch,
          setpoint->attitudeRate.yaw, 0, 0, 0, 0, 0};
  struct vecX setpointV = mkvecX(NBR_ACT, stpnt);

  //drone state in order for MLESO controller (att_rate xyz, vel zxy, att xyz)
  float stateArray[NBR_STATES] = {sensors->gyro.x, sensors->gyro.y, sensors->gyro.z,
      state->velocity.z, state->velocity.x, state->velocity.y,
      state->attitude.roll, state->attitude.pitch, state->attitude.yaw};
  struct vecX stateP = mkvecX(NBR_STATES, stateArray);
  
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {

    struct vecX stateFeedback = getStateFeedback(stateP);

    struct vecX actCmd = vXsub(mvXXmul(Kr, setpointV), stateFeedback);
    vcpyX(actCmd, convert2ActCmd(actCmd));

    struct vecX disMatched = mlesoModel(actCmd, stateP);
    vcpyX(actCmd, vXsub(actCmd, disMatched));

    vcpyX(actCmd, convert2PWMCmd(actCmd));

    control->thrust = actCmd.vec[0];
    control->roll = saturateSignedInt16(actCmd.vec[1]);
    control->pitch = saturateSignedInt16(actCmd.vec[2]);
    control->yaw = saturateSignedInt16(actCmd.vec[3]);

    control->yaw = -control->yaw;

    //logging
    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = (sensors->gyro.x);
    r_pitch = -(sensors->gyro.y);
    r_yaw = (sensors->gyro.z);
    accelz = sensors->acc.z;

    if (control->thrust == 0)
    {
      control->roll = 0;
      control->pitch = 0;
      control->yaw = 0;

      vcpyX(stateP_hat, vzeroX(stateP_hat.length));

      mlesoControllerResetAllPID();

    }
 
  }

}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
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
 * @brief Desired roll setpoint
 */

LOG_GROUP_STOP(controller)