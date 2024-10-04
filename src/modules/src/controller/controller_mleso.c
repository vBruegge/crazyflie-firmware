#define DEBUG_MODULE "MLESO_debug"
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
  DEBUG_PRINT("Initializing controller");

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

  DEBUG_PRINT("Controller initialized");

  isInit = true;
}

bool controllerMlesoTest(void)
{

  return isInit;
}


struct vecX getStateFeedback(const struct vecX stateP) {
  struct vecX v = vzeroX(NBR_ACT);
  pidSetDesired(&pidFeedbackRoll, stateP[0]);
  v.vec[1] = pidUpdate(&pidFeedbackRoll, 0.0f, true);
  pidSetDesired(&pidFeedbackPitch, stateP[1]);
  v.vec[2] = pidUpdate(&pidFeedbackPitch, 0.0f, true);
  pidSetDesired(&pidFeedbackYaw, stateP[2]);
  v.vec[3] = pidUpdate(&pidFeedbackYaw, 0.0f, true);
  
  return vscl(ANGLE2ACT_CMD, v);
}

struct vecX convert2ActCmd(struct vecX act_cmd) {
  struct vecX v = {.length act_cmd.length}
  v.vec[0] = fmin(act_cmd.vec[0], FLAPPER_MAX_THRUST);
  v.vec[0] = (act_cmd.vec[0] - 0.5f*act_cmd.vec[1])*PWM_THROTTLE2FREQ;
  v.vec[1] = (act_cmd.vec[0] + 0.5f*act_cmd.vec[1])*PWM_THROTTLE2FREQ;
  v.vec[2] = act_cmd.vec[2];
  v.vec[3] = act_cmd.vec[3];
  //act_cmd[2] /= -(float)UINT16_MAX; TODO: check input mleso
  //act_cmd[3] /= -(float)UINT16_MAX;
  return v;
}

struct vecX convert2PWMCmd(struct vecX act_cmd){
  struct vecX v = {.length act_cmd.length}
  v.vec[0] = 0.5f*(act_cmd.vec[0] + act_cmd.vec[1])/PWM_THROTTLE2FREQ;
  v.vec[1] = (act_cmd.vec[1] - act_cmd.vec[0])/PWM_THROTTLE2FREQ;
  v.vec[2] = act_cmd.vec[2] * ANGLE2ACT_CMD;
  v.vec[3] = act_cmd.vec[3] * ANGLE2ACT_CMD;
  return v;
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

struct vecX mlesoModel(struct vecX actCmd, struct vecX stateP) {
  const struct matXX Aref = {.mat A_REF, .rows NBR_STATES, .colums NBR_STATES};
  const struct matXX B = {.mat B_P, .rows NBR_STATES, .colums NBR_ACT};
  const struct matXX L0 = {.mat L0_EST, .rows NBR_STATES, .colums NBR_STATES};
  const struct matXX M0 = {.mat M0_EST, .rows NBR_STATES, .colums NBR_STATES};

  struct vecX deltaState = vsub(stateP_hat, stateP);
  struct vecX stateP_hat_dot =  vadd3(mvmul(Aref, stateP), mvmul(B, actCmd), mvmul(L0, deltaState));
  struct vecX disturbance = mvmul(M0, deltaState);

  stateP_hat = vadd(stateP_hat, vscl(UPDATE_DT, stateP_hat_dot));

  return vneltX(NBR_ACT, disturbance);
}

void controllerMleso(control_t *control, const setpoint_t *setpoint,
                     const sensorData_t *sensors,
                     const state_t *state,
                     const stabilizerStep_t stabilizerStep)
{
  const struct matXX Kr = {.mat K_R, .rows NBR_ACT, .colums NBR_ACT};;
  control->controlMode = controlModeLegacy;
  
  // convert setpoint to vector
  //TODO: test is still yaw rate cmd
  struct vecX setpointV = mkvecX(NBR_STATES, {setpoint->thrust, setpoint->attitude.roll, setpoint->attitude.pitch,
          setpoint->attitudeRate.yaw, 0, 0, 0, 0, 0});

  //drone state in order for MLESO controller (att_rate xyz, vel zxy, att xyz)
  struct vecX stateP(NBR_STATES, {sensors->gyro.x, sensors->gyro.y, sensors->gyro.z,
      state->velocity.z, state->velocity.x, state->velocity.y,
      state->attitude.roll, state->attitude.pitch, state->attitude.yaw});
  
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {

    struct vecX stateFeedback = getStateFeedback(stateP);

    struct vecX actCmdPWM = vsub(mvmul(Kr, setpointV), stateFeedback);
    struct vecX actCmd = convert2ActCmd(actCmdPWM);

    struct vecX disMatched = mlesoModel(actCmd, stateP);
    struct vecX actCmdControl = vsub(actCmd, disMatched);

    struct vecX actCmdControlPWM = convert2PWMCmd(actCmd);

    control->thrust = actCmdControlPWM.vec[0];
    control->roll = saturateSignedInt16(actCmdControlPWM.vec[1]);
    control->pitch = saturateSignedInt16(actCmdControlPWM.vec[2]);
    control->yaw = saturateSignedInt16(actCmdControlPWM.vec[3]);

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

      stateP_hat = vzeroX(NBR_STATES);

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
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)
