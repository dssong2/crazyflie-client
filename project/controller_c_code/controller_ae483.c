#include "controller_ae483.h"
#include "stabilizer_types.h"
#include "power_distribution.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"

// Sensor measurements
// - tof (from the z ranger on the flow deck)
static uint16_t tof_count = 0;
static float tof_distance = 0.0f;
// - flow (from the optical flow sensor on the flow deck)
static uint16_t flow_count = 0;
static float flow_dpixelx = 0.0f;
static float flow_dpixely = 0.0f;

// Parameters
static bool use_observer = false;
static bool reset_observer = false;
static bool gains_are_set = false;
static bool control_started = false;

// State
static float p_x = 0.0f;
static float p_y = 0.0f;
static float p_z = 0.0f;
static float psi = 0.0f;
static float theta = 0.0f;
static float phi = 0.0f;
static float v_x = 0.0f;
static float v_y = 0.0f;
static float v_z = 0.0f;
static float w_x = 0.0f;
static float w_y = 0.0f;
static float w_z = 0.0f;

// Input
static float tau_x = 0.0f;
static float tau_y = 0.0f;
static float tau_z = 0.0f;
static float f_z = 0.0f;

// Desired position
static float p_x_des = 0.0f;
static float p_y_des = 0.0f;
static float p_z_des = 0.0f;

// Desired velocity (feed-forward). Plain position setpoints arrive with zero
// velocity (the commander zeroes every setpoint before decoding), so this only
// matters when the client sends full-state setpoints (move_smooth with
// velocity_feedforward = True).
static float v_x_des = 0.0f;
static float v_y_des = 0.0f;
static float v_z_des = 0.0f;

// Motor power command
static uint16_t m_1 = 0;
static uint16_t m_2 = 0;
static uint16_t m_3 = 0;
static uint16_t m_4 = 0;
// Measurements
static float n_x = 0.0f;
static float n_y = 0.0f;
static float r = 0.0f;
static float a_z = 0.0f;

// Constants
static float g = 9.81f;

// Observer gains from project/observer_design.ipynb (observer_gains.json,
// created 2026-11-09T11:34:48). Mocap observer: all 9 states corrected.
// Rows: p_x, p_y, p_z, psi, theta, phi, v_x, v_y, v_z
// Columns: errors in p_x, p_y, p_z, psi, theta, phi (estimate - mocap)
static const float L[9][6] = {
  {25.89153514f, 0.0f, 0.0f, 0.0f, 0.01161849242f, 0.0f},
  {0.0f, 34.51561754f, 0.0f, 0.0f, 0.0f, -0.02295224165f},
  {0.0f, 0.0f, 8.635718892f, 0.0f, 0.0f, 0.0f},
  {0.0f, 0.0f, 0.0f, 0.3446612598f, 0.0f, 0.0f},
  {6.571609982f, 0.0f, 0.0f, 0.0f, 0.1187164954f, 0.0f},
  {0.0f, -12.23070354f, 0.0f, 0.0f, 0.0f, 0.3750773607f},
  {97.14967006f, 0.0f, 0.0f, 0.0f, 0.3021999114f, 0.0f},
  {0.0f, 147.2248173f, 0.0f, 0.0f, 0.0f, -0.8008196608f},
  {0.0f, 0.0f, 27.37478053f, 0.0f, 0.0f, 0.0f}
};

// Mocap pose as received (marker deck, in the mocap frame). Written by
// ae483UpdateWithPose, which runs in the estimator task each time flight.py
// forwards a pose with drone_client.send_pose. The controller copies it out
// when mocap_new is set.
static volatile bool mocap_new = false;
static uint16_t mocap_count = 0;
static float pose_x = 0.0f;
static float pose_y = 0.0f;
static float pose_z = 0.0f;
static float pose_qx = 0.0f;
static float pose_qy = 0.0f;
static float pose_qz = 0.0f;
static float pose_qw = 1.0f;

// Mocap measurements in the start frame (origin on the ground below where the
// drone sat at the first pose after an observer reset, x axis along its
// initial heading). Same frame as the offline replay in observer_design.ipynb.
static float p_x_meas = 0.0f;
static float p_y_meas = 0.0f;
static float p_z_meas = 0.0f;
static float psi_meas = 0.0f;
static float theta_meas = 0.0f;
static float phi_meas = 0.0f;

// Start frame, set from the first pose after an observer reset
static bool mocap_origin_set = false;
static float mocap_x0 = 0.0f;
static float mocap_y0 = 0.0f;
static float mocap_z0 = 0.0f;
static float mocap_psi0 = 0.0f;

// Marker deck geometry (same values as transform_data_mocap in mytools.py)
static float d_1 = 0.016f;  // marker deck center above the body frame origin (m)
static float d_2 = 0.009f;  // body frame origin above the ground when sitting (m)

// Mocap older than this many 500 Hz steps (0.2 s) is not used for correction
static uint16_t mocap_age = 0;
static const uint16_t mocap_stale_steps = 100;

// Gains (these are set by flight.py - do not edit the values here)
static float eq_tau_x_cmd = 0.0f;
static float k_tau_x_cmd_p_y = 0.0f;
static float k_tau_x_cmd_phi = 0.0f;
static float k_tau_x_cmd_v_y = 0.0f;
static float k_tau_x_cmd_w_x = 0.0f;
static float k_tau_x_cmd_tau_x = 0.0f;
static float k_tau_x_cmd_e_y = 0.0f;
static float eq_tau_y_cmd = 0.0f;
static float k_tau_y_cmd_p_x = 0.0f;
static float k_tau_y_cmd_theta = 0.0f;
static float k_tau_y_cmd_v_x = 0.0f;
static float k_tau_y_cmd_w_y = 0.0f;
static float k_tau_y_cmd_tau_y = 0.0f;
static float k_tau_y_cmd_e_x = 0.0f;
static float eq_tau_z = 0.0f;
static float k_tau_z_psi = 0.0f;
static float k_tau_z_w_z = 0.0f;
static float eq_f_z = 0.0f;
static float k_f_z_p_z = 0.0f;
static float k_f_z_v_z = 0.0f;
static float k_f_z_e_z = 0.0f;
// For time delay
static float tau_x_cmd = 0.0f;    // tau_x command
static float tau_y_cmd = 0.0f;    // tau_y command
static float w_x_old = 0.0f;      // value of w_x from previous time step
static float w_y_old = 0.0f;      // value of w_y from previous time step
static float J_x = 1.846e-05;     // principal moment of inertia about x_B axis (box v1, project-MOI)
static float J_y = 1.899e-05;     // principal moment of inertia about y_B axis (box v1, project-MOI)
static float dt = 0.002f;         // time step (corresponds to 500 Hz)
// For integral action
static float e_x = 0.0f;    // integrated error in p_x
static float e_y = 0.0f;    // integrated error in p_y
static float e_z = 0.0f;    // integrated error in p_z

void ae483UpdateWithTOF(tofMeasurement_t *tof)
{
  tof_distance = tof->distance;
  tof_count++;
}

void ae483UpdateWithFlow(flowMeasurement_t *flow)
{
  flow_dpixelx = flow->dpixelx;
  flow_dpixely = flow->dpixely;
  flow_count++;
}

void ae483UpdateWithDistance(distanceMeasurement_t *meas)
{
  // If you have a loco positioning deck, this function will be called
  // each time a distance measurement is available. You will have to write
  // code to handle these measurements. These data are available:
  //
  //  meas->anchorId  uint8_t   id of anchor with respect to which distance was measured
  //  meas->x         float     x position of this anchor
  //  meas->y         float     y position of this anchor
  //  meas->z         float     z position of this anchor
  //  meas->distance  float     the measured distance
}

void ae483UpdateWithPosition(positionMeasurement_t *meas)
{
  // This function will be called each time you send an external position
  // measurement (x, y, z) from the client, e.g., from a motion capture system.
  // You will have to write code to handle these measurements. These data are
  // available:
  //
  //  meas->x         float     x component of external position measurement
  //  meas->y         float     y component of external position measurement
  //  meas->z         float     z component of external position measurement
}

void ae483UpdateWithPose(poseMeasurement_t *meas)
{
  // Called each time a mocap pose arrives (position x, y, z and orientation as
  // a quaternion, of the marker deck, in the mocap frame). Only store it here;
  // the controller converts it at its next step. This runs in a different task
  // from the controller, so keep it short.
  pose_x = meas->x;
  pose_y = meas->y;
  pose_z = meas->z;
  pose_qx = meas->quat.x;
  pose_qy = meas->quat.y;
  pose_qz = meas->quat.z;
  pose_qw = meas->quat.w;
  mocap_count++;
  mocap_new = true;
}

// Wrap an angle to [-pi, pi]
static float wrapAngle(float a)
{
  while (a > M_PI_F) {
    a -= 2.0f * M_PI_F;
  }
  while (a < -M_PI_F) {
    a += 2.0f * M_PI_F;
  }
  return a;
}

// Convert the latest stored pose to measurements in the start frame
static void processMocap(void)
{
  mocap_new = false;
  float x = pose_x;
  float y = pose_y;
  float z = pose_z;
  float qx = pose_qx;
  float qy = pose_qy;
  float qz = pose_qz;
  float qw = pose_qw;

  // ZYX (yaw, pitch, roll) Euler angles, the same convention as the model
  // (R = Rz(psi) Ry(theta) Rx(phi)) and as the mocap angles logged by the client
  float psi_q = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
  float sin_theta = fmaxf(-1.0f, fminf(1.0f, 2.0f * (qw * qy - qz * qx)));
  float theta_q = asinf(sin_theta);
  float phi_q = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));

  // Body frame origin is d_1 below the marker deck, along the body z axis
  // (third column of R)
  float r13 = 2.0f * (qx * qz + qw * qy);
  float r23 = 2.0f * (qy * qz - qw * qx);
  float r33 = 1.0f - 2.0f * (qx * qx + qy * qy);
  float b_x = x - d_1 * r13;
  float b_y = y - d_1 * r23;
  float b_z = z - d_1 * r33;

  bool first = !mocap_origin_set;
  if (first) {
    // Start frame: origin on the ground below the drone, x along its heading
    mocap_x0 = b_x;
    mocap_y0 = b_y;
    mocap_z0 = b_z - d_2;
    mocap_psi0 = psi_q;
    mocap_origin_set = true;
  }

  // Express in the start frame (rotate about z by -psi0; this leaves theta
  // and phi unchanged)
  float dx = b_x - mocap_x0;
  float dy = b_y - mocap_y0;
  float c0 = cosf(mocap_psi0);
  float s0 = sinf(mocap_psi0);
  p_x_meas = c0 * dx + s0 * dy;
  p_y_meas = -s0 * dx + c0 * dy;
  p_z_meas = b_z - mocap_z0;
  psi_meas = wrapAngle(psi_q - mocap_psi0);
  theta_meas = theta_q;
  phi_meas = phi_q;
  mocap_age = 0;

  if (first) {
    // Start the estimate at the first measurement, at rest
    p_x = p_x_meas;
    p_y = p_y_meas;
    p_z = p_z_meas;
    psi = psi_meas;
    theta = theta_meas;
    phi = phi_meas;
    v_x = 0.0f;
    v_y = 0.0f;
    v_z = 0.0f;
  }
}

void ae483UpdateWithData(const struct AE483Data* data)
{
  // This function will be called each time AE483-specific data are sent
  // from the client to the drone. You will have to write code to handle
  // these data. For the example AE483Data struct, these data are:
  //
  //  data->x         float
  //  data->y         float
  //  data->z         float
  //
  // Exactly what "x", "y", and "z" mean in this context is up to you.
}


void controllerAE483Init(void)
{
  // Do nothing
}

bool controllerAE483Test(void)
{
  // Do nothing (test is always passed)
  return true;
}

void controllerAE483(control_t *control,
                     const setpoint_t *setpoint,
                     const sensorData_t *sensors,
                     const state_t *state,
                     const stabilizerStep_t stabilizerStep)
{
  // Make sure this function only runs at 500 Hz
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    return;
  }

  // Desired position
  p_x_des = setpoint->position.x;
  p_y_des = setpoint->position.y;
  p_z_des = setpoint->position.z;

  // Desired velocity, in the world frame. The controller's velocities are in
  // the yaw frame, which matches the world frame for the yaw = 0 flights we fly.
  v_x_des = setpoint->velocity.x;
  v_y_des = setpoint->velocity.y;
  v_z_des = setpoint->velocity.z;

  // Measurements
  w_x = radians(sensors->gyro.x);
  w_y = radians(sensors->gyro.y);
  w_z = radians(sensors->gyro.z);
  a_z = g * sensors->acc.z;
  n_x = flow_dpixelx;
  n_y = flow_dpixely;
  r = tof_distance;

  // Torques by finite difference
  tau_x = J_x * (w_x - w_x_old) / dt;
  tau_y = J_y * (w_y - w_y_old) / dt;
  w_x_old = w_x;
  w_y_old = w_y;

  if (reset_observer) {
    p_x = 0.0f;
    p_y = 0.0f;
    p_z = 0.0f;
    psi = 0.0f;
    theta = 0.0f;
    phi = 0.0f;
    v_x = 0.0f;
    v_y = 0.0f;
    v_z = 0.0f;
    // Take a new start frame from the next mocap pose
    mocap_origin_set = false;
    reset_observer = false;
  }

  // Mocap: convert a newly arrived pose, otherwise count how old the last one is
  if (mocap_new) {
    processMocap();
  } else if (mocap_age < UINT16_MAX) {
    mocap_age++;
  }

  // State estimates
  if (!gains_are_set) {
    // Do not update the state estimates (either with the default or
    // the custom observer) until after gains are set, a process that
    // can take several seconds.

  } else if (use_observer) {

    // Measurement error C x - y (zero, i.e. prediction only, if mocap is
    // missing or older than mocap_stale_steps)
    float err[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    if (mocap_origin_set && mocap_age <= mocap_stale_steps) {
      err[0] = p_x - p_x_meas;
      err[1] = p_y - p_y_meas;
      err[2] = p_z - p_z_meas;
      err[3] = wrapAngle(psi - psi_meas);
      err[4] = theta - theta_meas;
      err[5] = phi - phi_meas;
    }

    // Correction L (C x - y) for each state
    float corr[9];
    for (int i = 0; i < 9; i++) {
      corr[i] = 0.0f;
      for (int j = 0; j < 6; j++) {
        corr[i] += L[i][j] * err[j];
      }
    }

    // State derivatives, all from the current estimate (same as run_observer
    // in observer_design.ipynb), then one forward Euler step
    float p_x_dot = v_x - corr[0];
    float p_y_dot = v_y - corr[1];
    float p_z_dot = v_z - corr[2];
    float psi_dot = w_z - corr[3];
    float theta_dot = w_y - corr[4];
    float phi_dot = w_x - corr[5];
    float v_x_dot = g * theta - corr[6];
    float v_y_dot = -g * phi - corr[7];
    float v_z_dot = a_z - g - corr[8];

    p_x += dt * p_x_dot;
    p_y += dt * p_y_dot;
    p_z += dt * p_z_dot;
    psi += dt * psi_dot;
    theta += dt * theta_dot;
    phi += dt * phi_dot;
    v_x += dt * v_x_dot;
    v_y += dt * v_y_dot;
    v_z += dt * v_z_dot;

  } else {
    p_x = state->position.x;
    p_y = state->position.y;
    p_z = state->position.z;
    psi = radians(state->attitude.yaw);
    theta = - radians(state->attitude.pitch);
    phi = radians(state->attitude.roll);
    v_x = state->velocity.x*cosf(psi)*cosf(theta) + state->velocity.y*sinf(psi)*cosf(theta) - state->velocity.z*sinf(theta);
    v_y = state->velocity.x*(sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi)) + state->velocity.y*(sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi)) + state->velocity.z*sinf(phi)*cosf(theta);
    v_z = state->velocity.x*(sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi)) + state->velocity.y*(-sinf(phi)*cosf(psi) + sinf(psi)*sinf(theta)*cosf(phi)) + state->velocity.z*cosf(phi)*cosf(theta);
  }

  // A large initial heading error is outside this controller's hover model.
  // Check only before starting; do not cut the motors for a heading change in flight.
  bool bad_start_heading = !control_started && (!isfinite(psi) || fabsf(psi) > radians(5.0f));
  // With the mocap observer, never start without a mocap start frame
  bool no_mocap_start = !control_started && use_observer && !mocap_origin_set;
  if (setpoint->mode.z == modeDisable || !gains_are_set || bad_start_heading || no_mocap_start) {
    // If there is no desired position, then all
    // motor power commands should be zero

    if (setpoint->mode.z == modeDisable || !gains_are_set) {
      control_started = false;
    }
    e_x = 0.0f;
    e_y = 0.0f;
    e_z = 0.0f;
    m_1 = 0;
    m_2 = 0;
    m_3 = 0;
    m_4 = 0;
  } else {
    // Otherwise, motor power commands should be
    // chosen by the controller
    control_started = true;

    // Estimate e_x, e_y, e_z by Forward Euler
    e_x += dt * (p_x - p_x_des);
    e_y += dt * (p_y - p_y_des);
    e_z += dt * (p_z - p_z_des);

    tau_x_cmd = eq_tau_x_cmd - (k_tau_x_cmd_p_y * (p_y - p_y_des) + k_tau_x_cmd_phi * phi + k_tau_x_cmd_v_y * (v_y - v_y_des) + k_tau_x_cmd_w_x * w_x + k_tau_x_cmd_tau_x * tau_x + k_tau_x_cmd_e_y * e_y);
    tau_y_cmd = eq_tau_y_cmd - (k_tau_y_cmd_p_x * (p_x - p_x_des) + k_tau_y_cmd_theta * theta + k_tau_y_cmd_v_x * (v_x - v_x_des) + k_tau_y_cmd_w_y * w_y + k_tau_y_cmd_tau_y * tau_y + k_tau_y_cmd_e_x * e_x);
    tau_z = eq_tau_z - (k_tau_z_psi * psi + k_tau_z_w_z * w_z);
    f_z = eq_f_z - (k_f_z_p_z * (p_z - p_z_des) + k_f_z_v_z * (v_z - v_z_des) + k_f_z_e_z * e_z);
    // Method of power distrubution
    // (k_F, k_M corrected by the measured hover thrust - controller_design.ipynb section 4)
    m_1 = limitUint16( -4424470.1f * tau_x_cmd -4424470.1f * tau_y_cmd -41483627.4f * tau_z + 143795.3f * f_z );
    m_2 = limitUint16( -4424470.1f * tau_x_cmd + 4424470.1f * tau_y_cmd + 41483627.4f * tau_z + 143795.3f * f_z );
    m_3 = limitUint16( 4424470.1f * tau_x_cmd + 4424470.1f * tau_y_cmd -41483627.4f * tau_z + 143795.3f * f_z );
    m_4 = limitUint16( 4424470.1f * tau_x_cmd -4424470.1f * tau_y_cmd + 41483627.4f * tau_z + 143795.3f * f_z );
  }
  
  // Do not spin the motors unless the gains have been sent by flight.py.
  // Without this, a drone that never received its gains would try to fly
  // with all gains equal to zero, which would end badly.
  if (!gains_are_set) {
    m_1 = 0;
    m_2 = 0;
    m_3 = 0;
    m_4 = 0;
  }

  // Apply motor power commands
  control->m1 = m_1;
  control->m2 = m_2;
  control->m3 = m_3;
  control->m4 = m_4;
}


//              1234567890123456789012345 <-- max total length
//              group   .name
LOG_GROUP_START(ae483log)
LOG_ADD(LOG_UINT16,      num_tof,                &tof_count)
LOG_ADD(LOG_UINT16,      num_flow,               &flow_count)
LOG_ADD(LOG_FLOAT,       p_x,                    &p_x)
LOG_ADD(LOG_FLOAT,       p_y,                    &p_y)
LOG_ADD(LOG_FLOAT,       p_z,                    &p_z)
LOG_ADD(LOG_FLOAT,       psi,                    &psi)
LOG_ADD(LOG_FLOAT,       theta,                  &theta)
LOG_ADD(LOG_FLOAT,       phi,                    &phi)
LOG_ADD(LOG_FLOAT,       v_x,                    &v_x)
LOG_ADD(LOG_FLOAT,       v_y,                    &v_y)
LOG_ADD(LOG_FLOAT,       v_z,                    &v_z)
LOG_ADD(LOG_FLOAT,       w_x,                    &w_x)
LOG_ADD(LOG_FLOAT,       w_y,                    &w_y)
LOG_ADD(LOG_FLOAT,       w_z,                    &w_z)
LOG_ADD(LOG_FLOAT,       p_x_des,                &p_x_des)
LOG_ADD(LOG_FLOAT,       p_y_des,                &p_y_des)
LOG_ADD(LOG_FLOAT,       p_z_des,                &p_z_des)
LOG_ADD(LOG_FLOAT,       v_x_des,                &v_x_des)
LOG_ADD(LOG_FLOAT,       v_y_des,                &v_y_des)
LOG_ADD(LOG_FLOAT,       v_z_des,                &v_z_des)
LOG_ADD(LOG_FLOAT,       tau_x,                  &tau_x)
LOG_ADD(LOG_FLOAT,       tau_y,                  &tau_y)
LOG_ADD(LOG_FLOAT,       tau_z,                  &tau_z)
LOG_ADD(LOG_FLOAT,       f_z,                    &f_z)
LOG_ADD(LOG_UINT16,      m_1,                    &m_1)
LOG_ADD(LOG_UINT16,      m_2,                    &m_2)
LOG_ADD(LOG_UINT16,      m_3,                    &m_3)
LOG_ADD(LOG_UINT16,      m_4,                    &m_4)
LOG_ADD(LOG_FLOAT,       tau_x_cmd,              &tau_x_cmd)
LOG_ADD(LOG_FLOAT,       tau_y_cmd,              &tau_y_cmd)
LOG_ADD(LOG_FLOAT,             e_x,                    &e_x)
LOG_ADD(LOG_FLOAT,             e_y,                    &e_y)
LOG_ADD(LOG_FLOAT,             e_z,                    &e_z) 
LOG_ADD(LOG_FLOAT,       n_x,                    &n_x)
LOG_ADD(LOG_FLOAT,       n_y,                    &n_y)
LOG_ADD(LOG_FLOAT,       r,                      &r)
LOG_ADD(LOG_FLOAT,       a_z,                    &a_z)
LOG_ADD(LOG_UINT16,      num_mocap,              &mocap_count)
LOG_ADD(LOG_UINT16,      mocap_age,              &mocap_age)
LOG_ADD(LOG_FLOAT,       p_x_mocap,              &p_x_meas)
LOG_ADD(LOG_FLOAT,       p_y_mocap,              &p_y_meas)
LOG_ADD(LOG_FLOAT,       p_z_mocap,              &p_z_meas)
LOG_ADD(LOG_FLOAT,       psi_mocap,              &psi_meas)
LOG_ADD(LOG_FLOAT,       theta_mocap,            &theta_meas)
LOG_ADD(LOG_FLOAT,       phi_mocap,              &phi_meas)
LOG_GROUP_STOP(ae483log)

//                1234567890123456789012345 <-- max total length
//                group   .name
PARAM_GROUP_START(ae483par)
PARAM_ADD(PARAM_UINT8,     use_observer,            &use_observer)
PARAM_ADD(PARAM_UINT8,     reset_observer,          &reset_observer)
PARAM_ADD(PARAM_UINT8,     gains_are_set,           &gains_are_set)
PARAM_GROUP_STOP(ae483par)

//                1234567890123456789012345 <-- max total length
//                group .name
PARAM_GROUP_START(ae483g)
PARAM_ADD(PARAM_FLOAT,   eq_tau_x_cmd,            &eq_tau_x_cmd)
PARAM_ADD(PARAM_FLOAT,   k_tau_x_cmd_p_y,         &k_tau_x_cmd_p_y)
PARAM_ADD(PARAM_FLOAT,   k_tau_x_cmd_phi,         &k_tau_x_cmd_phi)
PARAM_ADD(PARAM_FLOAT,   k_tau_x_cmd_v_y,         &k_tau_x_cmd_v_y)
PARAM_ADD(PARAM_FLOAT,   k_tau_x_cmd_w_x,         &k_tau_x_cmd_w_x)
PARAM_ADD(PARAM_FLOAT,   k_tau_x_cmd_tau_x,       &k_tau_x_cmd_tau_x)
PARAM_ADD(PARAM_FLOAT,   k_tau_x_cmd_e_y,         &k_tau_x_cmd_e_y)
PARAM_ADD(PARAM_FLOAT,   eq_tau_y_cmd,            &eq_tau_y_cmd)
PARAM_ADD(PARAM_FLOAT,   k_tau_y_cmd_p_x,         &k_tau_y_cmd_p_x)
PARAM_ADD(PARAM_FLOAT,   k_tau_y_cmd_theta,       &k_tau_y_cmd_theta)
PARAM_ADD(PARAM_FLOAT,   k_tau_y_cmd_v_x,         &k_tau_y_cmd_v_x)
PARAM_ADD(PARAM_FLOAT,   k_tau_y_cmd_w_y,         &k_tau_y_cmd_w_y)
PARAM_ADD(PARAM_FLOAT,   k_tau_y_cmd_tau_y,       &k_tau_y_cmd_tau_y)
PARAM_ADD(PARAM_FLOAT,   k_tau_y_cmd_e_x,         &k_tau_y_cmd_e_x)
PARAM_ADD(PARAM_FLOAT,   eq_tau_z,                &eq_tau_z)
PARAM_ADD(PARAM_FLOAT,   k_tau_z_psi,             &k_tau_z_psi)
PARAM_ADD(PARAM_FLOAT,   k_tau_z_w_z,             &k_tau_z_w_z)
PARAM_ADD(PARAM_FLOAT,   eq_f_z,                  &eq_f_z)
PARAM_ADD(PARAM_FLOAT,   k_f_z_p_z,               &k_f_z_p_z)
PARAM_ADD(PARAM_FLOAT,   k_f_z_v_z,               &k_f_z_v_z)
PARAM_ADD(PARAM_FLOAT,   k_f_z_e_z,               &k_f_z_e_z)
PARAM_GROUP_STOP(ae483g)

