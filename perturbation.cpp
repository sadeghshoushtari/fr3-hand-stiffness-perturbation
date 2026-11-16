// perturbation.cpp
//
// Measures the operator's arm stiffness at the FR3 handle. Single self-contained file.
//
// The robot moves to the home pose of the teleoperation slave, then holds the handle with a known Cartesian
// impedance and shifts its equilibrium point in small steps (+-x, +-y, +-z, shuffled, random rests). With the hand
// on the handle the handle moves only part of each step: K_h dx = K_r (dx_d - dx), see analyze_perturbation.py.
// A trial whose label starts with "rot_" turns the equilibrium orientation instead (+-20 deg about x, y, z of the
// base frame through the end-effector point), with a lower rotational K_r so the hand takes a measurable share.
// The elastography sensors are logged by a separate program (sensors.py on this PC over USB, or
// laptop_sensor_logger.py on a laptop); each trial's start and end are written here as Unix time so the two logs
// can be matched by clock time.
//
// Build: g++ -std=c++17 -O2 -Wall -Wextra perturbation.cpp -I/usr/include/eigen3 -o perturbation
//            -lfranka -pthread
// Usage: ./perturbation [<franka-ip>] [--out <dir>] [--console]
//        (default robot 169.254.105.210, default log dir perturbation_logs/<date_time>)
//
// Default: MOSAIC drives the perturbations (see below). Ctrl+C stops the robot at once.
//
// --console: trials from the console instead: type a label for the operator's state (e.g. empty, relaxed,
// cocontract_medium; rot_empty, rot_relaxed for rotation steps) and press Enter to run one trial; "q" quits.
//
// MOSAIC mode (mosaic/session_calibration.json), the default: the
// equilibrium offset comes as 6 doubles (dx, dy, dz in m, rx, ry, rz in rad; the rotations are ignored) on UDP port
// 50900; each axis is clamped to +-20 mm, the offset follows the command at most 0.1 m/s and returns to zero when no
// packet came for 0.25 s. The robot state goes to 127.0.0.1:50910 at 1 kHz as 15 doubles: t, x, y, z, x_d, y_d, z_d,
// F_ext x, y, z (O_F_ext_hat_K), q_w, q_x, q_y, q_z, K_r. The console then only takes "q".
//
// Files: robot.csv (1 kHz), trials.csv (label, start and end of each trial, also as Unix time).
//
// The home pose, default behaviour, motion generator, damping law, nullspace term, torque-rate limit, collision
// thresholds and joint-velocity limits are copied unchanged from the slave (Slave/slave_config.hpp,
// Slave/examples_common.cpp, Slave/slave_control_utils.hpp), so this program behaves like the teleoperation
// slave except for where the equilibrium point comes from.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

namespace {
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector7i = Eigen::Matrix<int, 7, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Matrix67d = Eigen::Matrix<double, 6, 7>;

constexpr double kPi = 3.14159265358979323846;

// ---- Values of the teleoperation slave (Slave/slave_config.hpp), unchanged ----
constexpr std::array<double, 7> kInitialJointConfiguration = {0.0, -kPi / 4.0, 0.0, -3.0 * kPi / 4.0,
                                                              0.0, kPi / 2.0,  kPi / 4.0};
constexpr double kInitialMotionSpeedFactor = 0.35;
constexpr double kCollisionThreshold = 35.0;
constexpr double kBaselineTranslationalStiffness = 500.0;  // N/m, reference of the damping law
constexpr double kBaselineTranslationalDamping = 20.0;     // Ns/m
constexpr double kBaselineRotationalStiffness = 80.0;      // Nm/rad
constexpr double kBaselineRotationalDamping = 0.5;         // Nms/rad
constexpr double kNullspaceStiffness = 8.0;
constexpr double kNullspaceDamping = 2.5;
constexpr double kDampedInverseLambda = 0.05;
constexpr double kMaxDeltaTorquePerCycleNm = 1.0;
constexpr std::array<double, 7> kJointVelocityGuardLimitsRadps = {2.0, 1.0, 1.5, 1.25, 3.0, 1.5, 3.0};
constexpr double kJointVelocityGuardStopRatio = 0.85;

// ---- Perturbation test ----
constexpr double kRobotTranslationalStiffness = 1000.0;  // N/m, known K_r for the robot-as-spring estimate
constexpr double kRobotRotationalStiffness = kBaselineRotationalStiffness;  // holds the handle orientation
constexpr double kStepAmplitudeM = 0.015;
constexpr double kStepRampS = 0.15;    // raised-cosine ramp to and from the offset
constexpr double kStepHoldS = 1.0;
constexpr double kRestMinS = 1.5;      // random rest between steps, so the operator cannot anticipate
constexpr double kRestMaxS = 2.5;
constexpr int kStepRepetitions = 2;    // each of the 6 directions this many times per trial
constexpr double kTrialLeadInS = 3.0;  // still hold before the first step (baseline)
constexpr int kMaxSteps = 6 * kStepRepetitions;
// Rotation trials: K_r about all three axes during the trial (blended from and back to the holding value)
// The slave's orientation law uses the quaternion vector part, sin(angle / 2): for small angles the torque per rad is
// half the gain, so the effective rotational K_r is kRotationTrialStiffness / 2 (written to trials.csv).
constexpr double kRotationTrialStiffness = 20.0;  // gain, Nm per unit quaternion error; effective 10 Nm/rad
constexpr double kRotationStepRad = 20.0 * kPi / 180.0;  // at most about 3.5 Nm on the hand
// Longer ramp than the translation steps: 20 deg in 0.6 s peaks at 0.9 rad/s, below the wrist joint guard
// (joint 6: 0.85 * 1.5 rad/s); longer hold so the handle settles before the analysis window.
constexpr double kRotationRampS = 0.6;
constexpr double kRotationHoldS = 1.5;
constexpr double kRotationStiffnessBlendS = 0.5;  // lead-in is 3 s, so the blend ends before the first step

// --udp mode
constexpr int kUdpOffsetPort = 50900;
constexpr int kUdpStatePort = 50910;
constexpr const char* kUdpStateHost = "127.0.0.1";
constexpr double kUdpMaxOffsetM = 0.02;
constexpr double kUdpOffsetRateMps = 0.1;
constexpr double kUdpTimeoutS = 0.25;

// Safety: end the control loop when exceeded
constexpr double kAbortPositionErrorM = 0.05;
constexpr double kAbortSpeedMps = 0.5;
constexpr double kAbortOrientationErrorRad = 35.0 * kPi / 180.0;  // above the 20 deg step
constexpr double kAbortAngularSpeedRadps = 2.0;

volatile std::sig_atomic_t g_stop_requested = 0;

void sigint_handler(int) {
  g_stop_requested = 1;
}

double elapsed_seconds(const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

double unix_seconds() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// Slave/slave_control_utils.hpp: limit the change of the commanded torque per cycle.
std::array<double, 7> saturate_torque_rate(const Vector7d& calculated, const std::array<double, 7>& previous) {
  std::array<double, 7> result{};
  for (std::size_t i = 0; i < 7; ++i) {
    const double delta = calculated[static_cast<int>(i)] - previous[i];
    result[i] = previous[i] + std::clamp(delta, -kMaxDeltaTorquePerCycleNm, kMaxDeltaTorquePerCycleNm);
  }
  return result;
}

// Slave/examples_common.cpp (Franka Emika example, Apache-2.0): collision behaviour and impedances set before
// control, and the joint motion generator used to reach the home pose.
void setDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

class MotionGenerator {
 public:
  MotionGenerator(double speed_factor, const std::array<double, 7> q_goal) : q_goal_(q_goal.data()) {
    dq_max_ *= speed_factor;
    ddq_max_start_ *= speed_factor;
    ddq_max_goal_ *= speed_factor;
    dq_max_sync_.setZero();
    q_start_.setZero();
    delta_q_.setZero();
    t_1_sync_.setZero();
    t_2_sync_.setZero();
    t_f_sync_.setZero();
    q_1_.setZero();
  }

  franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period) {
    time_ += period.toSec();

    if (time_ == 0.0) {
      q_start_ = Vector7d(robot_state.q_d.data());
      delta_q_ = q_goal_ - q_start_;
      calculateSynchronizedValues();
    }

    Vector7d delta_q_d;
    bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

    std::array<double, 7> joint_positions;
    Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
    franka::JointPositions output(joint_positions);
    output.motion_finished = motion_finished;
    return output;
  }

 private:
  bool calculateDesiredValues(double t, Vector7d* delta_q_d) const {
    Vector7i sign_delta_q;
    sign_delta_q << delta_q_.cwiseSign().cast<int>();
    Vector7d t_d = t_2_sync_ - t_1_sync_;
    Vector7d delta_t_2_sync = t_f_sync_ - t_2_sync_;
    std::array<bool, 7> joint_motion_finished{};

    for (size_t i = 0; i < 7; i++) {
      if (std::abs(delta_q_[i]) < kDeltaQMotionFinished) {
        (*delta_q_d)[i] = 0;
        joint_motion_finished[i] = true;
      } else {
        if (t < t_1_sync_[i]) {
          (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] *
                            (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
        } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
          (*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
        } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
          (*delta_q_d)[i] =
              delta_q_[i] + 0.5 *
                                (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                                     (t - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                                     std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
                                 (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                                dq_max_sync_[i] * sign_delta_q[i];
        } else {
          (*delta_q_d)[i] = delta_q_[i];
          joint_motion_finished[i] = true;
        }
      }
    }
    return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(), [](bool x) { return x; });
  }

  void calculateSynchronizedValues() {
    Vector7d dq_max_reach(dq_max_);
    Vector7d t_f = Vector7d::Zero();
    Vector7d delta_t_2 = Vector7d::Zero();
    Vector7d t_1 = Vector7d::Zero();
    Vector7d delta_t_2_sync = Vector7d::Zero();
    Vector7i sign_delta_q;
    sign_delta_q << delta_q_.cwiseSign().cast<int>();

    for (size_t i = 0; i < 7; i++) {
      if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
        if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
                                     3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))) {
          dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] *
                                      (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                      (ddq_max_start_[i] + ddq_max_goal_[i]));
        }
        t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
        delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
        t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
      }
    }
    double max_t_f = t_f.maxCoeff();
    for (size_t i = 0; i < 7; i++) {
      if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
        double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
        double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
        double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
        double delta = b * b - 4.0 * a * c;
        if (delta < 0.0) {
          delta = 0.0;
        }
        dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
        t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
        delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
        t_f_sync_[i] = (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
        t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
        q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
      }
    }
  }

  static constexpr double kDeltaQMotionFinished = 1e-6;
  const Vector7d q_goal_;

  Vector7d q_start_;
  Vector7d delta_q_;

  Vector7d dq_max_sync_;
  Vector7d t_1_sync_;
  Vector7d t_2_sync_;
  Vector7d t_f_sync_;
  Vector7d q_1_;

  double time_ = 0.0;

  Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
  Vector7d ddq_max_start_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
  Vector7d ddq_max_goal_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
};

// ---- Step schedule ----
struct StepPlan {
  double onset_s;
  int axis;  // 0..2 translation along x, y, z; 3..5 rotation about x, y, z
  double sign;
};

enum Phase : int { kIdle = 0, kRest = 1, kRampUp = 2, kHold = 3, kRampDown = 4 };

enum AbortReason : int { kNone = 0, kPositionError = 1, kSpeed = 2, kJointVelocity = 3, kOrientationError = 4,
                         kAngularSpeed = 5 };

double raised_cosine(double x) {
  x = std::clamp(x, 0.0, 1.0);
  return 0.5 * (1.0 - std::cos(kPi * x));
}

double ramp_s(bool rotation) {
  return rotation ? kRotationRampS : kStepRampS;
}

double hold_s(bool rotation) {
  return rotation ? kRotationHoldS : kStepHoldS;
}

// Offset profile (0..1) and phase of one step at trial time t.
double step_profile(double t, double onset, bool rotation, Phase* phase) {
  const double s = t - onset;
  const double ramp = ramp_s(rotation), hold = hold_s(rotation);
  if (s < 0.0) {
    *phase = kRest;
    return 0.0;
  }
  if (s < ramp) {
    *phase = kRampUp;
    return raised_cosine(s / ramp);
  }
  if (s < ramp + hold) {
    *phase = kHold;
    return 1.0;
  }
  if (s < 2.0 * ramp + hold) {
    *phase = kRampDown;
    return 1.0 - raised_cosine((s - ramp - hold) / ramp);
  }
  *phase = kRest;
  return 0.0;
}

std::vector<StepPlan> make_plan(std::mt19937& rng, bool rotation) {
  std::vector<StepPlan> plan;
  for (int r = 0; r < kStepRepetitions; ++r) {
    for (int axis = 0; axis < 3; ++axis) {
      plan.push_back({0.0, axis + (rotation ? 3 : 0), 1.0});
      plan.push_back({0.0, axis + (rotation ? 3 : 0), -1.0});
    }
  }
  std::shuffle(plan.begin(), plan.end(), rng);
  std::uniform_real_distribution<double> rest(kRestMinS, kRestMaxS);
  double t = kTrialLeadInS;
  for (auto& step : plan) {
    step.onset_s = t;
    t += 2.0 * ramp_s(rotation) + hold_s(rotation) + rest(rng);
  }
  return plan;
}

double trial_duration(const std::array<StepPlan, kMaxSteps>& plan) {
  const bool rotation = plan[0].axis >= 3;
  return plan[kMaxSteps - 1].onset_s + 2.0 * ramp_s(rotation) + hold_s(rotation) + kRestMinS;
}

// ---- Logging: single producer (control callback), single consumer (writer thread) ----
struct LogRecord {
  double t;
  int trial;
  int step;
  int phase;
  int axis;
  double sign;
  double desired[3];
  double position[3];
  double quaternion[4];  // w, x, y, z
  double wrench[6];      // O_F_ext_hat_K
  double velocity[3];
  double desired_rotation[3];  // equilibrium orientation offset from home, rotation vector in the base frame, rad
};

constexpr std::size_t kLogCapacity = 1 << 15;

struct LogRing {
  std::array<LogRecord, kLogCapacity> records;
  std::atomic<std::size_t> head{0};
  std::atomic<std::size_t> tail{0};
  std::atomic<uint64_t> dropped{0};

  void push(const LogRecord& record) {
    const std::size_t h = head.load(std::memory_order_relaxed);
    if (h - tail.load(std::memory_order_acquire) >= kLogCapacity) {
      dropped.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    records[h % kLogCapacity] = record;
    head.store(h + 1, std::memory_order_release);
  }
};

std::string default_log_dir() {
  const std::time_t now = std::time(nullptr);
  char name[64];
  std::strftime(name, sizeof(name), "perturbation_logs/%Y%m%d_%H%M%S", std::localtime(&now));
  return name;
}
}  // namespace

int main(int argc, char** argv) {
  std::string log_dir = default_log_dir();
  std::string robot_ip = "169.254.105.210";
  bool udp_mode = true;
  bool usage_error = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--out" && i + 1 < argc) {
      log_dir = argv[++i];
    } else if (arg == "--console") {
      udp_mode = false;
    } else if (arg == "--udp") {
      udp_mode = true;
    } else if (i == 1 && arg.rfind("--", 0) != 0) {
      robot_ip = arg;
    } else {
      usage_error = true;
    }
  }
  if (usage_error) {
    std::cerr << "Usage: " << argv[0] << " [<franka-ip>] [--out <dir>] [--console]\n";
    return 1;
  }

  std::signal(SIGINT, sigint_handler);

  std::filesystem::create_directories(log_dir);
  std::FILE* robot_file = std::fopen((log_dir + "/robot.csv").c_str(), "w");
  std::FILE* trial_file = std::fopen((log_dir + "/trials.csv").c_str(), "w");
  if (!robot_file || !trial_file) {
    std::perror("log files");
    return 1;
  }
  std::fprintf(robot_file,
               "t_s,trial,step,phase,axis,sign,xd,yd,zd,x,y,z,qw,qx,qy,qz,"
               "fx,fy,fz,mx,my,mz,vx,vy,vz,rdx,rdy,rdz\n");
  std::fprintf(trial_file,
               "trial,label,start_s,end_s,start_unix_s,end_unix_s,completed,k_robot_n_per_m,step_m,ramp_s,hold_s,"
               "kind,k_robot_nm_per_rad,step_rad\n");
  std::fflush(nullptr);

  const auto clock_start = std::chrono::steady_clock::now();

  auto log_ring = std::make_unique<LogRing>();
  std::array<StepPlan, kMaxSteps> plan{};
  std::atomic<int> requested_trial{0};
  std::atomic<int> finished_trial{0};
  std::atomic<int> abort_reason{kNone};
  std::atomic<bool> control_running{false};

  // --udp: latest offset command (control callback reads it) and the state socket (writer thread)
  std::array<std::atomic<double>, 3> udp_target{};
  std::atomic<double> udp_target_time{-1.0};
  int state_socket = -1;
  sockaddr_in state_address{};
  std::thread udp_thread;
  if (udp_mode) {
    const int offset_socket = socket(AF_INET, SOCK_DGRAM, 0);
    state_socket = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in listen_address{};
    listen_address.sin_family = AF_INET;
    listen_address.sin_addr.s_addr = htonl(INADDR_ANY);
    listen_address.sin_port = htons(kUdpOffsetPort);
    state_address.sin_family = AF_INET;
    state_address.sin_port = htons(kUdpStatePort);
    inet_pton(AF_INET, kUdpStateHost, &state_address.sin_addr);
    timeval timeout{0, 200000};
    if (offset_socket < 0 || state_socket < 0 ||
        setsockopt(offset_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0 ||
        bind(offset_socket, reinterpret_cast<sockaddr*>(&listen_address), sizeof(listen_address)) != 0) {
      std::perror("udp sockets");
      return 1;
    }
    udp_thread = std::thread([&, offset_socket]() {
      double values[6];
      while (!g_stop_requested) {
        const ssize_t n = recv(offset_socket, values, sizeof(values), 0);
        if (n < static_cast<ssize_t>(3 * sizeof(double))) {
          continue;  // timeout or short packet
        }
        if (!std::isfinite(values[0]) || !std::isfinite(values[1]) || !std::isfinite(values[2])) {
          continue;
        }
        for (int i = 0; i < 3; ++i) {
          udp_target[i].store(std::clamp(values[i], -kUdpMaxOffsetM, kUdpMaxOffsetM));
        }
        udp_target_time.store(elapsed_seconds(clock_start));
      }
      close(offset_socket);
    });
  }

  std::thread writer_thread([&]() {
    while (true) {
      const bool stopping = g_stop_requested && !control_running.load();
      const std::size_t head = log_ring->head.load(std::memory_order_acquire);
      std::size_t tail = log_ring->tail.load(std::memory_order_relaxed);
      for (; tail < head; ++tail) {
        const LogRecord& r = log_ring->records[tail % kLogCapacity];
        if (state_socket >= 0) {
          const double packet[15] = {r.t, r.position[0], r.position[1], r.position[2], r.desired[0], r.desired[1],
                                     r.desired[2], r.wrench[0], r.wrench[1], r.wrench[2], r.quaternion[0],
                                     r.quaternion[1], r.quaternion[2], r.quaternion[3],
                                     kRobotTranslationalStiffness};
          sendto(state_socket, packet, sizeof(packet), 0, reinterpret_cast<const sockaddr*>(&state_address),
                 sizeof(state_address));
        }
        std::fprintf(robot_file,
                     "%.4f,%d,%d,%d,%d,%.0f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                     "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f\n",
                     r.t, r.trial, r.step, r.phase, r.axis, r.sign, r.desired[0], r.desired[1], r.desired[2],
                     r.position[0], r.position[1], r.position[2], r.quaternion[0], r.quaternion[1],
                     r.quaternion[2], r.quaternion[3], r.wrench[0], r.wrench[1], r.wrench[2], r.wrench[3],
                     r.wrench[4], r.wrench[5], r.velocity[0], r.velocity[1], r.velocity[2], r.desired_rotation[0],
                     r.desired_rotation[1], r.desired_rotation[2]);
      }
      log_ring->tail.store(tail, std::memory_order_release);
      std::fflush(robot_file);
      if (stopping) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::fclose(robot_file);
  });

  // Console: one trial per label. Runs until "q", Ctrl+C or the end of the control loop.
  std::thread console_thread([&]() {
    std::mt19937 rng(static_cast<uint32_t>(std::time(nullptr)));
    int trial = 0;
    while (!g_stop_requested) {
      if (!control_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      std::cout << "\nLabel for trial " << trial + 1 << " (Enter = start, q = quit): " << std::flush;

      std::string label;
      while (!g_stop_requested) {
        fd_set input;
        FD_ZERO(&input);
        FD_SET(STDIN_FILENO, &input);
        timeval wait{0, 200000};
        if (select(STDIN_FILENO + 1, &input, nullptr, nullptr, &wait) > 0) {
          std::getline(std::cin, label);
          break;
        }
      }
      if (g_stop_requested) {
        break;
      }
      if (label == "q") {
        g_stop_requested = 1;
        break;
      }
      if (udp_mode) {
        std::cout << "MOSAIC drives the perturbations (start with --console for console trials); type q to quit.\n";
        continue;
      }
      if (label.empty()) {
        label = "trial";
      }

      const bool rotation = label.rfind("rot_", 0) == 0;
      const std::vector<StepPlan> steps = make_plan(rng, rotation);
      std::copy(steps.begin(), steps.end(), plan.begin());
      ++trial;
      const double start_s = elapsed_seconds(clock_start);
      const double start_unix = unix_seconds();
      std::cout << "[TRIAL " << trial << "] " << label << ": " << steps.size() << (rotation ? " rotation" : "")
                << " steps, about " << static_cast<int>(trial_duration(plan))
                << " s. Hold the handle, do not react to the steps.\n"
                << std::flush;
      // Stored after the plan is copied; the control callback reads it with a seq_cst load before the plan.
      requested_trial.store(trial);

      while (!g_stop_requested && finished_trial.load() < trial) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      std::fprintf(trial_file, "%d,%s,%.4f,%.4f,%.3f,%.3f,%d,%.1f,%.4f,%.3f,%.3f,%s,%.1f,%.4f\n", trial,
                   label.c_str(), start_s, elapsed_seconds(clock_start), start_unix, unix_seconds(),
                   finished_trial.load() >= trial ? 1 : 0, kRobotTranslationalStiffness, kStepAmplitudeM,
                   ramp_s(rotation), hold_s(rotation), rotation ? "rotation" : "translation",
                   0.5 * (rotation ? kRotationTrialStiffness : kRobotRotationalStiffness),
                   rotation ? kRotationStepRad : 0.0);
      std::fflush(trial_file);
      if (!g_stop_requested) {
        std::cout << "[TRIAL " << trial << "] done.\n";
      }
    }
    std::fclose(trial_file);
  });

  try {
    std::cerr << "[FRANKA] Connecting to " << robot_ip << "...\n";
    franka::Robot robot(robot_ip);
    std::cerr << "[FRANKA] Connected.\n";
    setDefaultBehavior(robot);

    if (udp_mode) {
      std::cout << "[PERTURBATION] MOSAIC mode: offsets on port " << kUdpOffsetPort << " (clamped to +-"
                << kUdpMaxOffsetM * 1000 << " mm), state to " << kUdpStateHost << ":" << kUdpStatePort << '\n';
    }
    std::cout << "[PERTURBATION] K_r=" << kRobotTranslationalStiffness << " N/m | steps +-" << kStepAmplitudeM * 1000
              << " mm, hold " << kStepHoldS << " s | rot_ trials: K_r=" << 0.5 * kRotationTrialStiffness
              << " Nm/rad, steps +-" << kRotationStepRad * 180.0 / kPi << " deg | logs in " << log_dir << '\n'
              << "WARNING: Franka will move to the initial pose. Do not hold the handle during this move. "
              << "Keep emergency stop accessible. Press Enter to continue..." << std::endl;
    std::cin.get();

    MotionGenerator motion_generator(kInitialMotionSpeedFactor, kInitialJointConfiguration);
    robot.control(motion_generator);
    std::cout << "\n[FRANKA] Initial pose reached.\n";

    franka::Model model = robot.loadModel();
    const Vector7d q_goal_eigen = Eigen::Map<const Vector7d>(kInitialJointConfiguration.data());

    std::array<double, 7> joint_thresholds{};
    joint_thresholds.fill(kCollisionThreshold);
    std::array<double, 6> cartesian_thresholds{};
    cartesian_thresholds.fill(kCollisionThreshold);
    robot.setCollisionBehavior(joint_thresholds, joint_thresholds, cartesian_thresholds, cartesian_thresholds);

    // Slave damping law: D = D_baseline * sqrt(K / K_baseline).
    const double d_translational =
        kBaselineTranslationalDamping * std::sqrt(kRobotTranslationalStiffness / kBaselineTranslationalStiffness);
    double k_rotational = kRobotRotationalStiffness;  // blended toward the active trial's value

    bool initialized = false;
    Eigen::Vector3d home_position;
    Eigen::Quaterniond home_orientation;
    int active_trial = 0;
    double trial_time = 0.0;
    int step_index = 0;
    Eigen::Vector3d udp_offset = Eigen::Vector3d::Zero();  // rate-limited toward the MOSAIC command

    control_running.store(true);
    robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
      const double dt = period.toSec();

      const std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
      const std::array<double, 7> coriolis_array = model.coriolis(state);
      Eigen::Map<const Matrix67d> jacobian(jacobian_array.data());
      Eigen::Map<const Vector7d> coriolis(coriolis_array.data());
      Eigen::Map<const Vector7d> q(state.q.data());
      Eigen::Map<const Vector7d> dq(state.dq.data());

      Eigen::Affine3d transform(Eigen::Matrix4d::Map(state.O_T_EE.data()));
      const Eigen::Vector3d position = transform.translation();
      Eigen::Quaterniond orientation(transform.rotation());
      orientation.normalize();
      const Vector6d cartesian_velocity = jacobian * dq;

      // The pose at the start of the control loop (the home pose) is the equilibrium for the whole run.
      if (!initialized) {
        home_position = position;
        home_orientation = orientation;
        initialized = true;
      }

      // Schedule: start a requested trial, then walk through its steps.
      if (active_trial == 0 && requested_trial.load() > finished_trial.load()) {
        active_trial = requested_trial.load();
        trial_time = 0.0;
        step_index = 0;
      }
      Eigen::Vector3d offset = Eigen::Vector3d::Zero();
      Eigen::Vector3d rotation_offset = Eigen::Vector3d::Zero();
      Phase phase = kIdle;
      if (active_trial != 0) {
        trial_time += dt;
        if (step_index + 1 < kMaxSteps && trial_time >= plan[step_index + 1].onset_s) {
          ++step_index;
        }
        const double profile = step_profile(trial_time, plan[step_index].onset_s, plan[0].axis >= 3, &phase);
        const int axis = plan[step_index].axis;
        if (axis < 3) {
          offset[axis] = plan[step_index].sign * kStepAmplitudeM * profile;
        } else {
          rotation_offset[axis - 3] = plan[step_index].sign * kRotationStepRad * profile;
        }
        if (trial_time >= trial_duration(plan)) {
          finished_trial.store(active_trial);
          active_trial = 0;
          offset.setZero();
          rotation_offset.setZero();
          phase = kIdle;
        }
      }
      if (udp_mode) {
        Eigen::Vector3d target = Eigen::Vector3d::Zero();
        const double command_time = udp_target_time.load();
        if (command_time >= 0.0 && elapsed_seconds(clock_start) - command_time < kUdpTimeoutS) {
          target = Eigen::Vector3d(udp_target[0].load(), udp_target[1].load(), udp_target[2].load());
        }
        for (int i = 0; i < 3; ++i) {
          udp_offset[i] += std::clamp(target[i] - udp_offset[i], -kUdpOffsetRateMps * dt, kUdpOffsetRateMps * dt);
        }
        offset = udp_offset;
      }
      const Eigen::Vector3d desired_position = home_position + offset;
      // Rotation about a base-frame axis through the end-effector point.
      Eigen::Quaterniond desired_orientation = home_orientation;
      if (rotation_offset.norm() > 0.0) {
        desired_orientation =
            Eigen::Quaterniond(Eigen::AngleAxisd(rotation_offset.norm(), rotation_offset.normalized())) *
            home_orientation;
      }
      const bool rotation_trial = active_trial != 0 && plan[0].axis >= 3;
      const double k_rotational_target = rotation_trial ? kRotationTrialStiffness : kRobotRotationalStiffness;
      const double blend_rate = (kRobotRotationalStiffness - kRotationTrialStiffness) / kRotationStiffnessBlendS;
      k_rotational += std::clamp(k_rotational_target - k_rotational, -blend_rate * dt, blend_rate * dt);
      const double d_rotational =
          kBaselineRotationalDamping * std::sqrt(k_rotational / kBaselineRotationalStiffness);

      // Orientation error as in the slave, toward the (possibly turned) equilibrium orientation.
      Eigen::Quaterniond orientation_for_error = orientation;
      if (desired_orientation.coeffs().dot(orientation_for_error.coeffs()) < 0.0) {
        orientation_for_error.coeffs() *= -1.0;
      }
      Eigen::Quaterniond error_quaternion = orientation_for_error.inverse() * desired_orientation;
      error_quaternion.normalize();
      const Eigen::Vector3d orientation_error =
          -transform.rotation() * Eigen::Vector3d(error_quaternion.x(), error_quaternion.y(), error_quaternion.z());

      // Safety checks: large tracking error, fast handle, joint velocity near the FR3 limits.
      int reason = kNone;
      if ((position - desired_position).norm() > kAbortPositionErrorM) {
        reason = kPositionError;
      } else if (cartesian_velocity.head<3>().norm() > kAbortSpeedMps) {
        reason = kSpeed;
      } else if (2.0 * std::asin(std::min(1.0, orientation_error.norm())) > kAbortOrientationErrorRad) {
        reason = kOrientationError;
      } else if (cartesian_velocity.tail<3>().norm() > kAbortAngularSpeedRadps) {
        reason = kAngularSpeed;
      } else {
        for (std::size_t i = 0; i < 7; ++i) {
          if (std::abs(dq[static_cast<int>(i)]) >
              kJointVelocityGuardStopRatio * kJointVelocityGuardLimitsRadps[i]) {
            reason = kJointVelocity;
          }
        }
      }

      Vector6d cartesian_wrench;
      cartesian_wrench.head<3>() = -kRobotTranslationalStiffness * (position - desired_position) -
                                   d_translational * cartesian_velocity.head<3>();
      cartesian_wrench.tail<3>() =
          -k_rotational * orientation_error - d_rotational * cartesian_velocity.tail<3>();

      const Vector7d task_torque = jacobian.transpose() * cartesian_wrench;

      Matrix6d jj_transpose = jacobian * jacobian.transpose();
      jj_transpose.diagonal().array() += kDampedInverseLambda * kDampedInverseLambda;
      const Matrix6d jj_inverse = jj_transpose.ldlt().solve(Matrix6d::Identity());
      const Matrix7d nullspace_projector = Matrix7d::Identity() - jacobian.transpose() * jj_inverse * jacobian;
      const Vector7d nullspace_torque =
          nullspace_projector * (kNullspaceStiffness * (q_goal_eigen - q) - kNullspaceDamping * dq);

      const Vector7d desired_joint_torque = task_torque + nullspace_torque + coriolis;
      const std::array<double, 7> saturated_torque = saturate_torque_rate(desired_joint_torque, state.tau_J_d);

      LogRecord record{};
      record.t = elapsed_seconds(clock_start);
      record.trial = active_trial;
      record.step = active_trial != 0 ? step_index : -1;
      record.phase = phase;
      record.axis = active_trial != 0 ? plan[step_index].axis : -1;
      record.sign = active_trial != 0 ? plan[step_index].sign : 0.0;
      for (int i = 0; i < 3; ++i) {
        record.desired[i] = desired_position[i];
        record.position[i] = position[i];
        record.velocity[i] = cartesian_velocity[i];
        record.desired_rotation[i] = rotation_offset[i];
      }
      record.quaternion[0] = orientation.w();
      record.quaternion[1] = orientation.x();
      record.quaternion[2] = orientation.y();
      record.quaternion[3] = orientation.z();
      for (int i = 0; i < 6; ++i) {
        record.wrench[i] = state.O_F_ext_hat_K[i];
      }
      log_ring->push(record);

      if (reason != kNone) {
        abort_reason.store(reason);
        g_stop_requested = 1;
      }
      if (g_stop_requested) {
        return franka::MotionFinished(franka::Torques(saturated_torque));
      }
      return franka::Torques(saturated_torque);
    });

  } catch (const franka::Exception& exception) {
    std::cerr << "\nFranka exception: " << exception.what() << '\n';
  } catch (const std::exception& exception) {
    std::cerr << "\nException: " << exception.what() << '\n';
  }

  g_stop_requested = 1;
  control_running.store(false);

  static const char* kAbortText[] = {"", "tracking error above limit", "handle speed above limit",
                                     "joint velocity near the FR3 limit", "orientation error above limit",
                                     "angular speed above limit"};
  if (abort_reason.load() != kNone) {
    std::cerr << "[SAFETY] Stopped: " << kAbortText[abort_reason.load()] << '\n';
  }

  console_thread.join();
  if (udp_thread.joinable()) {
    udp_thread.join();
  }
  writer_thread.join();
  if (state_socket >= 0) {
    close(state_socket);
  }
  if (log_ring->dropped.load() > 0) {
    std::cerr << "[LOG] " << log_ring->dropped.load() << " robot samples dropped (writer too slow)\n";
  }
  std::cout << "[LOG] " << log_dir << '\n';
  return 0;
}
