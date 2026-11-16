#pragma once
// Test stub of libfranka (not the real library) for perturbation.cpp: a 3 kg handle at (0.3, 0, 0.5) held by the
// program's torques 1-3 (the stub Jacobian makes them the Cartesian force) and by a hand spring K_h with 20 Ns/m.
// K_h (9 numbers, N/m, row by row) is read every 50 ms from the file named by FRANKA_STUB_HAND (none: no hand).
// O_F_ext_hat_K = K_h dx (force of the robot on the hand). Runs at 1 kHz wall-clock time until the program ends the
// control loop.
#include <array>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <string>
#include <thread>
#include "control_types.h"
#include "duration.h"
#include "model.h"
#include "robot_state.h"
namespace franka {
struct Robot {
  explicit Robot(const std::string&) {}
  void setCollisionBehavior(const std::array<double,7>&, const std::array<double,7>&, const std::array<double,6>&, const std::array<double,6>&) {}
  void setCollisionBehavior(const std::array<double,7>&, const std::array<double,7>&, const std::array<double,7>&, const std::array<double,7>&, const std::array<double,6>&, const std::array<double,6>&, const std::array<double,6>&, const std::array<double,6>&) {}
  void setJointImpedance(const std::array<double,7>&) {}
  void setCartesianImpedance(const std::array<double,6>&) {}
  void control(std::function<Torques(const RobotState&, Duration)> cb) {
    const double home[3] = {0.3, 0.0, 0.5}, mass = 3.0, hand_damping = 20.0, dt = 0.001;
    const std::array<double, 7> q_home = {0.0, -0.785398163397448, 0.0, -2.356194490192345, 0.0, 1.570796326794897,
                                          0.785398163397448};
    const char* hand_file = std::getenv("FRANKA_STUB_HAND");
    double k[9] = {0}, x[3] = {0}, v[3] = {0};
    RobotState st{};
    st.q = q_home;
    st.q_d = q_home;
    auto next = std::chrono::steady_clock::now();
    for (long i = 0;; ++i) {
      if (hand_file && i % 50 == 0) {
        std::ifstream f(hand_file);
        double kn[9];
        bool ok = true;
        for (double& value : kn) ok = ok && static_cast<bool>(f >> value);
        if (ok) for (int n = 0; n < 9; ++n) k[n] = kn[n];
      }
      st.O_T_EE = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, home[0] + x[0], home[1] + x[1], home[2] + x[2], 1};
      for (int r = 0; r < 3; ++r) {
        st.dq[r] = v[r];
        st.O_F_ext_hat_K[r] = k[3 * r] * x[0] + k[3 * r + 1] * x[1] + k[3 * r + 2] * x[2];
      }
      const Torques out = cb(st, Duration{});
      st.tau_J_d = out.tau_J;
      if (out.motion_finished) return;
      for (int r = 0; r < 3; ++r) v[r] += dt * (out.tau_J[r] - st.O_F_ext_hat_K[r] - hand_damping * v[r]) / mass;
      for (int r = 0; r < 3; ++r) x[r] += dt * v[r];
      next += std::chrono::microseconds(1000);
      std::this_thread::sleep_until(next);
    }
  }
  template <typename G> void control(G&) {}
  Model loadModel() { return {}; }
};
}  // namespace franka
