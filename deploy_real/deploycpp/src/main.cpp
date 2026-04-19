#include <exception>
#include <cmath>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "FSMController.h"
#include "RobotIO.h"
#include "Controller.h"
#include "config/Config.h"
#include "launcher/Launcher.h"
#include "math/QuaternionUtils.h"
#include "policy/OnnxPolicyRunner.h"
#include "policy/TorchPolicyRunner.h"
#include "states/BeyondMimicState.h"

namespace {

std::string joinNames(const std::vector<std::string>& names) {
  std::string joined;
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (i > 0) {
      joined += ",";
    }
    joined += names[i];
  }
  return joined;
}

int checkConfig(const std::string& project_root) {
  const Config config = loadConfig(project_root);
  std::cout << "net=" << config.net << '\n';
  std::cout << "num_joints=" << config.num_joints << '\n';
  std::cout << "lowcmd_topic=" << config.lowcmd_topic << '\n';
  std::cout << "lowstate_topic=" << config.lowstate_topic << '\n';
  std::cout << "control_dt=" << config.control_dt << '\n';
  std::cout << "loco_model=" << config.loco_mode_model_path << '\n';
  std::cout << "beyond_mimic2_model=" << config.beyond_mimic2_model_path << '\n';
  return 0;
}

int checkOnnx(const std::string& project_root) {
  const Config config = loadConfig(project_root);
  std::cout << "model=" << config.beyond_mimic_model_path << '\n';
  OnnxPolicyRunner runner(config.beyond_mimic_model_path);
  std::vector<float> obs(154, 0.0f);
  const PolicyRunResult result = runner.run(obs, 0.0f);

  std::cout << "inputs=" << joinNames(runner.inputNames()) << '\n';
  std::cout << "outputs=" << joinNames(runner.outputNames()) << '\n';
  std::cout << "actions_size=" << result.actions.size() << '\n';
  std::cout << "joint_pos_size=" << result.joint_pos.size() << '\n';
  std::cout << "joint_vel_size=" << result.joint_vel.size() << '\n';
  std::cout << "body_quat_w_size=" << result.body_quat_w.size() << '\n';
  return 0;
}

int checkTorch(const std::string& project_root) {
  const Config config = loadConfig(project_root);
  std::cout << "model=" << config.loco_mode_model_path << '\n';

  TorchPolicyRunner runner(config.loco_mode_model_path);
  const std::vector<float> obs(96, 0.0f);
  const std::vector<float> action = runner.run(obs);
  std::cout << "actions_size=" << action.size() << '\n';
  return action.size() == 29 ? 0 : 1;
}

bool approx(float lhs, float rhs, float tolerance = 1e-5f) {
  return std::fabs(lhs - rhs) <= tolerance;
}

int checkMath() {
  const Quat identity{1.0f, 0.0f, 0.0f, 0.0f};
  const Quat yaw90 = eulerSingleAxisToQuat(static_cast<float>(M_PI) * 0.5f, 'z');
  const Quat identity_product = quatMul(identity, yaw90);
  const bool quat_identity_ok = approx(identity_product[0], yaw90[0]) &&
                                approx(identity_product[1], yaw90[1]) &&
                                approx(identity_product[2], yaw90[2]) &&
                                approx(identity_product[3], yaw90[3]);

  const Quat extracted_yaw = yawQuat(yaw90);
  const bool yaw_quat_ok = approx(extracted_yaw[0], yaw90[0]) &&
                           approx(extracted_yaw[1], yaw90[1]) &&
                           approx(extracted_yaw[2], yaw90[2]) &&
                           approx(extracted_yaw[3], yaw90[3]);

  const Mat3 identity_matrix = matrixFromQuat(identity);
  const bool matrix_identity_ok = approx(identity_matrix[0][0], 1.0f) &&
                                  approx(identity_matrix[1][1], 1.0f) &&
                                  approx(identity_matrix[2][2], 1.0f) &&
                                  approx(identity_matrix[0][1], 0.0f) &&
                                  approx(identity_matrix[0][2], 0.0f) &&
                                  approx(identity_matrix[1][0], 0.0f) &&
                                  approx(identity_matrix[1][2], 0.0f) &&
                                  approx(identity_matrix[2][0], 0.0f) &&
                                  approx(identity_matrix[2][1], 0.0f);

  std::cout << "quat_identity_ok=" << (quat_identity_ok ? 1 : 0) << '\n';
  std::cout << "yaw_quat_ok=" << (yaw_quat_ok ? 1 : 0) << '\n';
  std::cout << "matrix_identity_ok=" << (matrix_identity_ok ? 1 : 0) << '\n';
  return quat_identity_ok && yaw_quat_ok && matrix_identity_ok ? 0 : 1;
}

const char* stateName(FSMStateName state) {
  switch (state) {
    case FSMStateName::Passive:
      return "Passive";
    case FSMStateName::FixedPose:
      return "FixedPose";
    case FSMStateName::LocoMode:
      return "LocoMode";
    case FSMStateName::BeyondMimic:
      return "BeyondMimic";
    case FSMStateName::BeyondMimic2:
      return "BeyondMimic2";
  }
  return "Unknown";
}

int checkFsm(const std::string& project_root) {
  const Config config = loadConfig(project_root);
  const auto advanceToFixedPoseComplete = [](FSMController& fsm, StateAndCmd& state, PolicyOutput& output) {
    fsm.run(state, output);
    state.remote.start = true;
    fsm.run(state, output);
    state.remote.start = false;
    for (int i = fsm.fixedPoseState().currentStep(); i < fsm.fixedPoseState().totalSteps(); ++i) {
      fsm.run(state, output);
    }
  };

  FSMController initial_fsm(config);
  StateAndCmd initial_state_input;
  PolicyOutput initial_output;
  initial_fsm.run(initial_state_input, initial_output);
  const bool initial_state = initial_fsm.currentState() == FSMStateName::Passive;

  initial_state_input.remote.start = true;
  initial_fsm.run(initial_state_input, initial_output);
  initial_state_input.remote.start = false;
  const bool start_to_fixed_pose = initial_fsm.currentState() == FSMStateName::FixedPose;

  initial_state_input.remote.R1 = true;
  initial_state_input.remote.A = true;
  initial_fsm.run(initial_state_input, initial_output);
  const bool early_loco_blocked = initial_fsm.currentState() == FSMStateName::FixedPose;

  FSMController loco_fsm(config);
  StateAndCmd loco_state;
  PolicyOutput loco_output;
  advanceToFixedPoseComplete(loco_fsm, loco_state, loco_output);
  const bool fixed_pose_complete_after_steps =
      loco_fsm.fixedPoseState().complete() &&
      loco_fsm.fixedPoseState().currentStep() == loco_fsm.fixedPoseState().totalSteps();
  loco_state.remote.R1 = true;
  loco_state.remote.A = true;
  loco_fsm.run(loco_state, loco_output);
  const bool fixed_pose_to_loco = loco_fsm.currentState() == FSMStateName::LocoMode;
  loco_state.remote.A = false;
  loco_state.remote.B = true;
  loco_fsm.run(loco_state, loco_output);
  const bool loco_to_beyond_mimic2 = loco_fsm.currentState() == FSMStateName::BeyondMimic2;

  FSMController beyond_mimic2_fsm(config);
  StateAndCmd beyond_mimic2_state;
  PolicyOutput beyond_mimic2_output;
  advanceToFixedPoseComplete(beyond_mimic2_fsm, beyond_mimic2_state, beyond_mimic2_output);
  beyond_mimic2_state.remote.R1 = true;
  beyond_mimic2_state.remote.B = true;
  beyond_mimic2_fsm.run(beyond_mimic2_state, beyond_mimic2_output);
  const bool fixed_pose_to_beyond_mimic2 = beyond_mimic2_fsm.currentState() == FSMStateName::BeyondMimic2;
  beyond_mimic2_state.remote.R1 = false;
  beyond_mimic2_state.remote.B = false;
  beyond_mimic2_state.remote.F1 = true;
  beyond_mimic2_fsm.run(beyond_mimic2_state, beyond_mimic2_output);
  const bool beyond_mimic2_to_passive = beyond_mimic2_fsm.currentState() == FSMStateName::Passive;

  FSMController beyond_mimic_fsm(config);
  StateAndCmd beyond_mimic_state;
  PolicyOutput beyond_mimic_output;
  advanceToFixedPoseComplete(beyond_mimic_fsm, beyond_mimic_state, beyond_mimic_output);
  beyond_mimic_state.remote.L1 = true;
  beyond_mimic_state.remote.Y = true;
  beyond_mimic_fsm.run(beyond_mimic_state, beyond_mimic_output);
  const bool fixed_pose_to_beyond_mimic = beyond_mimic_fsm.currentState() == FSMStateName::BeyondMimic;
  beyond_mimic_state.remote.L1 = false;
  beyond_mimic_state.remote.Y = false;
  beyond_mimic_state.remote.R1 = true;
  beyond_mimic_state.remote.A = true;
  beyond_mimic_fsm.run(beyond_mimic_state, beyond_mimic_output);
  const bool beyond_mimic_to_loco = beyond_mimic_fsm.currentState() == FSMStateName::LocoMode;

  FSMController beyond_mimic_f1_fsm(config);
  StateAndCmd beyond_mimic_f1_state;
  PolicyOutput beyond_mimic_f1_output;
  advanceToFixedPoseComplete(beyond_mimic_f1_fsm, beyond_mimic_f1_state, beyond_mimic_f1_output);
  beyond_mimic_f1_state.remote.L1 = true;
  beyond_mimic_f1_state.remote.Y = true;
  beyond_mimic_f1_fsm.run(beyond_mimic_f1_state, beyond_mimic_f1_output);
  beyond_mimic_f1_state.remote.L1 = false;
  beyond_mimic_f1_state.remote.Y = false;
  beyond_mimic_f1_state.remote.F1 = true;
  beyond_mimic_f1_fsm.run(beyond_mimic_f1_state, beyond_mimic_f1_output);
  const bool beyond_mimic_to_passive = beyond_mimic_f1_fsm.currentState() == FSMStateName::Passive;

  FSMController beyond_mimic_start_fsm(config);
  StateAndCmd beyond_mimic_start_state;
  PolicyOutput beyond_mimic_start_output;
  advanceToFixedPoseComplete(beyond_mimic_start_fsm, beyond_mimic_start_state, beyond_mimic_start_output);
  beyond_mimic_start_state.remote.L1 = true;
  beyond_mimic_start_state.remote.Y = true;
  beyond_mimic_start_fsm.run(beyond_mimic_start_state, beyond_mimic_start_output);
  beyond_mimic_start_state.remote.L1 = false;
  beyond_mimic_start_state.remote.Y = false;
  beyond_mimic_start_state.remote.start = true;
  beyond_mimic_start_fsm.run(beyond_mimic_start_state, beyond_mimic_start_output);
  const bool beyond_mimic_to_fixed_pose = beyond_mimic_start_fsm.currentState() == FSMStateName::FixedPose;

  std::cout << "initial_state=" << stateName(FSMStateName::Passive) << '\n';
  std::cout << "start_to_fixed_pose=" << (start_to_fixed_pose ? 1 : 0) << '\n';
  std::cout << "early_loco_blocked=" << (early_loco_blocked ? 1 : 0) << '\n';
  std::cout << "fixed_pose_complete_after_steps=" << loco_fsm.fixedPoseState().totalSteps() << '\n';
  std::cout << "fixed_pose_to_loco=" << (fixed_pose_to_loco ? 1 : 0) << '\n';
  std::cout << "fixed_pose_to_beyond_mimic=" << (fixed_pose_to_beyond_mimic ? 1 : 0) << '\n';
  std::cout << "fixed_pose_to_beyond_mimic2=" << (fixed_pose_to_beyond_mimic2 ? 1 : 0) << '\n';
  std::cout << "beyond_mimic_to_loco=" << (beyond_mimic_to_loco ? 1 : 0) << '\n';
  std::cout << "beyond_mimic_to_fixed_pose=" << (beyond_mimic_to_fixed_pose ? 1 : 0) << '\n';
  std::cout << "beyond_mimic_to_passive=" << (beyond_mimic_to_passive ? 1 : 0) << '\n';
  std::cout << "loco_to_beyond_mimic2=" << (loco_to_beyond_mimic2 ? 1 : 0) << '\n';
  std::cout << "beyond_mimic2_to_passive=" << (beyond_mimic2_to_passive ? 1 : 0) << '\n';

  return initial_state && start_to_fixed_pose && early_loco_blocked && fixed_pose_complete_after_steps &&
                 fixed_pose_to_loco && fixed_pose_to_beyond_mimic && fixed_pose_to_beyond_mimic2 &&
                 beyond_mimic_to_loco && beyond_mimic_to_fixed_pose && beyond_mimic_to_passive &&
                 loco_to_beyond_mimic2 && beyond_mimic2_to_passive
             ? 0
             : 1;
}

int probeLowState(const std::string& project_root) {
  const Config config = loadConfig(project_root);
  RobotIO robot_io(config);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!robot_io.hasLowState() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (!robot_io.hasLowState()) {
    std::cout << "low_state_connected=0\n";
    return 1;
  }

  const StateAndCmd state = robot_io.latestStateAndCmd();
  std::cout << "low_state_connected=1\n";
  std::cout << "num_joints=" << config.num_joints << '\n';
  std::cout << "remote_decode_ok=1\n";
  std::cout << "q0=" << state.q[0]
            << " dq0=" << state.dq[0]
            << " quat=" << state.base_quat[0] << "," << state.base_quat[1] << "," << state.base_quat[2] << "," << state.base_quat[3]
            << " gyro=" << state.ang_vel[0] << "," << state.ang_vel[1] << "," << state.ang_vel[2]
            << " buttons="
            << "Start:" << (state.remote.start ? 1 : 0)
            << ",R1:" << (state.remote.R1 ? 1 : 0)
            << ",R2:" << (state.remote.R2 ? 1 : 0)
            << ",L2:" << (state.remote.L2 ? 1 : 0)
            << ",Y:" << (state.remote.Y ? 1 : 0)
            << ",F1:" << (state.remote.F1 ? 1 : 0)
            << ",B:" << (state.remote.B ? 1 : 0)
            << ",Select:" << (state.remote.select ? 1 : 0)
            << '\n';
  return 0;
}

void printJsonArray(const std::string& key, const std::vector<float>& values, bool trailing_comma) {
  std::cout << "  \"" << key << "\": [";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      std::cout << ", ";
    }
    std::cout << std::setprecision(9) << values[i];
  }
  std::cout << "]";
  if (trailing_comma) {
    std::cout << ",";
  }
  std::cout << "\n";
}

int dumpBeyondMimicAlignment(const std::string& project_root) {
  BeyondMimicState state(loadConfig(project_root));
  const BeyondMimicAlignmentDump dump = state.dumpAlignmentFixture();
  std::cout << "{\n";
  printJsonArray("qj_mj2lab", dump.qj_mj2lab, true);
  printJsonArray("robot_quat", dump.robot_quat, true);
  printJsonArray("motion_anchor_ori_b", dump.motion_anchor_ori_b, true);
  printJsonArray("mimic_obs", dump.mimic_obs, true);
  printJsonArray("target_dof_pos_mj", dump.target_dof_pos_mj, false);
  std::cout << "}\n";
  return 0;
}

void printUsage(const char* program) {
  std::cerr << "Usage:\n";
  std::cerr << "  " << program << " --check-config /home/abc/RoboMimic_Deploy\n";
  std::cerr << "  " << program << " --check-onnx /home/abc/RoboMimic_Deploy\n";
  std::cerr << "  " << program << " --check-torch /home/abc/RoboMimic_Deploy\n";
  std::cerr << "  " << program << " --check-math\n";
  std::cerr << "  " << program << " --check-fsm /home/abc/RoboMimic_Deploy\n";
  std::cerr << "  " << program << " --probe-lowstate /home/abc/RoboMimic_Deploy\n";
  std::cerr << "  " << program << " --run /home/abc/RoboMimic_Deploy\n";
  std::cerr << "  " << program << " --launcher /home/abc/RoboMimic_Deploy\n";
  std::cerr << "  " << program << " --dump-beyond-mimic-alignment /home/abc/RoboMimic_Deploy\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 1) {
      std::cout << "RoboMimic deploycpp bootstrap" << std::endl;
      std::cout << "argc=" << argc << std::endl;
      return 0;
    }

    const std::string mode = argv[1];
    if (mode == "--check-math") {
      if (argc != 2) {
        printUsage(argv[0]);
        return 2;
      }
      return checkMath();
    }

    if (argc != 3) {
      printUsage(argv[0]);
      return 2;
    }

    const std::string project_root = argv[2];
    if (mode == "--check-config") {
      return checkConfig(project_root);
    }
    if (mode == "--check-onnx") {
      return checkOnnx(project_root);
    }
    if (mode == "--check-torch") {
      return checkTorch(project_root);
    }
    if (mode == "--check-fsm") {
      return checkFsm(project_root);
    }
    if (mode == "--probe-lowstate") {
      return probeLowState(project_root);
    }
    if (mode == "--run") {
      Controller controller(loadConfig(project_root));
      return controller.run();
    }
    if (mode == "--launcher") {
      Launcher launcher(loadConfig(project_root), argv[0], project_root);
      return launcher.run();
    }
    if (mode == "--dump-beyond-mimic-alignment") {
      return dumpBeyondMimicAlignment(project_root);
    }

    printUsage(argv[0]);
    return 2;
  } catch (const std::exception& exc) {
    std::cerr << "error: " << exc.what() << '\n';
    return 1;
  }
}
