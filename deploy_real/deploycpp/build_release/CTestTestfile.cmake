# CMake generated Testfile for 
# Source directory: /home/abc/RoboMimic_Deploy/deploy_real/deploycpp
# Build directory: /home/abc/RoboMimic_Deploy/deploy_real/deploycpp/build_release
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(command_utils_test "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/build_release/command_utils_test")
set_tests_properties(command_utils_test PROPERTIES  _BACKTRACE_TRIPLES "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;102;add_test;/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;0;")
add_test(config_surface_test "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/build_release/config_surface_test")
set_tests_properties(config_surface_test PROPERTIES  _BACKTRACE_TRIPLES "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;121;add_test;/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;0;")
add_test(torch_policy_runner_smoke_test "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/build_release/torch_policy_runner_smoke_test")
set_tests_properties(torch_policy_runner_smoke_test PROPERTIES  _BACKTRACE_TRIPLES "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;143;add_test;/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;0;")
add_test(loco_mode_state_test "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/build_release/loco_mode_state_test")
set_tests_properties(loco_mode_state_test PROPERTIES  _BACKTRACE_TRIPLES "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;176;add_test;/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;0;")
add_test(beyond_mimic2_state_test "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/build_release/beyond_mimic2_state_test")
set_tests_properties(beyond_mimic2_state_test PROPERTIES  _BACKTRACE_TRIPLES "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;200;add_test;/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;0;")
add_test(beyond_mimic_state_test "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/build_release/beyond_mimic_state_test")
set_tests_properties(beyond_mimic_state_test PROPERTIES  _BACKTRACE_TRIPLES "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;224;add_test;/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;0;")
add_test(controller_safety_test "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/build_release/controller_safety_test")
set_tests_properties(controller_safety_test PROPERTIES  _BACKTRACE_TRIPLES "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;234;add_test;/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;0;")
add_test(remote_control_test "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/build_release/remote_control_test")
set_tests_properties(remote_control_test PROPERTIES  _BACKTRACE_TRIPLES "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;245;add_test;/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;0;")
add_test(launcher_state_machine_test "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/build_release/launcher_state_machine_test")
set_tests_properties(launcher_state_machine_test PROPERTIES  _BACKTRACE_TRIPLES "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;256;add_test;/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;0;")
add_test(fsm_controller_test "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/build_release/fsm_controller_test")
set_tests_properties(fsm_controller_test PROPERTIES  _BACKTRACE_TRIPLES "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;288;add_test;/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;0;")
add_test(check_config_cli_test "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/build_release/robomimic_deploycpp" "--check-config" "/home/abc/RoboMimic_Deploy")
set_tests_properties(check_config_cli_test PROPERTIES  PASS_REGULAR_EXPRESSION "net=.*;loco_model=.*;beyond_mimic2_model=.*" _BACKTRACE_TRIPLES "/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;290;add_test;/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/CMakeLists.txt;0;")
