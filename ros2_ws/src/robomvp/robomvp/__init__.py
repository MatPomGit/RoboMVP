# Pakiet Python dla węzłów RoboMVP v0.2.0
#
# Węzły percepcji:
#   camera_interface       – obrazy z kamer sprzętowych (V4L2/OpenCV)
#   marker_detection       – detekcja AprilTag tag36h11 i QR fallback
#   marker_pose_estimator  – estymacja pozy 3D metodą pinhole
#
# Węzły sterowania:
#   main_node              – automat stanowy + serwisy + Action Server
#   motion_sequences       – predefiniowane sekwencje waypoints
#   state_machine          – FSM z notify_sequence_done()
#   unitree_robot_api      – adapter do Unitree SDK 2 (LocoClient)
#   offset_corrector       – kalkulator korekcji offsetu
#
# Węzły infrastrukturalne:
#   robomvp_diagnostics    – DiagnosticArray (zdrowie systemu, 1 Hz)
#   robomvp_odometry       – nav_msgs/Odometry dead reckoning (50 Hz)
#   robomvp_tf_publisher   – statyczne TF base_link → kamery
#   robomvp_teleop         – ręczne sterowanie przez /cmd_vel
