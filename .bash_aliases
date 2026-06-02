# .bash_aliases
alias visualize_pdc='ros2 launch pdc_visualization pdc_launch.py'
alias run_yolo='ros2 launch yolo_vision yolo_launch.py'
alias run_pilot_speed='ros2 launch vision_speed_guard vision_drive_all_launch.py default_mode:=/pilotnet/ackermann_cmd'
alias run_manual_speed='ros2 launch vision_speed_guard vision_drive_all_launch.py'
alias run_pilotnet='ros2 launch pilotnet pilotnet_launch.py'
alias run_parking='ros2 launch smart_parking smart_parking_launch.py'
