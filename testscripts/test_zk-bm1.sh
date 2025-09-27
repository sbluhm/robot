ros2 control list_controllers

echo "Cutter Motor Forward"
ros2 topic pub --times 5 /cutter_effort_controller/commands std_msgs/msg/Float64MultiArray "data:
- 100"
read
echo "Cutter Motor Stop"
ros2 topic pub --times 5 /cutter_effort_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0"
read
echo "Cutter Motor Backwards"
ros2 topic pub --times 5 /cutter_effort_controller/commands std_msgs/msg/Float64MultiArray "data:
- -100"
read
echo "Cutter Motor Stop"
ros2 topic pub --times 5 /cutter_effort_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0"
read
echo "Brush Motor Forward"
ros2 topic pub --times 5 /brush_effort_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.1"
read
echo "Brush Motor Stop"
ros2 topic pub --times 5 /brush_effort_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0"
read
echo "Brush Motor Backwards"
ros2 topic pub --times 5 /brush_effort_controller/commands std_msgs/msg/Float64MultiArray "data:
- -0.1"
read
echo "Brush Motor Stop"
ros2 topic pub --times 5 /brush_effort_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0"
