# Avoiding QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root' 
export XDG_RUNTIME_DIR=/run/user/$(id -u)
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

ros2 run rviz2 rviz2 -d $(ros2 pkg prefix bluhmbot)/share/bluhmbot/rviz/nav2_default_view.rviz 
