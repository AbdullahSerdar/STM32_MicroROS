Burada, Stm ile Gazebo arasındaki bağlantıyı sağlayan script yapısı var.

husky_ws klasörü ise tüm yapının colcon build edilmiş hali olarak var. Gerekli kurulumlar sağlandıktan sonra çalıştırılabilir

Bunun için :
cd husky_ws
ros2 launch husky_gazebo husky_playpen.launch.py
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
cd ros2_ws/src/topic_relay
python3 topic_relay.py
python3 camera_bottom.py  , ifadeleri sırayla çalıştırılabilir 






