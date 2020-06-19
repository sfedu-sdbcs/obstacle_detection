# obstacle_detection
Пакет *occupancy_ugv_map*
* Для запуска ноды, необходимо поместить папку *occupancy_ugv_map* в ранее созданную catkin директорию, а именно в `{catkin_dir}/src/`
* Затем сбилдить его командой `catkin_make --pkg occupancy_ugv_map`
* После билда при желании можно поменять параметры в файле `occupancy_ugv_map/launch/occupancy_ugv.launch`
  * `~cloud_in` - название топика откуда будет браться облако точек `sensor_msgs/PointCloud2`
  * `~output` - название топика куда будет публиковаться готовая карта `nav_msgs/OccupancyGrid`
  * `~odom` - название топика откуда будет браться одометрия `nav_msgs/Odometry`
  * `~width` - ширина карты в точках
  * `~height` - высотка карты в точках
  * `~resolution`- разрешение карты. Сколько метров реального мира приходится на одну точку итоговой карты. 0,2 = 20 см на 1 точку
* После чего можно запустить ноду командой `roslaunch --screen occupancy_ugv_map occupancy_ugv.launch`

Тестовая сцена находится в [этом](https://github.com/sfedu-sdbcs/simulation) репозитории, а именно `coppeliasim/scenes/user-scenes/nkbvs.ttt`
