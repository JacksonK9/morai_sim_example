# morai_sim_example
- Morai Simulator 및 ROS 기반 예제 코드

## How to Use
- Simulator 연결을 위한 Rosbridge 및 제어를 위한 상위 제어기 실행
```bash
$ roslaunch morai_sim_example sim.launch
```
- Ego_topic 기반 경로 저장 (<path_name> 부분을 저장하려는 이름으로 변경하고 실행 시 저장 시작되며, 수동 조작하여 이동한 후, Ctrl + C로 종료하면 저장)
```bash
$ rosrun morai_sim_example path_saver.py path <path_name>
```
- 저장된 경로 불러오기 및 Ego_topic 기반으로 PurePursuit를 통한 경로 추종
```bash
$ roslaunch morai_sim_example gps_follower.launch
```
- Ego_topic 기반의 Odometry 및 Gmapping을 이용한 Map 생성
```bash
$ roslaunch morai_sim_example slam_gmapping.launch
```
- 지도 생성 완료 후 저장 (<map_name> 부분 지정)
```bash
$ roscd morai_sim_example
$ rosrun map_server map_saver -f <map_name>
```
- 저장된 지도를 이용하여 BFS를 이용한 경로 생성
```bash
$ roslaunch morai_sim_example grid_searching.launch
```

## Future Works
- GridMap을 CostMap으로 업데이트하여 경로 생성 (좁은 구역을 가지 않도록 변경)
- 생성된 경로를 Smoothing하여 차량으로 갈 수 있는 경로로 생성
- 경로 생성 시작 시, 차량의 헤딩각 고려하기