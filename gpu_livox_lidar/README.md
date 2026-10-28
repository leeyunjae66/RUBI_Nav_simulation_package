# gpu_livox_lidar

Livox MID-360용 **GPU 렌더링** LiDAR 플러그인 (Gazebo Classic 11).

기존 `ros2_livox_simulation`(`libros2_livox.so`)은 Livox 광선을 하나하나 ODE로 CPU에서 계산해서,
MID-360(스캔당 40,000점)을 켜면 실시간 비율이 약 0.25로 떨어졌습니다. 이 플러그인은 `gpu_ray` 센서로
시야 전체를 0.2° 간격 거리 영상으로 렌더링한 뒤, 같은 Livox 스캔 패턴 CSV의 방향마다 샘플링합니다.
MID-360의 비반복 스캔 패턴은 그대로 유지됩니다.

## 출력 (ros2_livox와 호환)

- `<topic>`: `livox_ros_driver2/CustomMsg`
- `<topic>_PointCloud2`: `sensor_msgs/PointCloud2` (x, y, z)
- 좌표계: 센서 이름(또는 `<frame_id>`), 타임스탬프는 Gazebo 시뮬레이션 시간
- 범위 밖 반환은 ros2_livox처럼 원점(0, 0, 0) 점으로 내보냅니다. `<drop_invalid>true</drop_invalid>`면 버립니다.

## 사용

`gpu_ray` 센서에 붙입니다. 예시는 `rubi_gazebo_sim/models/RUBI_mid360_imu_gpu/model.sdf`.

```xml
<sensor name='livox_frame_raw' type='gpu_ray'>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal><samples>1800</samples><min_angle>-3.1415927</min_angle><max_angle>3.1381020</max_angle></horizontal>
      <vertical><samples>300</samples><min_angle>-0.1274090</min_angle><max_angle>0.9128072</max_angle></vertical>
    </scan>
    <range><min>0.1</min><max>200.0</max></range>
  </ray>
  <plugin name='livox_frame_raw_plugin' filename='libgpu_livox_plugin.so'>
    <csv_file_name>/tmp/rubi_mid360_imu_mid360.csv</csv_file_name>   <!-- ros2_livox_simulation/scan_mode/mid360.csv -->
    <samples>40000</samples>
    <downsample>1</downsample>
    <topic>/livox/lidar_raw</topic>
  </plugin>
</sensor>
```

- 수평은 360°(마지막 칸이 -180°와 겹치지 않게 `max = π - 2π/samples`), 수직은 스캔 패턴 범위(MID-360: -7.2°~52.2°)를 덮어야 합니다.
  덮지 못하면 시작할 때 경고를 냅니다.
- `GAZEBO_PLUGIN_PATH`에 이 패키지의 `lib`가 있어야 합니다(`rubi_gazebo.launch.py`가 추가).
- GPU 센서라서 X 디스플레이와 GPU가 필요합니다.

## 검증 결과

`my_map.world`에 고정 장치를 두고 CPU·GPU 플러그인을 같은 위치에 달아 첫 스캔의 같은 방향 40,000개를 비교했습니다.

| | CPU (`libros2_livox.so`) | GPU (이 플러그인) |
|---|---|---|
| 점과 실제 표면 사이 거리 (중앙값 / 99%) | 8.9 cm / 10.0 cm | **0.22 cm / 2.1 cm** |
| RUBI + `my_map.world` 실시간 비율 | 0.24 | **0.99** |
| 라이다 / IMU 발행 속도 (설정 10 / 200 Hz) | 약 2.5 / 49 Hz | **9.8 / 198 Hz** |

CPU 플러그인은 최소 거리(0.1 m)에서 시작하는 광선의 거리를 센서 원점 기준인 것처럼 써서 점이 0.1 m 짧게 찍힙니다.
CPU 거리에 0.1 m를 더하면 GPU와 중앙값 0.26 cm 차이로 일치합니다. 남는 차이는 물체 경계에서 격자 한 칸(0.2°) 차이로 생깁니다.
