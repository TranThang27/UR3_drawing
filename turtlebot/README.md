# TurtleBot 4: SLAM, AMCL và Nav2

Mô phỏng TurtleBot 4 Standard trong Gazebo Fortress, xây dựng bản đồ bằng
SLAM Toolbox, load lại bản đồ để localization bằng AMCL và navigation bằng Nav2.
Dự án dùng simulator, controller và navigation packages chính thức của TurtleBot 4.

Môi trường mặc định `simple_square` là phòng 6 × 6 m với hai hộp và một trụ tròn.
Có thể dùng `world:=depot` để chạy môi trường có sẵn; tuyến khảo sát tự động chỉ
hỗ trợ `simple_square`.

## Cài đặt

Cần Ubuntu 22.04, ROS 2 Humble và desktop có GPU hỗ trợ Gazebo/RViz2.
Chạy các lệnh bên dưới trong thư mục chứa `run.sh`.

```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes \
  ros-humble-turtlebot4-navigation ros-humble-turtlebot4-viz \
  ros-humble-slam-toolbox ros-humble-nav2-bringup \
  ros-humble-nav2-map-server ros-humble-teleop-twist-keyboard ros-humble-rviz2 \
  python3-apt python3-pil python3-numpy python3-yaml ffmpeg wmctrl
./run.sh setup
./run.sh doctor
```

`setup` tải và kiểm tra SHA256 của dependencies, giải nén vào `.runtime/`;
không thay đổi packages hệ thống. Overlay này chọn **Ogre2** cho sensor Create 3:
backend Ogre đã gây méo một đoạn tường LiDAR trong phép thử trên máy phát triển.
Cần kết nối mạng và apt sources của ROS Humble khi chạy setup.

`run.sh` tự source ROS và overlay. Khi chạy trực tiếp `ros2` trong terminal khác:

```bash
source ./environment.sh
```

Các terminal dùng cùng `ROS_DOMAIN_ID` (mặc định `42`) và `use_sim_time=true`.
Dừng phiên mô phỏng cũ trước khi chạy một phiên mới.

## Phần 1 — Simulation và kiểm tra sensor

Terminal 1:

```bash
./run.sh demo
```

Lệnh này mở Gazebo/RViz2, chờ sensor, undock robot rồi mở điều khiển bàn phím.
Nhấn `i` để tiến, `j`/`l` để quay, `k` để dừng; giữ phím khi di chuyển.
Nhấn `Ctrl+C` để đóng phiên. Dock được bỏ sau khi undock để tránh vật cản thấp
ngoài mặt phẳng quét LiDAR; thêm `--keep-dock` nếu cần giữ dock.

Terminal 2:

```bash
./run.sh check --report reports/part1_topics.json
./run.sh hz /scan --wall-time
./run.sh hz /odom --wall-time
```

Dừng từng lệnh `hz` bằng `Ctrl+C` trước khi chạy lệnh tiếp theo. Gazebo phải đang
Play. `check` kiểm tra dữ liệu scan và odometry; quan sát robot trong Gazebo khi
điều khiển. Có thể kiểm tra phản hồi vận tốc bằng `./run.sh motion` khi robot đang
đứng yên và vùng quanh robot trống.

RViz2 có `RobotModel`, `LaserScan` (`/scan`) và TF; frame của phần này là `odom`.
Nếu chỉ muốn simulator, dùng `./run.sh sim`; điều khiển riêng bằng
`./run.sh undock` rồi `./run.sh teleop`.

## Phần 2 — Mapping bằng SLAM Toolbox

```bash
./run.sh mapping
```

Điều khiển robot đi chậm quanh phòng, nhìn đủ các mặt vật cản và trở lại vùng
đã khảo sát để đóng vòng. RViz2 dùng frame `map`, hiển thị occupancy map, LiDAR
và đường khảo sát. Khi quét xong, nhấn `Ctrl+C` một lần để lưu bản đồ trước khi
đóng SLAM và simulator.

Hoặc khảo sát tự động phòng mặc định:

```bash
./run.sh auto-map
```

Tuyến 17 waypoint dùng pose Gazebo để điều khiển khảo sát môi trường đã biết;
SLAM vẫn nhận LiDAR, odometry và TF của robot. Đây là tuyến khảo sát thiết kế sẵn.
Phần navigation ở dưới dùng planner/controller của Nav2.

Bản đồ lưu trong `maps/map.yaml` và `maps/map.pgm`. Lệnh tự động xuất thêm
`maps/map_grid.png`; các bản cũ được giữ trong `maps/backups/`.
Nếu đang chạy SLAM riêng, lưu thủ công bằng:

```bash
./run.sh save maps/map
python3 scripts/export_grid_map.py maps/map.yaml
```

Kiểm tra tường liền, vật cản rõ, không nhân đôi tường trên RViz2 trước khi sử
dụng bản đồ. [Bản đồ kèm repository](maps/map_grid.png) được tạo từ lần khảo sát
đã hoàn thành trong `simple_square`.

## Phần 3 — Load map và localization

Dừng SLAM và phiên mapping bằng `Ctrl+C`. Khởi động lại cùng môi trường:

Terminal 1:

```bash
./run.sh sim rviz_config:="$PWD/config/localization.rviz"
```

Terminal 2:

```bash
./run.sh undock
./run.sh remove-dock
./run.sh localize maps/map.yaml
```

Trên RViz2, chọn **2D Pose Estimate**, đặt vị trí và hướng theo robot trong
Gazebo. Sau undock ở phòng mặc định, pose ban đầu xấp xỉ `(-0.35, 0, pi)`.
Quan sát hạt AMCL hội tụ và điểm LiDAR khớp tường trên map.

Có thể kiểm chứng AMCL với ground truth trong simulator bằng:

```bash
source ./environment.sh
python3 scripts/check_localization.py --map maps/map.yaml \
  --initial-pose -0.35 0 3.14159 --report reports/localization_check.json
```

Script này đặt initial pose, kiểm tra rồi di chuyển robot chậm qua các điểm
kiểm chứng. Chạy trước khi đặt navigation goals.

## Phần 4 — Autonomous navigation với Nav2

Dừng các terminal của phần 3 rồi chạy phiên mới dùng map đã lưu:

```bash
./run.sh navigation --map maps/map.yaml
```

Đặt **2D Pose Estimate**, kiểm tra LiDAR khớp map, sau đó dùng **Nav2 Goal** để
chọn lần lượt ba vị trí khác nhau, ví dụ `(1.8, -1.8)`, `(-1.8, -1.8)` và
`(-1.8, 2.0)` trong phòng mặc định. Kéo chuột để đặt hướng cuối của robot.
Chờ mỗi goal hoàn thành trước khi gửi goal tiếp theo; không teleop trong lúc
Nav2 đang chạy. RViz2 hiển thị AMCL, costmap/inflation và đường đi.

Để gửi và kiểm chứng ba goal qua ROS 2 action thay vì thao tác chuột:

```bash
./run.sh goals --goal 1.8 -1.8 0 --goal -1.8 -1.8 0 --goal -1.8 2.0 0 \
  --report reports/navigation_goals.json
```

AMCL phải được khởi tạo trước. Report ghi trạng thái goal, sai số vị trí,
đường planner và bumper. Sau khi gửi goal, robot di chuyển bằng Nav2.

## Quay video demo

Cần desktop **X11**, màn hình mở khóa, `ffmpeg` và `wmctrl`.
Video hiển thị các cửa sổ Gazebo/RViz2 thực tế.

```bash
./run.sh video-demo --output reports/video_demo_new
python3 scripts/finish_video_demo.py reports/video_demo_new
```

Runner khảo sát, lưu map, dừng SLAM, khởi động lại simulator, kiểm chứng AMCL
rồi đặt ba goal qua công cụ RViz2. Giữ nguyên bố cục cửa sổ trong lúc quay;
GUI automation phụ thuộc bố cục RViz và phiên X11. Thư mục output phải chưa tồn tại.

Output gồm `turtlebot4_demo.mp4`, bản rút gọn `turtlebot4_demo_short.mp4`,
`demo_results.json`, report localization/navigation và map của lần quay đó.
Bản rút gọn có ghi nhãn tốc độ ở những đoạn được tua nhanh.
Các video và log trong `reports/` được giữ cục bộ, không commit lên Git.

## Cấu trúc và chẩn đoán

- `worlds/`: thế giới SDF dùng trong demo.
- `launch/`: ghép simulator và navigation packages có sẵn.
- `config/`: RViz2, SLAM Toolbox và DDS cho recorder.
- `scripts/`: chạy demo, kiểm tra ROS, lưu map, xuất ảnh và quay video.
- `maps/`: bản đồ mặc định dùng cho localization/navigation.
- `.runtime/`, `reports/`: dependencies và kết quả phát sinh, được Git bỏ qua.

`./run.sh doctor` kiểm tra packages. Nếu `hz` không có dữ liệu, kiểm tra Gazebo
Play và domain giữa các terminal. Để kiểm tra nhận một mẫu scan với QoS sensor,
dùng `ros2 topic echo /scan --once --qos-reliability best_effort` sau khi source
`environment.sh`. Nếu RViz báo thiếu transform,
kiểm tra frame `odom` khi simulation hoặc `map` khi mapping/localization.
Nếu robot không nhận vận tốc, chạy undock và kiểm tra không có phiên teleop/Nav2
khác đang gửi lệnh cùng lúc.
