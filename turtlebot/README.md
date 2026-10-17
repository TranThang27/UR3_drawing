# TurtleBot 4 — SLAM và Nav2

Ubuntu 22.04, ROS 2 Humble. Chạy lệnh trong thư mục `turtlebot/`.
Dừng phiên cũ bằng `Ctrl+C` trước khi chuyển phần.

## Cài đặt

```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes \
  ros-humble-turtlebot4-navigation ros-humble-turtlebot4-viz \
  ros-humble-slam-toolbox ros-humble-nav2-bringup \
  ros-humble-nav2-map-server ros-humble-teleop-twist-keyboard ros-humble-rviz2 \
  python3-apt python3-pil python3-numpy python3-yaml python3-venv ffmpeg wmctrl \
  ros-humble-cv-bridge ros-humble-rmw-cyclonedds-cpp
./run.sh setup
./run.sh doctor
```

## Phần 1 — Simulation

Terminal 1:

```bash
./run.sh demo
```

Điều khiển: `i` tiến, `j/l` quay, `k` dừng. `Ctrl+C` đóng phiên.

Terminal 2 (dừng từng lệnh `hz` bằng `Ctrl+C`):

```bash
./run.sh check
./run.sh hz /scan --wall-time
./run.sh hz /odom --wall-time
```

## Phần 2 — Mapping

```bash
./run.sh mapping
```

Điều khiển khảo sát phòng; nhấn `Ctrl+C` để lưu `maps/map.yaml` và `maps/map.pgm`.
Hoặc quét tự động phòng mặc định:

```bash
./run.sh auto-map
```

## Phần 3 — Localization

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

RViz: chọn **2D Pose Estimate**, đặt vị trí/hướng theo Gazebo, kiểm tra LiDAR khớp map.

## Phần 4 — Navigation

```bash
./run.sh navigation --map maps/map.yaml
```

RViz: **2D Pose Estimate** → **Nav2 Goal**. Đặt 3 goal khác nhau, chờ từng goal
hoàn thành; không điều khiển thủ công.

Hoặc gửi 3 goal bằng lệnh trong terminal khác sau khi khởi tạo AMCL:

```bash
./run.sh goals --goal 1.8 -1.8 0 --goal -1.8 -1.8 0 --goal -1.8 2.0 0
```

## Quay video

Dùng desktop X11, mở khóa màn hình; chọn thư mục output chưa tồn tại.

```bash
./run.sh video-demo --output reports/video_demo_new
python3 scripts/finish_video_demo.py reports/video_demo_new
```

Video và kết quả lưu trong thư mục output.

## Nav2 bằng code + YOLO

Sau khi cài đặt, chạy:

```bash
./run.sh setup-yolo
./run.sh yolo-demo --output reports/nav_yolo_demo_new
python3 scripts/finish_yolo_demo.py reports/nav_yolo_demo_new
```

Dùng map cũ; tự localization, gửi 3 goal trong `config/goals.yaml`, chạy YOLO
và quay video. Output phải chưa tồn tại. Video: `turtlebot4_nav_yolo.mp4`.

Chạy riêng khi simulation/Nav2 đã sẵn sàng và AMCL đã khởi tạo:

```bash
./run.sh yolo --display
# Terminal khác:
./run.sh goals --goals-file config/goals.yaml --dwell 6 --timeout 360
```

Xem tọa độ và ảnh nhận diện:

```bash
source environment.sh
ros2 topic echo /clicked_point
ros2 topic echo /amcl_pose
ros2 topic echo /yolo/detections
```

RViz: **Publish Point** để lấy x/y; yaw tính bằng radian.
Ảnh đã vẽ bounding box: `/yolo/annotated_image`.

## Báo cáo

LaTeX nằm trong `report/` (`pdflatex report.tex`, cần `t5enc.def` kèm theo cho tiếng Việt); bản PDF: `report/report.pdf`.
Video demo: https://drive.google.com/drive/folders/1LF-GkSyY50TnkUzGGbq_RCvAMGTMZOS7?hl=vi
