cmake_minimum_required(VERSION 3.5)
project(ur_drawing)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/draw_p.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/circle_geometry.py
  scripts/drawing_node.py
  scripts/robot_motion.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
