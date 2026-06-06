cmake_minimum_required(VERSION 3.5)
project(asterassembly_description)

find_package(ament_cmake REQUIRED)

# Installer tous les fichiers utiles
install(DIRECTORY launch urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

