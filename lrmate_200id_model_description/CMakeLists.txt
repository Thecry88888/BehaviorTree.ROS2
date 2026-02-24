cmake_minimum_required(VERSION 3.5)
project(lrmate_200id_model_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY lrmate_200id/meshes lrmate_200id/srdf lrmate_200id/urdf lrmate_200id/rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
