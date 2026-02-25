cmake_minimum_required(VERSION 3.5)
project(lrmate_200id_model_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY lrmate_200id
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
