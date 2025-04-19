cmake_minimum_required(VERSION 3.10)
project(listener)

find_package(gz-transport12 REQUIRED)
find_package(gz-msgs9 REQUIRED)
find_package(gz-math7 REQUIRED)

add_executable(listener listener.cc)
target_link_libraries(listener
  gz-transport12::gz-transport12
  gz-msgs9::gz-msgs9
  gz-math7::gz-math7
)
find_package(Protobuf REQUIRED)

add_executable(thrust thrust.cc)
target_link_libraries(thrust gz-transport12::gz-transport12)