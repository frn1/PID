project(pid)
set(CMAKE_CXX_STANDARD 17)

add_library(pid
  src/pid.cpp
)
target_include_directories(pid PUBLIC "src/")