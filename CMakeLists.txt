project(pid)
set(CMAKE_CXX_STANDARD 17)

add_library(pid
  src/PID.cpp
)
target_include_directories(pid PUBLIC "src/")
