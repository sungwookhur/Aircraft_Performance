cmake_minimum_required(VERSION 3.10)

# 프로젝트 이름 설정
project(Proj2)

# C++ 표준 설정 (선택 사항이지만 권장)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 실행 파일 생성 (타겟 이름: app, 소스 파일: src/main.cpp)
add_executable(app src/main.cpp)