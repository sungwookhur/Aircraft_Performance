cmake_minimum_required(VERSION 3.10)

project(Proj03)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 외부 패키지(라이브러리) 찾기
# REQUIRED: 찾지 못하면 빌드를 중단하고 에러를 발생시킴
find_package(Threads REQUIRED)

# 2. 실행 파일 타겟 정의
add_executable(thread_app src/main.cpp)

# 3. 타겟에 라이브러리 링크
# PRIVATE: 이 라이브러리는 my_thread_app 내부에서만 필요함
# Threads::Threads는 CMake가 제공하는 'Modern CMake' 방식의 타겟 이름입니다.
target_link_libraries(thread_app PRIVATE Threads::Threads)