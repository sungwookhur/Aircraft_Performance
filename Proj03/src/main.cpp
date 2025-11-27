#include <iostream>
#include <thread> // 외부 라이브러리(pthread 등) 의존

void thread_function() {
    std::cout << "Thread is running!" << std::endl;
}

int main() {
    std::thread t(thread_function);
    t.join();
    std::cout << "Main thread finished." << std::endl;
    return 0;
}
