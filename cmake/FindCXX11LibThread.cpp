#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

class ThreadGuard
{
    public :

    ThreadGuard () noexcept {}

    ThreadGuard (const ThreadGuard &) = delete;

    ThreadGuard &operator= (const ThreadGuard &) = delete;

    ThreadGuard (std::thread &&thr) noexcept :
        thread_(std::move(thr)) {}

    ThreadGuard (ThreadGuard &&other) noexcept :
        thread_(std::move(other.thread_)) {}

    ThreadGuard &operator= (ThreadGuard &&other) noexcept
    {thread_ = std::move(other.thread_); return *this;}

    ~ThreadGuard () noexcept {if (thread_.joinable()) thread_.join();}

    private :

    std::thread thread_;
}; // class ThreadGuard

void work ()
{
    std::chrono::milliseconds dura(100);
    std::this_thread::sleep_for(dura);
    std::cout << "Hello World" << std::endl;
}

int main ()
{
    std::vector<ThreadGuard> tg;
    tg.reserve(4);
    {
        for (std::size_t i = 0; i != 4; ++i)
            tg.push_back(ThreadGuard(std::thread(work)));
    }
}
