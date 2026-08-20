#include <pond/manager/manager.hpp>

#include <chrono>
#include <signal.h>

std::atomic<bool> is_running{true};
int32_t int_counter = 0;

void signalHandler(int signal)
{
    if (signal != SIGINT && signal != SIGTERM)return;
    printf(" INTERRUPT\n"); fflush(stdout);
    is_running.store(false);

    int_counter++;
    if (int_counter == 10) exit(187);
}

int main()
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    {
        PondManager pm(false, false);

        
        
        while (is_running.load()) std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}