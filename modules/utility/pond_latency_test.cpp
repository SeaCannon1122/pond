#include <pond/pond.hpp>

class Caller : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
private:
    pond::Distributor distributor;

    void do_test(uint32_t count)
    {
        double time = pond::get_time();

        for (uint32_t i = 0; i < count; i++) distributor.distribute((void*)NULL);

        POND_LOG("Distributed %7d times with average time %7.3f microseconds", count, (pond::get_time() - time) / (double)count * 1000000);
    }
};

POND_MODULE_CPP_DECLARE(Caller, "latency_test_caller", "info")

pond_result Caller::onStartup(const std::vector<void*>& args)
{
    distributor = createDistributor({"latency_test_channel"});

    do_test(10);
    do_test(100);
    do_test(1000);
    do_test(5000);
    do_test(10000);
    do_test(50000);
    do_test(100000);
    do_test(500000);
    do_test(1000000);

    distributor.destroy();

    shutdown();
    return POND_SUCCESS;
}

class Receiver : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:
    pond::Receiver receiver;
};

POND_MODULE_CPP_DECLARE(Receiver, "latency_test_receiver", "info")


pond_result Receiver::onStartup(const std::vector<void*>& args)
{
    receiver = createReceiver({"latency_test_channel"}, [](void** data) {});
    return POND_SUCCESS;
}

void Receiver::onShutdown()
{
    receiver.destroy();
}