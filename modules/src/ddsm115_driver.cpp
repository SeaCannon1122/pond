#include <pond/pond.hpp>
#include "DDSM115CMD.h"

struct ddsm115_motor
{
  std::string name;
  double last_position;
  double command_velocity;
  double state_position;
  double state_velocity;
  double state_current;
  int id;
  int scalar;
  bool read;
};

class DDSM115Driver : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    std::vector<ddsm115_motor> wheels;

    DDSM115CMD cmd;
};

POND_MODULE_CPP_DECLARE(DDSM115Driver, "ddsm115_driver", "driver for the DDSM115 Motors")

pond_result DDSM115Driver::onStartup(const std::vector<void*>& args)
{
    auto device = parameter("device").asString().getStrict();
    if (!device) return POND_ERROR;

    if (cmd.connect(*device) == false)
    {
        POND_LOG(cmd.get_error());
        return POND_ERROR;
    }

    return POND_SUCCESS;
}

void DDSM115Driver::onShutdown()
{
    cmd.disconnect();
}

void DDSM115Driver::onFrame()
{
}
