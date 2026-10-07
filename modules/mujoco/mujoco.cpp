#include <pond/pond.hpp>
#include <pond_data_types/motor_types.hpp>

#include <mujoco/mujoco.h>
#include <simulate/glfw_adapter.h>
#include <simulate/simulate.h>
#include <simulate/array_safety.h>

namespace mj  = ::mujoco;
namespace mju = ::mujoco::sample_util;

struct actuator
{
  int aid, jid, ctrl_adr;
  double cmd_pos = 0, cmd_vel = 0, fb_pos = 0, fb_vel = 0, fb_torque = 0;
  bool cmd_is_pos = false, cmd_is_vel = false;
};

class Mujoco : public pond::ModuleBase
{
public:
  virtual pond_result onStartup(const std::vector<void*>& args) override;
  virtual void onShutdown() override;
  virtual void onFrame() override;
private:
  std::thread gui_thread;
  mjvCamera cam; mjvOption opt; mjvPerturb pert;

  mjModel* m = nullptr;
  mjData*  d = nullptr;

  double syncCPU = 0.0;
  mjtNum syncSim = 0;
  int last_run = -1;

  std::unique_ptr<mj::Simulate> sim;

  std::atomic<bool> sim_created{false};
  std::string robot_path;
  std::optional<std::string> plugin_dir;

  std::vector<pond::Receiver> command_receiver, feedback_receiver;
  
  std::vector<std::vector<actuator>> actuators_groups;
  std::mutex interface_mutex;
  double last_update_time = 0;

  const double syncMisalign       = 0.1;  // maximum mis-alignment before re-sync (simulation seconds)
  const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation

  const char* Diverged(int disableflags, const mjData* d)
  {
    if (disableflags & mjDSBL_AUTORESET)
      for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS})
        if (d->warning[w].number > 0)
          return mju_warningText(w, d->warning[w].lastinfo);
    
    return nullptr;
  }

  void sim_thread_func()
  {
    if (int nplugin = mjp_pluginCount())
    {
      POND_LOG("Built-in plugins:\n");
      for (int i = 0; i < nplugin; ++i) POND_LOG("    %s\n", mjp_getPluginAtSlot(i)->name);
    }

    if (plugin_dir) mj_loadAllPluginLibraries(
      plugin_dir->c_str(),
      +[](const char* filename, int first, int count)
      {
        printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) printf("    %s\n", mjp_getPluginAtSlot(i)->name);
      }
    );

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultPerturb(&pert);

    sim = std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), &cam, &opt, &pert, false);

    sim_created.store(true);
    sim->RenderLoop();
  }
};

POND_MODULE_CPP_DECLARE(Mujoco, "mujoco", "mujoco pond binding")
POND_BUNDLE_DECLARE("mojoco bundle", POND_MODULE(Mujoco))

pond_result Mujoco::onStartup(const std::vector<void*>& args)
{
  POND_LOG("MuJoCo version %s", mj_versionString());
  if (mjVERSION_HEADER != mj_version()) POND_LOG_RETURN_ERROR("Headers and library have different versions");

  plugin_dir = parameter("plugin_directory").asString().getStrict({}, false);
  if (auto robot_path_o = parameter("robot_path").asString().getStrict()) robot_path = *robot_path_o; else return POND_ERROR;
  
  const int kErrorLength = 1024; char loadError[kErrorLength] = "";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mjSpec* spec = mj_parse(robot_path.c_str(), nullptr, &vfs, loadError, kErrorLength);

  if (!spec) POND_LOG("Error: Could not parse model: %s", loadError);
  else
  {
    if (!(m = mj_compile(spec, &vfs))) POND_LOG(mjs_getError(spec));
    mj_deleteSpec(spec);
  }
  mj_deleteVFS(&vfs);

  if (!spec || !m) return POND_ERROR;
  if (!(d = mj_makeData(m))) { mj_deleteModel(m); POND_LOG_RETURN_ERROR("Failed to create model data"); }

  mj_forward(m, d);

  auto space = parameterSpace("actuator_groups");
  actuators_groups.resize(space.listLength("names"));
  command_receiver.resize(actuators_groups.size());
  feedback_receiver.resize(actuators_groups.size());

  for (uint32_t gi = 0; gi < actuators_groups.size(); gi++)
  {
    auto names = space.parameterAtIndex(gi, "names").asStringArray().get({});
    auto types = space.parameterAtIndex(gi, "types").asStringArray().get({}, names.size());
    actuators_groups[gi].resize(names.size());

    pond::ChannelsInfo cmd_info, fb_info;
    for (uint32_t i = 0; i < names.size(); i++)
    {
      cmd_info.channel<MotorCommand>(names[i] + "/command");
      fb_info.channel<MotorFeedback>(names[i] + "/get_feedback");

      if ((actuators_groups[gi][i].aid = mj_name2id(m, mjOBJ_ACTUATOR, names[i].c_str())) == -1)
      {
        mj_deleteModel(m);
        POND_LOG_RETURN_ERROR("Did not find actuator %s", names[i].c_str());
      }

      actuators_groups[gi][i].jid = m->actuator_trnid[2 * actuators_groups[gi][i].aid];

      int ctrl_num = m->actuator_ctrlnum[actuators_groups[gi][i].aid];
      if (ctrl_num != 1)
      {
        mj_deleteModel(m);
        POND_LOG_RETURN_ERROR("actuator %s has %d control interfaces", names[i].c_str(), ctrl_num);
      }

      actuators_groups[gi][i].cmd_is_pos = (types[i] == "position");
      actuators_groups[gi][i].cmd_is_vel = (types[i] == "velocity");
      actuators_groups[gi][i].ctrl_adr = m->actuator_ctrladr[actuators_groups[gi][i].aid];

      if (!actuators_groups[gi][i].cmd_is_pos && !actuators_groups[gi][i].cmd_is_vel)
      {
        mj_deleteModel(m);
        POND_LOG_RETURN_ERROR("Actuator type '%s' for actuator %s not implemented", types[i].c_str(), names[i].c_str());
      }
    } 

    feedback_receiver[gi] = createReceiver<MotorFeedback>(fb_info, [this, gi](MotorFeedback** feedbacks) {

      std::lock_guard<std::mutex> lock(interface_mutex);

      for (size_t i = 0; i < actuators_groups[gi].size(); i++)
      {
        feedbacks[i]->pos = actuators_groups[gi][i].fb_pos;
        feedbacks[i]->vel = actuators_groups[gi][i].fb_vel;
        feedbacks[i]->torque = actuators_groups[gi][i].fb_torque;
        
        feedbacks[i]->time = last_update_time;
        feedbacks[i]->hw_time = last_update_time;
      }
    });

    command_receiver[gi] = createReceiver<MotorCommand>(cmd_info, [this, gi](MotorCommand** commands) {

      std::lock_guard<std::mutex> lock(interface_mutex);

      for (size_t i = 0; i < actuators_groups[gi].size(); i++)
      {
        if (commands[i]->pos) actuators_groups[gi][i].cmd_pos = *commands[i]->pos;
        if (commands[i]->vel) actuators_groups[gi][i].cmd_vel = *commands[i]->vel;
      }
    });
  }   

  gui_thread = std::thread(&Mujoco::sim_thread_func, this);
  while (!sim_created.load()) pond::sleep(0.001);

  sim->Load(m, d, robot_path.c_str());

  return POND_SUCCESS;
}

void Mujoco::onShutdown()
{
  sim->exitrequest.store(1);
  gui_thread.join();

  for (auto& rec : feedback_receiver) rec.destroy();
  for (auto& rec : command_receiver) rec.destroy();

  mj_deleteData(d);
  mj_deleteModel(m);
}

void Mujoco::onFrame()
{
  if (sim->exitrequest.load() == 2) { shutdown(); return;}

  if (sim->droploadrequest.load()) sim->droploadrequest.store(0);
  if (sim->uiloadrequest.load()) sim->uiloadrequest.store(0);

  // lock the sim mutex
  const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

  // reset timers on transition between running and paused
  if (sim->run != last_run)
  {
    if (last_run != -1)
    {
      std::memset(d->timer, 0, sizeof(d->timer));
      std::memset(sim->timer_prev_, 0, sizeof(sim->timer_prev_));
    }
    last_run = sim->run;
  }

  // running
  if (sim->run)
  {
    {
      std::lock_guard<std::mutex> lock(interface_mutex);

      last_update_time = pond::get_time();

      for (uint32_t gi = 0; gi < actuators_groups.size(); gi++)
      {
        for (size_t i = 0; i < actuators_groups[gi].size(); i++)
        {
          if (actuators_groups[gi][i].cmd_is_pos) d->ctrl[actuators_groups[gi][i].ctrl_adr] = actuators_groups[gi][i].cmd_pos;
          if (actuators_groups[gi][i].cmd_is_vel) d->ctrl[actuators_groups[gi][i].ctrl_adr] = actuators_groups[gi][i].cmd_vel;
        }
      }
    }

    bool stepped = false;

    double startCPU = pond::get_time();

    // elapsed CPU and simulation time since last sync
    double elapsedCPU = startCPU - syncCPU;
    double elapsedSim = d->time - syncSim;

    // requested slow-down factor
    double slowdown = 100 / sim->percentRealTime[sim->real_time_index];

    // misalignment condition: distance from target sim time is bigger than syncMisalign
    bool misaligned = std::abs(elapsedCPU / slowdown - elapsedSim) > syncMisalign;

    // out-of-sync (for any reason): reset sync times, step
    if (elapsedSim < 0.0 || elapsedCPU < 0.0 || syncCPU == 0.0 || misaligned || sim->speed_changed)
    {
      // re-sync
      syncCPU           = startCPU;
      syncSim           = d->time;
      sim->speed_changed = false;

      // inject noise
      sim->InjectNoise(sim->key);

      // run single step, let next iteration deal with timing
      mj_step(m, d);

      if (auto message = Diverged(m->opt.disableflags, d)) {sim->run = 0; mju::strcpy_arr(sim->load_error, message);}
      else stepped = true;
    }

    // in-sync: step until ahead of cpu
    else {
      bool   measured = false;
      mjtNum prevSim  = d->time;

      double refreshTime = simRefreshFraction / sim->refresh_rate;

      // step while sim lags behind cpu and within refreshTime
      double time_now = pond::get_time();
      while ((d->time - syncSim) * slowdown < time_now - syncCPU && time_now - startCPU < refreshTime)
      {
        // measure slowdown before first step
        if (!measured && elapsedSim)
        {
          sim->measured_slowdown = elapsedCPU / elapsedSim;
          measured = true;
        }

        // inject noise
        sim->InjectNoise(sim->key);

        // call mj_step
        mj_step(m, d);

        if (auto message = Diverged(m->opt.disableflags, d)) {sim->run = 0; mju::strcpy_arr(sim->load_error, message);}
        else stepped = true;

        // break if reset
        if (d->time < prevSim) break;
        time_now = pond::get_time();
      }
    }

    // save current state to history buffer
    if (stepped) sim->AddToHistory();
  }

  // paused
  else
  {
    // run mj_forward, to update rendering and joint sliders
    mj_forward(m, d);
    if (sim->pause_update) { mju_copy(d->qacc_warmstart, d->qacc, m->nv); }
    sim->speed_changed = true;
  }

  {
    std::lock_guard<std::mutex> lock(interface_mutex);

    for (uint32_t gi = 0; gi < actuators_groups.size(); gi++)
    {
      for (uint32_t i = 0; i < actuators_groups[gi].size(); i++)
      {
        int qpos_adr = m->jnt_qposadr[actuators_groups[gi][i].jid];
        int dof_adr  = m->jnt_dofadr[actuators_groups[gi][i].jid];

        actuators_groups[gi][i].fb_vel = d->qvel[dof_adr];
        actuators_groups[gi][i].fb_pos = d->qpos[qpos_adr];
        actuators_groups[gi][i].fb_torque = d->qfrc_actuator[dof_adr];
      }
    }
  }
}