#include "pond/hpp/module_base.hpp"
#include "pond/pond.h"
#include <pond/pond.hpp>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <simulate/glfw_adapter.h>
#include <simulate/simulate.h>
#include <simulate/array_safety.h>

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <errno.h>
  #include <unistd.h>
#endif
}

namespace {
namespace mj  = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign       = 0.1;  // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int    kErrorLength       = 1024;  // load error string length

using Seconds = std::chrono::duration<double>;


//---------------------------------------- plugin handling -----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string    realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD                   buf_size = 128;
    bool                    success  = false;
    while (!success) {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath is too small, grow and retry
        buf_size *= 2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
  #if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char* path = buf.get();
  #else
  const char* path = "/proc/self/exe";
  #endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t           buf_size = 128;

    bool success = false;
    while (!success) {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';

        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) { return ""; }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) { return realpath.substr(0, i); }
  }

  // don't scan through the entire file system's root
  return "";
}

//------------------------------------------- simulation -------------------------------------------

const char* Diverged(int disableflags, const mjData* d) {
  if (disableflags & mjDSBL_AUTORESET) {
    for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
      if (d->warning[w].number > 0) { return mju_warningText(w, d->warning[w].lastinfo); }
    }
  }
  return nullptr;
}

}

class Mujoco : public pond::ModuleBase
{
public:
  virtual pond_result onStartup(const std::vector<void*>& args) override;
  virtual void onShutdown() override;
  virtual void onFrame() override;
private:
  std::thread gui_thread;

  mjvCamera cam;
  mjvOption opt;
  mjvPerturb pert;

  // model and data
  mjModel* m = nullptr;
  mjData*  d = nullptr;

  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;
  int last_run = -1;

  std::unique_ptr<mj::Simulate> sim;

  std::atomic<bool> sim_created{false};
  std::optional<std::string> robot_path;


  void sim_thread_func()
  {
    // scan for libraries in the plugin directory to load additional plugins
    // check and print plugins that are linked directly into the executable
    int nplugin = mjp_pluginCount();
    if (nplugin) {
      POND_LOG("Built-in plugins:\n");
      for (int i = 0; i < nplugin; ++i) { POND_LOG("    %s\n", mjp_getPluginAtSlot(i)->name); }
    }

    // define platform-specific strings
  #if defined(_WIN32) || defined(__CYGWIN__)
    const std::string sep = "\\";
  #else
    const std::string sep = "/";
  #endif


    // try to open the ${EXECDIR}/MUJOCO_PLUGIN_DIR directory
    // ${EXECDIR} is the directory containing the simulate binary itself
    // MUJOCO_PLUGIN_DIR is the MUJOCO_PLUGIN_DIR preprocessor macro
    const std::string executable_dir = getExecutableDir();
    if (executable_dir.empty()) { return; }

    const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
    mj_loadAllPluginLibraries(
      plugin_dir.c_str(),
      +[](const char* filename, int first, int count) {
        printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      }
    );

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultPerturb(&pert);

    // simulate object encapsulates the UI
    sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam,
      &opt,
      &pert,
      /* is_passive = */ false
    );

    sim_created.store(true);
    sim->RenderLoop();
  }
};

POND_MODULE_CPP_DECLARE(Mujoco, "mujoco", "mujoco pond binding")

POND_BUNDLE_DECLARE(
  "mojoco bundle",
  POND_MODULE(Mujoco),
)

pond_result Mujoco::onStartup(const std::vector<void*>& args)
{
  POND_LOG("MuJoCo version %s", mj_versionString());

  if (mjVERSION_HEADER != mj_version()) POND_LOG_RETURN_ERROR("Headers and library have different versions");

  if (!(robot_path = parameter("robot_path").asString().getStrict())) return POND_ERROR;
  
  POND_LOG("Loading robot model from '%s'", robot_path->c_str());
  
  // load and compile
  char loadError[kErrorLength] = "";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mjSpec* spec = mj_parse(robot_path->c_str(), nullptr, &vfs, loadError, kErrorLength);

  if (!spec) POND_LOG("Error: Could not parse model: %s", loadError);
  else
  {
    if (!(m = mj_compile(spec, &vfs))) POND_LOG(mjs_getError(spec));
    mj_deleteSpec(spec);
  }
  mj_deleteVFS(&vfs);

  if (!spec) return POND_ERROR;
  if (!m) return POND_ERROR;
  if (!(d = mj_makeData(m))) { mj_deleteModel(m); POND_LOG_RETURN_ERROR("Failed to create model data"); }

  mj_forward(m, d);

  gui_thread = std::thread(&Mujoco::sim_thread_func, this);
  while (!sim_created.load()) pond::sleep(0.001);

  sim->Load(m, d, robot_path->c_str());

  return POND_SUCCESS;
}

void Mujoco::onShutdown()
{
  sim->exitrequest.store(1);
  gui_thread.join();

  mj_deleteData(d);
  mj_deleteModel(m);
}

void Mujoco::onFrame()
{
  if (sim->exitrequest.load() == 2) { shutdown(); return;}

  if (sim->droploadrequest.load()) sim->droploadrequest.store(0);
  if (sim->uiloadrequest.load()) sim->uiloadrequest.store(0);

  // sleep for 1 ms or yield, to let main thread run
  //  yield results in busy wait - which has better timing but kills battery life
  if (sim->run && sim->busywait) std::this_thread::yield();
  else std::this_thread::sleep_for(std::chrono::milliseconds(1));

  {
    // lock the sim mutex
    const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

    // run only if model is present
    if (m) {
      // reset timers on transition between running and paused
      if (sim->run != last_run) {
        if (last_run != -1) {
          std::memset(d->timer, 0, sizeof(d->timer));
          std::memset(sim->timer_prev_, 0, sizeof(sim->timer_prev_));
        }
        last_run = sim->run;
      }

      // running
      if (sim->run) {
        bool stepped = false;

        // record cpu time at start of iteration
        const auto startCPU = mj::Simulate::Clock::now();

        // elapsed CPU and simulation time since last sync
        const auto elapsedCPU = startCPU - syncCPU;
        double     elapsedSim = d->time - syncSim;

        // requested slow-down factor
        double slowdown = 100 / sim->percentRealTime[sim->real_time_index];

        // misalignment condition: distance from target sim time is bigger than syncMisalign
        bool misaligned =
            std::abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

        // out-of-sync (for any reason): reset sync times, step
        if (elapsedSim < 0 ||
            elapsedCPU.count() < 0 ||
            syncCPU.time_since_epoch().count() == 0 ||
            misaligned ||
            sim->speed_changed) {
          // re-sync
          syncCPU           = startCPU;
          syncSim           = d->time;
          sim->speed_changed = false;

          // inject noise
          sim->InjectNoise(sim->key);

          // run single step, let next iteration deal with timing
          mj_step(m, d);
          const char* message = Diverged(m->opt.disableflags, d);
          if (message) {
            sim->run = 0;
            mju::strcpy_arr(sim->load_error, message);
          } else {
            stepped = true;
          }
        }

        // in-sync: step until ahead of cpu
        else {
          bool   measured = false;
          mjtNum prevSim  = d->time;

          double refreshTime = simRefreshFraction / sim->refresh_rate;

          // step while sim lags behind cpu and within refreshTime
          while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                  mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
            // measure slowdown before first step
            if (!measured && elapsedSim) {
              sim->measured_slowdown =
                  std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
              measured = true;
            }

            // inject noise
            sim->InjectNoise(sim->key);

            // call mj_step
            mj_step(m, d);
            const char* message = Diverged(m->opt.disableflags, d);
            if (message) {
              sim->run = 0;
              mju::strcpy_arr(sim->load_error, message);
            } else {
              stepped = true;
            }

            // break if reset
            if (d->time < prevSim) { break; }
          }
        }

        // save current state to history buffer
        if (stepped) { sim->AddToHistory(); }
      }

      // paused
      else {
        // run mj_forward, to update rendering and joint sliders
        mj_forward(m, d);
        if (sim->pause_update) { mju_copy(d->qacc_warmstart, d->qacc, m->nv); }
        sim->speed_changed = true;
      }
    }
  }  // release std::lock_guard<std::mutex>
}