// Taken from https://github.com/jonititan/jsbsim-ffi/blob/a309f5b67a02d7e5c3d1b9559be40c27ef467887/c_wrapper/jsbsim_wrapper.c
// No licence file but the Cargo.toml specifies an MIT licence

#include "wrapper.h"
#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <initialization/FGInitialCondition.h>
#include <input_output/FGGroundCallback.h>
#include <input_output/FGPropertyManager.h>
#include <math/FGColumnVector3.h>
#include <math/FGLocation.h>
#include <models/FGInertial.h>
#include <models/FGPropulsion.h>
#include <simgear/misc/sg_path.hxx>
#include <string>
#include <cstring>
#include <algorithm>

// Helper: safely cast the opaque pointer to FGFDMExec.
static inline JSBSim::FGFDMExec* as_exec(JSBSim_FGFDMExec* fdm) {
    return reinterpret_cast<JSBSim::FGFDMExec*>(fdm);
}

// Helper: copy a std::string into a caller-supplied C buffer.
// Returns the full string length (may exceed buf_len).
static int copy_str(const std::string& src, char* buf, int buf_len) {
    if (buf && buf_len > 0) {
        int n = std::min(static_cast<int>(src.size()), buf_len - 1);
        std::memcpy(buf, src.data(), n);
        buf[n] = '\0';
    }
    return static_cast<int>(src.size());
}

extern "C" {

/* ── Lifecycle ────────────────────────────────────────────────────── */

JSBSim_FGFDMExec* jsbsim_create(const char* root_dir) {
    auto* fdm = new JSBSim::FGFDMExec();
    if (root_dir && *root_dir) {
        fdm->SetRootDir(SGPath(root_dir));
        fdm->SetAircraftPath(SGPath("aircraft"));
        fdm->SetEnginePath(SGPath("engine"));
        fdm->SetSystemsPath(SGPath("systems"));
    }
    return reinterpret_cast<JSBSim_FGFDMExec*>(fdm);
}

void jsbsim_destroy(JSBSim_FGFDMExec* fdm) {
    delete as_exec(fdm);
}

/* ── Loading ──────────────────────────────────────────────────────── */

bool jsbsim_load_model(JSBSim_FGFDMExec* fdm, const char* model) {
    if (!fdm || !model || !*model) return false;
    try {
        return as_exec(fdm)->LoadModel(std::string(model));
    } catch (...) {
        return false;
    }
}

bool jsbsim_load_model_ex(JSBSim_FGFDMExec* fdm,
                          const char* aircraft_path,
                          const char* engine_path,
                          const char* systems_path,
                          const char* model,
                          bool add_model_to_path) {
    if (!fdm || !aircraft_path || !engine_path || !systems_path || !model || !*model) {
        return false;
    }
    try {
        SGPath ap = SGPath(std::string(aircraft_path));
        SGPath ep = SGPath(std::string(engine_path));
        SGPath sp = SGPath(std::string(systems_path));
        return as_exec(fdm)->LoadModel(ap, ep, sp, std::string(model), add_model_to_path);
    } catch (...) {
        return false;
    }
}

bool jsbsim_load_script(JSBSim_FGFDMExec* fdm, const char* filename) {
    if (!fdm || !filename || !*filename) return false;
    try {
        return as_exec(fdm)->LoadScript(SGPath(std::string(filename)));
    } catch (...) {
        return false;
    }
}

bool jsbsim_load_script_ex(JSBSim_FGFDMExec* fdm, const char* filename,
                           double dt, const char* initfile) {
    if (!fdm || !filename || !*filename) return false;
    try {
        SGPath script = SGPath(std::string(filename));
        SGPath ic = (initfile && *initfile) ? SGPath(std::string(initfile))
                                            : SGPath();
        return as_exec(fdm)->LoadScript(script, dt, ic);
    } catch (...) {
        return false;
    }
}

bool jsbsim_load_planet(JSBSim_FGFDMExec* fdm, const char* filename,
                        bool use_aircraft_path) {
    if (!fdm || !filename || !*filename) return false;
    try {
        return as_exec(fdm)->LoadPlanet(SGPath(std::string(filename)),
                                        use_aircraft_path);
    } catch (...) {
        return false;
    }
}

bool jsbsim_load_ic(JSBSim_FGFDMExec* fdm, const char* filename, bool use_aircraft_path) {
    if (!fdm || !filename || !*filename) return false;
    try {
        auto ic = as_exec(fdm)->GetIC();
        if (!ic) return false;
        return ic->Load(SGPath(std::string(filename)), use_aircraft_path);
    } catch (...) {
        return false;
    }
}

/* ── Simulation control ───────────────────────────────────────────── */

bool jsbsim_run_ic(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return false;
    try {
        return as_exec(fdm)->RunIC();
    } catch (...) {
        return false;
    }
}

bool jsbsim_run(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return false;
    try {
        return as_exec(fdm)->Run();
    } catch (...) {
        return false;
    }
}

void jsbsim_set_dt(JSBSim_FGFDMExec* fdm, double dt) {
    if (!fdm) return;
    as_exec(fdm)->Setdt(dt);
}

double jsbsim_get_dt(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return 0.0;
    return as_exec(fdm)->GetDeltaT();
}

double jsbsim_get_sim_time(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return 0.0;
    return as_exec(fdm)->GetSimTime();
}

void jsbsim_reset_to_initial_conditions(JSBSim_FGFDMExec* fdm, int mode) {
    if (!fdm) return;
    try {
        as_exec(fdm)->ResetToInitialConditions(mode);
    } catch (...) {}
}

/* ── Hold / Resume ────────────────────────────────────────────────── */

void jsbsim_hold(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return;
    as_exec(fdm)->Hold();
}

void jsbsim_resume(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return;
    as_exec(fdm)->Resume();
}

bool jsbsim_holding(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return false;
    return as_exec(fdm)->Holding();
}

void jsbsim_enable_increment_then_hold(JSBSim_FGFDMExec* fdm, int steps) {
    if (!fdm) return;
    as_exec(fdm)->EnableIncrementThenHold(steps);
}

void jsbsim_check_incremental_hold(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return;
    as_exec(fdm)->CheckIncrementalHold();
}

/* ── Integration suspend ──────────────────────────────────────────── */

void jsbsim_suspend_integration(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return;
    as_exec(fdm)->SuspendIntegration();
}

void jsbsim_resume_integration(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return;
    as_exec(fdm)->ResumeIntegration();
}

/* ── Trim ─────────────────────────────────────────────────────────── */

bool jsbsim_do_trim(JSBSim_FGFDMExec* fdm, int mode) {
    if (!fdm) return false;
    try {
        as_exec(fdm)->DoTrim(mode);
        return true;
    } catch (...) {
        return false;
    }
}

bool jsbsim_do_linearization(JSBSim_FGFDMExec* fdm, int mode) {
    if (!fdm) return false;
    // DoLinearization dereferences subsystem pointers that are only populated
    // once a model has been loaded.  Guard against a SIGSEGV on a bare sim by
    // refusing to call through when no model name has been set.
    if (as_exec(fdm)->GetModelName().empty()) return false;
    try {
        as_exec(fdm)->DoLinearization(mode);
        return true;
    } catch (...) {
        return false;
    }
}

int jsbsim_get_propulsion_tank_report(JSBSim_FGFDMExec* fdm,
                                      char* buf, int buf_len) {
    if (!fdm) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
    try {
        std::string report = as_exec(fdm)->GetPropulsionTankReport();
        return copy_str(report, buf, buf_len);
    } catch (...) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
}

int jsbsim_get_random_seed(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return 0;
    return as_exec(fdm)->SRand();
}

int jsbsim_get_fdm_count(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return 0;
    return static_cast<int>(as_exec(fdm)->GetFDMCount());
}

int jsbsim_enumerate_fdms_count(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return 0;
    try {
        return static_cast<int>(as_exec(fdm)->EnumerateFDMs().size());
    } catch (...) {
        return 0;
    }
}

int jsbsim_enumerate_fdms_name(JSBSim_FGFDMExec* fdm, int i,
                               char* buf, int buf_len) {
    if (!fdm || i < 0) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
    try {
        auto list = as_exec(fdm)->EnumerateFDMs();
        if (static_cast<size_t>(i) >= list.size()) {
            if (buf && buf_len > 0) buf[0] = '\0';
            return 0;
        }
        return copy_str(list[i], buf, buf_len);
    } catch (...) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
}

void jsbsim_set_child(JSBSim_FGFDMExec* fdm, bool is_child) {
    if (!fdm) return;
    as_exec(fdm)->SetChild(is_child);
}

/* ── Propulsion helpers ──────────────────────────────────────────── */

int jsbsim_get_num_engines(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return 0;
    auto prop = as_exec(fdm)->GetPropulsion();
    if (!prop) return 0;
    return static_cast<int>(prop->GetNumEngines());
}

int jsbsim_get_num_tanks(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return 0;
    auto prop = as_exec(fdm)->GetPropulsion();
    if (!prop) return 0;
    return static_cast<int>(prop->GetNumTanks());
}

bool jsbsim_init_running(JSBSim_FGFDMExec* fdm, int n) {
    if (!fdm) return false;
    // InitRunning dereferences Engine pointers that are only populated
    // after LoadModel — guard against bare-sim SIGSEGV.
    if (as_exec(fdm)->GetModelName().empty()) return false;
    auto prop = as_exec(fdm)->GetPropulsion();
    if (!prop) return false;
    try {
        prop->InitRunning(n);
        return true;
    } catch (...) {
        return false;
    }
}

bool jsbsim_propulsion_get_steady_state(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return false;
    if (as_exec(fdm)->GetModelName().empty()) return false;
    auto prop = as_exec(fdm)->GetPropulsion();
    if (!prop) return false;
    try {
        return prop->GetSteadyState();
    } catch (...) {
        return false;
    }
}

void jsbsim_set_trim_status(JSBSim_FGFDMExec* fdm, bool status) {
    if (!fdm) return;
    as_exec(fdm)->SetTrimStatus(status);
}

bool jsbsim_get_trim_status(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return false;
    return as_exec(fdm)->GetTrimStatus();
}

void jsbsim_set_trim_mode(JSBSim_FGFDMExec* fdm, int mode) {
    if (!fdm) return;
    as_exec(fdm)->SetTrimMode(mode);
}

int jsbsim_get_trim_mode(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return 0;
    return as_exec(fdm)->GetTrimMode();
}

/* ── Properties ───────────────────────────────────────────────────── */

double jsbsim_get_property(JSBSim_FGFDMExec* fdm, const char* name) {
    if (!fdm || !name) return 0.0;
    return as_exec(fdm)->GetPropertyValue(name);
}

bool jsbsim_set_property(JSBSim_FGFDMExec* fdm, const char* name, double value) {
    if (!fdm || !name) return false;
    as_exec(fdm)->SetPropertyValue(name, value);
    return true;
}

bool jsbsim_has_property(JSBSim_FGFDMExec* fdm, const char* name) {
    if (!fdm || !name) return false;
    auto pm = as_exec(fdm)->GetPropertyManager();
    if (!pm) return false;
    return pm->HasNode(std::string(name));
}

/* ── Property catalog ─────────────────────────────────────────────── */

int jsbsim_query_property_catalog(JSBSim_FGFDMExec* fdm, const char* check,
                                  char* buf, int buf_len) {
    if (!fdm || !check) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
    std::string result = as_exec(fdm)->QueryPropertyCatalog(std::string(check));
    return copy_str(result, buf, buf_len);
}

void jsbsim_print_property_catalog(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return;
    as_exec(fdm)->PrintPropertyCatalog();
}

void jsbsim_print_simulation_configuration(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return;
    try {
        as_exec(fdm)->PrintSimulationConfiguration();
    } catch (...) {}
}

int jsbsim_get_property_catalog_size(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return 0;
    try {
        return static_cast<int>(as_exec(fdm)->GetPropertyCatalog().size());
    } catch (...) {
        return 0;
    }
}

int jsbsim_get_property_catalog_entry(JSBSim_FGFDMExec* fdm, int i,
                                      char* buf, int buf_len) {
    if (!fdm || i < 0) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
    try {
        auto& cat = as_exec(fdm)->GetPropertyCatalog();
        if (static_cast<size_t>(i) >= cat.size()) {
            if (buf && buf_len > 0) buf[0] = '\0';
            return 0;
        }
        return copy_str(cat[i], buf, buf_len);
    } catch (...) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
}

/* ── Output control ───────────────────────────────────────────────── */

bool jsbsim_set_output_directive(JSBSim_FGFDMExec* fdm, const char* fname) {
    if (!fdm || !fname || !*fname) return false;
    try {
        return as_exec(fdm)->SetOutputDirectives(SGPath(std::string(fname)));
    } catch (...) {
        return false;
    }
}

void jsbsim_force_output(JSBSim_FGFDMExec* fdm, int idx) {
    if (!fdm) return;
    try {
        as_exec(fdm)->ForceOutput(idx);
    } catch (...) {}
}

void jsbsim_set_logging_rate(JSBSim_FGFDMExec* fdm, double rate_hz) {
    if (!fdm) return;
    try {
        as_exec(fdm)->SetLoggingRate(rate_hz);
    } catch (...) {}
}

void jsbsim_enable_output(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return;
    as_exec(fdm)->EnableOutput();
}

void jsbsim_disable_output(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return;
    as_exec(fdm)->DisableOutput();
}

bool jsbsim_set_output_filename(JSBSim_FGFDMExec* fdm, int n, const char* fname) {
    if (!fdm || !fname) return false;
    try {
        return as_exec(fdm)->SetOutputFileName(n, std::string(fname));
    } catch (...) {
        return false;
    }
}

/* ── Path configuration ───────────────────────────────────────────── */

bool jsbsim_set_aircraft_path(JSBSim_FGFDMExec* fdm, const char* path) {
    if (!fdm || !path) return false;
    return as_exec(fdm)->SetAircraftPath(SGPath(std::string(path)));
}

bool jsbsim_set_engine_path(JSBSim_FGFDMExec* fdm, const char* path) {
    if (!fdm || !path) return false;
    return as_exec(fdm)->SetEnginePath(SGPath(std::string(path)));
}

bool jsbsim_set_systems_path(JSBSim_FGFDMExec* fdm, const char* path) {
    if (!fdm || !path) return false;
    return as_exec(fdm)->SetSystemsPath(SGPath(std::string(path)));
}

bool jsbsim_set_output_path(JSBSim_FGFDMExec* fdm, const char* path) {
    if (!fdm || !path) return false;
    return as_exec(fdm)->SetOutputPath(SGPath(std::string(path)));
}

void jsbsim_set_root_dir(JSBSim_FGFDMExec* fdm, const char* path) {
    if (!fdm || !path) return;
    as_exec(fdm)->SetRootDir(SGPath(std::string(path)));
}

/* ── Integration state query ──────────────────────────────────────── */

bool jsbsim_integration_suspended(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return false;
    return as_exec(fdm)->IntegrationSuspended();
}

/* ── Simulation time ──────────────────────────────────────────────── */

void jsbsim_set_sim_time(JSBSim_FGFDMExec* fdm, double time) {
    if (!fdm) return;
    as_exec(fdm)->Setsim_time(time);
}

double jsbsim_incr_time(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return 0.0;
    return as_exec(fdm)->IncrTime();
}

unsigned int jsbsim_get_frame(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return 0;
    return as_exec(fdm)->GetFrame();
}

int jsbsim_get_debug_level(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return 0;
    return as_exec(fdm)->GetDebugLevel();
}

void jsbsim_set_hold_down(JSBSim_FGFDMExec* fdm, bool hold_down) {
    if (!fdm) return;
    try {
        as_exec(fdm)->SetHoldDown(hold_down);
    } catch (...) {}
}

bool jsbsim_get_hold_down(JSBSim_FGFDMExec* fdm) {
    if (!fdm) return false;
    return as_exec(fdm)->GetHoldDown();
}

/* ── Path getters ─────────────────────────────────────────────────── */

int jsbsim_get_root_dir(JSBSim_FGFDMExec* fdm, char* buf, int buf_len) {
    if (!fdm) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
    const std::string path = as_exec(fdm)->GetRootDir().utf8Str();
    return copy_str(path, buf, buf_len);
}

int jsbsim_get_aircraft_path(JSBSim_FGFDMExec* fdm, char* buf, int buf_len) {
    if (!fdm) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
    const std::string path = as_exec(fdm)->GetAircraftPath().utf8Str();
    return copy_str(path, buf, buf_len);
}

int jsbsim_get_engine_path(JSBSim_FGFDMExec* fdm, char* buf, int buf_len) {
    if (!fdm) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
    const std::string path = as_exec(fdm)->GetEnginePath().utf8Str();
    return copy_str(path, buf, buf_len);
}

int jsbsim_get_systems_path(JSBSim_FGFDMExec* fdm, char* buf, int buf_len) {
    if (!fdm) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
    const std::string path = as_exec(fdm)->GetSystemsPath().utf8Str();
    return copy_str(path, buf, buf_len);
}

int jsbsim_get_output_path(JSBSim_FGFDMExec* fdm, char* buf, int buf_len) {
    if (!fdm) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
    const std::string path = as_exec(fdm)->GetOutputPath().utf8Str();
    return copy_str(path, buf, buf_len);
}

int jsbsim_get_full_aircraft_path(JSBSim_FGFDMExec* fdm, char* buf, int buf_len) {
    if (!fdm) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
    const std::string path = as_exec(fdm)->GetFullAircraftPath().utf8Str();
    return copy_str(path, buf, buf_len);
}

/* ── Output filename getter ───────────────────────────────────────── */

int jsbsim_get_output_filename(JSBSim_FGFDMExec* fdm, int n, char* buf, int buf_len) {
    if (!fdm) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
    const std::string name = as_exec(fdm)->GetOutputFileName(n);
    return copy_str(name, buf, buf_len);
}

/* ── Info / Debug ─────────────────────────────────────────────────── */

void jsbsim_set_debug_level(JSBSim_FGFDMExec* fdm, int level) {
    if (!fdm) return;
    as_exec(fdm)->SetDebugLevel(level);
}

int jsbsim_get_model_name(JSBSim_FGFDMExec* fdm, char* buf, int buf_len) {
    if (!fdm) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }
    const std::string& name = as_exec(fdm)->GetModelName();
    return copy_str(name, buf, buf_len);
}

int jsbsim_get_version(char* buf, int buf_len) {
    const std::string& ver = JSBSim::FGJSBBase::GetVersion();
    return copy_str(ver, buf, buf_len);
}

} // extern "C"

/* ── Ground callback bridge (C++ class, defined outside extern "C") ── */

/// C++ class that inherits FGGroundCallback and delegates to a C function
/// pointer, enabling Rust (or any C-ABI language) to provide custom terrain.
class FFIGroundCallback : public JSBSim::FGGroundCallback {
public:
    FFIGroundCallback(jsbsim_get_agl_fn_t fn, void* ud)
        : get_agl_fn(fn), user_data(ud) {}

    double GetAGLevel(double t, const JSBSim::FGLocation& location,
                      JSBSim::FGLocation& contact,
                      JSBSim::FGColumnVector3& normal,
                      JSBSim::FGColumnVector3& v,
                      JSBSim::FGColumnVector3& w) const override
    {
        // Extract ECEF coordinates from the query location.
        double loc[3] = { location(1), location(2), location(3) };
        double ct[3], n[3], vel[3], av[3];

        double agl = get_agl_fn(user_data, t, loc, ct, n, vel, av);

        // Reconstruct the contact point.  Copy the input location first so
        // that the FGLocation carries the correct ellipsoid parameters, then
        // overwrite with the ECEF values returned by the callback.
        contact = location;
        contact(1) = ct[0];
        contact(2) = ct[1];
        contact(3) = ct[2];

        normal(1)  = n[0];   normal(2)  = n[1];   normal(3)  = n[2];
        v(1)       = vel[0]; v(2)       = vel[1]; v(3)       = vel[2];
        w(1)       = av[0];  w(2)       = av[1];  w(3)       = av[2];

        return agl;
    }

private:
    jsbsim_get_agl_fn_t get_agl_fn;
    void*               user_data;
};

/* ── Ground callback C entry points ───────────────────────────────── */

extern "C" {

void jsbsim_set_ground_callback(JSBSim_FGFDMExec* fdm,
                                jsbsim_get_agl_fn_t get_agl,
                                void* user_data) {
    if (!fdm || !get_agl) return;
    auto inertial = as_exec(fdm)->GetInertial();
    if (!inertial) return;
    // FGInertial takes ownership via unique_ptr.
    inertial->SetGroundCallback(new FFIGroundCallback(get_agl, user_data));
}

void jsbsim_set_terrain_elevation(JSBSim_FGFDMExec* fdm, double elevation_ft) {
    if (!fdm) return;
    auto inertial = as_exec(fdm)->GetInertial();
    if (!inertial) return;
    inertial->SetTerrainElevation(elevation_ft);
}

} // extern "C"
