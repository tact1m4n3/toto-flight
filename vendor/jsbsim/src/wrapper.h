// Taken from https://github.com/jonititan/jsbsim-ffi/blob/a309f5b67a02d7e5c3d1b9559be40c27ef467887/c_wrapper/jsbsim_wrapper.h

/**
 * @file jsbsim_wrapper.h
 * @brief C FFI wrapper for JSBSim's FGFDMExec flight dynamics executive.
 *
 * This header declares a flat C API that wraps the C++ class
 * [JSBSim::FGFDMExec](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h).
 * It is consumed by the Rust FFI bindings in `src/lib.rs`.
 *
 * **JSBSim C++ traceability:** each function documents which C++ method
 * from the [JSBSim source tree](https://github.com/JSBSim-Team/jsbsim/tree/master/src)
 * it wraps.
 */
#ifndef JSBSIM_WRAPPER_H
#define JSBSIM_WRAPPER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque handle to a JSBSim FGFDMExec instance.
 *
 *  **C++ origin:** [`JSBSim::FGFDMExec`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
typedef struct JSBSim_FGFDMExec JSBSim_FGFDMExec;

/* ── Lifecycle ────────────────────────────────────────────────────── */

/** Create a new FGFDMExec and configure its data paths.
 *
 *  **C++ origin:**
 *  - `FGFDMExec::FGFDMExec()` constructor
 *  - `FGFDMExec::SetRootDir(SGPath)` — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 *  - `FGFDMExec::SetAircraftPath(SGPath("aircraft"))`
 *  - `FGFDMExec::SetEnginePath(SGPath("engine"))`
 *  - `FGFDMExec::SetSystemsPath(SGPath("systems"))`
 */
JSBSim_FGFDMExec* jsbsim_create(const char* root_dir);

/** Destroy an FGFDMExec instance.
 *
 *  **C++ origin:** `~FGFDMExec()` destructor — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_destroy(JSBSim_FGFDMExec* fdm);

/* ── Loading ──────────────────────────────────────────────────────── */

/** Load an aircraft model by name (e.g. "c172x").
 *
 *  **C++ origin:** `FGFDMExec::LoadModel(const string& model, bool addModelToPath = true)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_load_model(JSBSim_FGFDMExec* fdm, const char* model);

/** Load an aircraft model and re-target the aircraft, engine, and systems
 *  search paths in a single call.
 *
 *  **C++ origin:** the 5-arg overload of
 *  `FGFDMExec::LoadModel(SGPath aircraft, SGPath engines, SGPath systems,
 *                        const string& model, bool addModelToPath = true)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_load_model_ex(JSBSim_FGFDMExec* fdm,
                          const char* aircraft_path,
                          const char* engine_path,
                          const char* systems_path,
                          const char* model,
                          bool add_model_to_path);

/** Load a JSBSim XML script file.
 *
 *  **C++ origin:** `FGFDMExec::LoadScript(const SGPath& Script, double deltaT, const SGPath& initfile)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_load_script(JSBSim_FGFDMExec* fdm, const char* filename);

/** Load a JSBSim XML script file with overrides for the time step and the
 *  initial-conditions file.
 *
 *  @param fdm       FDM handle.
 *  @param filename  path to the script file.
 *  @param dt        integration step size to override the script's value
 *                   (use 0.0 to keep the script's own value).
 *  @param initfile  optional override for the IC file referenced by the
 *                   script.  Pass NULL or "" to keep the script's own IC.
 *
 *  **C++ origin:** `FGFDMExec::LoadScript(const SGPath& Script, double deltaT, const SGPath& initfile)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_load_script_ex(JSBSim_FGFDMExec* fdm,
                           const char* filename,
                           double dt,
                           const char* initfile);

/** Load a planet definition XML file.
 *
 *  **C++ origin:** `FGFDMExec::LoadPlanet(const SGPath& PlanetPath, bool useAircraftPath)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_load_planet(JSBSim_FGFDMExec* fdm,
                        const char* filename,
                        bool use_aircraft_path);

/** Load initial conditions from an XML file.
 *
 *  **C++ origin:** `FGInitialCondition::Load(const SGPath&, bool)`
 *  — [`src/initialization/FGInitialCondition.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/initialization/FGInitialCondition.h)
 *  via `FGFDMExec::GetIC()->Load(...)`.
 */
bool jsbsim_load_ic(JSBSim_FGFDMExec* fdm, const char* filename, bool use_aircraft_path);

/* ── Simulation control ───────────────────────────────────────────── */

/** Initialize the simulation from current ICs.
 *
 *  **C++ origin:** `FGFDMExec::RunIC()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_run_ic(JSBSim_FGFDMExec* fdm);

/** Advance the simulation by one time step.
 *
 *  **C++ origin:** `FGFDMExec::Run()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_run(JSBSim_FGFDMExec* fdm);

/** Set the simulation time step (seconds).
 *
 *  **C++ origin:** `FGFDMExec::Setdt(double delta_t)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_set_dt(JSBSim_FGFDMExec* fdm, double dt);

/** Get the current time step (seconds).
 *
 *  **C++ origin:** `FGFDMExec::GetDeltaT()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
double jsbsim_get_dt(JSBSim_FGFDMExec* fdm);

/** Get the current simulation time (seconds).
 *
 *  **C++ origin:** `FGFDMExec::GetSimTime()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
double jsbsim_get_sim_time(JSBSim_FGFDMExec* fdm);

/** Reset the simulation to initial conditions.
 *
 *  **C++ origin:** `FGFDMExec::ResetToInitialConditions(int mode)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_reset_to_initial_conditions(JSBSim_FGFDMExec* fdm, int mode);

/* ── Hold / Resume ────────────────────────────────────────────────── */

/** Pause the simulation.
 *
 *  **C++ origin:** `FGFDMExec::Hold()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_hold(JSBSim_FGFDMExec* fdm);

/** Resume from hold.
 *
 *  **C++ origin:** `FGFDMExec::Resume()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_resume(JSBSim_FGFDMExec* fdm);

/** Query hold state.
 *
 *  **C++ origin:** `FGFDMExec::Holding()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_holding(JSBSim_FGFDMExec* fdm);

/** Run N steps then auto-hold.
 *
 *  **C++ origin:** `FGFDMExec::EnableIncrementThenHold(int Timesteps)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_enable_increment_then_hold(JSBSim_FGFDMExec* fdm, int steps);

/** Process incremental-hold check.
 *
 *  **C++ origin:** `FGFDMExec::CheckIncrementalHold()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_check_incremental_hold(JSBSim_FGFDMExec* fdm);

/* ── Integration suspend ──────────────────────────────────────────── */

/** Freeze physics integration (saves dT, sets to 0).
 *
 *  **C++ origin:** `FGFDMExec::SuspendIntegration()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_suspend_integration(JSBSim_FGFDMExec* fdm);

/** Resume physics integration (restores dT from saved_dT).
 *
 *  **C++ origin:** `FGFDMExec::ResumeIntegration()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_resume_integration(JSBSim_FGFDMExec* fdm);

/* ── Trim ─────────────────────────────────────────────────────────── */

/** Trim the aircraft.
 *
 *  **C++ origin:** `FGFDMExec::DoTrim(int mode)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h).
 *  Mode values from `tType` enum: 0=Longitudinal, 1=Full, 2=Ground, etc.
 */
bool jsbsim_do_trim(JSBSim_FGFDMExec* fdm, int mode);

/** Run the linearization algorithm.  Aircraft must be trimmed first.
 *
 *  **C++ origin:** `FGFDMExec::DoLinearization(int)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_do_linearization(JSBSim_FGFDMExec* fdm, int mode);

/** Get the propulsion tank report string.  Writes into @p buf; returns
 *  the full string length.
 *
 *  **C++ origin:** `FGFDMExec::GetPropulsionTankReport()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_get_propulsion_tank_report(JSBSim_FGFDMExec* fdm,
                                      char* buf,
                                      int buf_len);

/** Get the current random seed.
 *
 *  **C++ origin:** `FGFDMExec::SRand() const`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_get_random_seed(JSBSim_FGFDMExec* fdm);

/** Get the number of child FDMs attached to this executive.
 *
 *  **C++ origin:** `FGFDMExec::GetFDMCount()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_get_fdm_count(JSBSim_FGFDMExec* fdm);

/** Get the number of loaded FDMs (EnumerateFDMs list size).
 *
 *  **C++ origin:** `FGFDMExec::EnumerateFDMs().size()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_enumerate_fdms_count(JSBSim_FGFDMExec* fdm);

/** Get the name of the FDM at index @p i from EnumerateFDMs().
 *
 *  **C++ origin:** `FGFDMExec::EnumerateFDMs()[i]`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_enumerate_fdms_name(JSBSim_FGFDMExec* fdm,
                               int i,
                               char* buf,
                               int buf_len);

/** Mark this instance as a child FDM.
 *
 *  **C++ origin:** `FGFDMExec::SetChild(bool)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_set_child(JSBSim_FGFDMExec* fdm, bool is_child);

/* ── Propulsion helpers ──────────────────────────────────────────── */

/** Get the number of engines defined for the loaded aircraft.
 *
 *  **C++ origin:** `FGPropulsion::GetNumEngines()` via
 *  `FGFDMExec::GetPropulsion()`
 *  — [`src/models/FGPropulsion.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGPropulsion.h)
 */
int jsbsim_get_num_engines(JSBSim_FGFDMExec* fdm);

/** Get the number of fuel tanks defined for the loaded aircraft.
 *
 *  **C++ origin:** `FGPropulsion::GetNumTanks()` via
 *  `FGFDMExec::GetPropulsion()`
 *  — [`src/models/FGPropulsion.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGPropulsion.h)
 */
int jsbsim_get_num_tanks(JSBSim_FGFDMExec* fdm);

/** Warm-start engines as "running" (skips the cold-start procedure).
 *
 *  @param n  engine index, or `-1` to start all engines at once.
 *
 *  **C++ origin:** `FGPropulsion::InitRunning(int)` via
 *  `FGFDMExec::GetPropulsion()`
 *  — [`src/models/FGPropulsion.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGPropulsion.h)
 */
bool jsbsim_init_running(JSBSim_FGFDMExec* fdm, int n);

/** Iterate the propulsion model until thrust output is steady (used
 *  during trim).
 *
 *  **C++ origin:** `FGPropulsion::GetSteadyState()` via
 *  `FGFDMExec::GetPropulsion()`
 *  — [`src/models/FGPropulsion.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGPropulsion.h)
 */
bool jsbsim_propulsion_get_steady_state(JSBSim_FGFDMExec* fdm);

/** Set the trim status flag.
 *
 *  **C++ origin:** `FGFDMExec::SetTrimStatus(bool)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_set_trim_status(JSBSim_FGFDMExec* fdm, bool status);

/** Get the trim status flag.
 *
 *  **C++ origin:** `FGFDMExec::GetTrimStatus()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_get_trim_status(JSBSim_FGFDMExec* fdm);

/** Set the stored trim mode (does not invoke the trimmer).
 *
 *  **C++ origin:** `FGFDMExec::SetTrimMode(int)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_set_trim_mode(JSBSim_FGFDMExec* fdm, int mode);

/** Get the stored trim mode.
 *
 *  **C++ origin:** `FGFDMExec::GetTrimMode()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_get_trim_mode(JSBSim_FGFDMExec* fdm);

/* ── Properties ───────────────────────────────────────────────────── */

/** Read a property value.
 *
 *  **C++ origin:** `FGFDMExec::GetPropertyValue(const string& property)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
double jsbsim_get_property(JSBSim_FGFDMExec* fdm, const char* name);

/** Set a property value.
 *
 *  **C++ origin:** `FGFDMExec::SetPropertyValue(const string& property, double value)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_set_property(JSBSim_FGFDMExec* fdm, const char* name, double value);

/** Check if a property node exists.
 *
 *  **C++ origin:** `FGPropertyManager::HasNode(const string&)`
 *  — [`src/input_output/FGPropertyManager.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/input_output/FGPropertyManager.h)
 *  via `FGFDMExec::GetPropertyManager()->HasNode(name)`.
 */
bool jsbsim_has_property(JSBSim_FGFDMExec* fdm, const char* name);

/* ── Property catalog ─────────────────────────────────────────────── */

/** Search the property catalog.  Writes result into buf; returns string length.
 *
 *  **C++ origin:** `FGFDMExec::QueryPropertyCatalog(const string& check, const string& end_of_line)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int  jsbsim_query_property_catalog(JSBSim_FGFDMExec* fdm, const char* check,
                                   char* buf, int buf_len);

/** Print the full property catalog to stdout.
 *
 *  **C++ origin:** `FGFDMExec::PrintPropertyCatalog()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_print_property_catalog(JSBSim_FGFDMExec* fdm);

/** Print the simulation configuration to stdout.
 *
 *  **C++ origin:** `FGFDMExec::PrintSimulationConfiguration()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_print_simulation_configuration(JSBSim_FGFDMExec* fdm);

/** Get the number of entries in the property catalog.
 *
 *  **C++ origin:** `FGFDMExec::GetPropertyCatalog().size()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_get_property_catalog_size(JSBSim_FGFDMExec* fdm);

/** Get the property catalog entry at index `i`.  Writes into @p buf;
 *  returns the full entry length.  Returns 0 if @p i is out of range.
 *
 *  **C++ origin:** `FGFDMExec::GetPropertyCatalog()[i]`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_get_property_catalog_entry(JSBSim_FGFDMExec* fdm,
                                      int i,
                                      char* buf,
                                      int buf_len);

/* ── Output control ───────────────────────────────────────────────── */

/** Add an output directive from an XML file.
 *
 *  **C++ origin:** `FGFDMExec::SetOutputDirectives(const SGPath& fname)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_set_output_directive(JSBSim_FGFDMExec* fdm, const char* fname);

/** Force a single output to be flushed once.
 *
 *  **C++ origin:** `FGFDMExec::ForceOutput(int idx)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_force_output(JSBSim_FGFDMExec* fdm, int idx);

/** Set the global logging rate (Hz) for all output objects.
 *
 *  **C++ origin:** `FGFDMExec::SetLoggingRate(double rate)` → `FGOutput::SetRateHz(rate)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_set_logging_rate(JSBSim_FGFDMExec* fdm, double rate_hz);

/** Enable data output.
 *
 *  **C++ origin:** `FGFDMExec::EnableOutput()` → `FGOutput::Enable()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h),
 *    [`src/models/FGOutput.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGOutput.h)
 */
void jsbsim_enable_output(JSBSim_FGFDMExec* fdm);

/** Disable data output.
 *
 *  **C++ origin:** `FGFDMExec::DisableOutput()` → `FGOutput::Disable()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h),
 *    [`src/models/FGOutput.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGOutput.h)
 */
void jsbsim_disable_output(JSBSim_FGFDMExec* fdm);

/** Set the filename for output channel n.
 *
 *  **C++ origin:** `FGFDMExec::SetOutputFileName(int n, const string& fname)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_set_output_filename(JSBSim_FGFDMExec* fdm, int n, const char* fname);

/* ── Path configuration ───────────────────────────────────────────── */

/** Set the aircraft search path.
 *
 *  **C++ origin:** `FGFDMExec::SetAircraftPath(const SGPath& path)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_set_aircraft_path(JSBSim_FGFDMExec* fdm, const char* path);

/** Set the engine search path.
 *
 *  **C++ origin:** `FGFDMExec::SetEnginePath(const SGPath& path)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_set_engine_path(JSBSim_FGFDMExec* fdm, const char* path);

/** Set the systems search path.
 *
 *  **C++ origin:** `FGFDMExec::SetSystemsPath(const SGPath& path)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_set_systems_path(JSBSim_FGFDMExec* fdm, const char* path);

/** Set the directory where output files will be written.
 *
 *  **C++ origin:** `FGFDMExec::SetOutputPath(const SGPath& path)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_set_output_path(JSBSim_FGFDMExec* fdm, const char* path);

/** Set the root directory used to resolve relative paths.
 *
 *  Note: SetRootDir does NOT update the aircraft / engine / systems
 *  / output paths — call the corresponding setters separately if you
 *  need them re-rooted as well.
 *
 *  **C++ origin:** `FGFDMExec::SetRootDir(const SGPath& rootDir)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_set_root_dir(JSBSim_FGFDMExec* fdm, const char* path);

/* ── Integration state query ──────────────────────────────────────── */

/** Query whether integration is suspended.
 *
 *  **C++ origin:** `FGFDMExec::IntegrationSuspended()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_integration_suspended(JSBSim_FGFDMExec* fdm);

/* ── Simulation time ──────────────────────────────────────────────── */

/** Set the simulation time directly.
 *
 *  **C++ origin:** `FGFDMExec::Setsim_time(double cur_time)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_set_sim_time(JSBSim_FGFDMExec* fdm, double time);

/** Increment simulation time by `dt` (when not held) and return the new time.
 *
 *  **C++ origin:** `FGFDMExec::IncrTime()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
double jsbsim_incr_time(JSBSim_FGFDMExec* fdm);

/** Get the current frame counter.
 *
 *  **C++ origin:** `FGFDMExec::GetFrame()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
unsigned int jsbsim_get_frame(JSBSim_FGFDMExec* fdm);

/** Get the current debug level.
 *
 *  **C++ origin:** `FGFDMExec::GetDebugLevel()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_get_debug_level(JSBSim_FGFDMExec* fdm);

/** Set the hold-down flag (forces/hold-down property).
 *
 *  **C++ origin:** `FGFDMExec::SetHoldDown(bool)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_set_hold_down(JSBSim_FGFDMExec* fdm, bool hold_down);

/** Get the hold-down flag.
 *
 *  **C++ origin:** `FGFDMExec::GetHoldDown()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
bool jsbsim_get_hold_down(JSBSim_FGFDMExec* fdm);

/* ── Path getters ─────────────────────────────────────────────────── */

/** Get the root directory path.
 *
 *  **C++ origin:** `FGFDMExec::GetRootDir()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_get_root_dir(JSBSim_FGFDMExec* fdm, char* buf, int buf_len);

/** Get the aircraft search path.
 *
 *  **C++ origin:** `FGFDMExec::GetAircraftPath()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_get_aircraft_path(JSBSim_FGFDMExec* fdm, char* buf, int buf_len);

/** Get the engine search path.
 *
 *  **C++ origin:** `FGFDMExec::GetEnginePath()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_get_engine_path(JSBSim_FGFDMExec* fdm, char* buf, int buf_len);

/** Get the systems search path.
 *
 *  **C++ origin:** `FGFDMExec::GetSystemsPath()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_get_systems_path(JSBSim_FGFDMExec* fdm, char* buf, int buf_len);

/** Get the output directory path.
 *
 *  **C++ origin:** `FGFDMExec::GetOutputPath()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_get_output_path(JSBSim_FGFDMExec* fdm, char* buf, int buf_len);

/** Get the fully-resolved aircraft path (root + aircraft + model).
 *
 *  **C++ origin:** `FGFDMExec::GetFullAircraftPath()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_get_full_aircraft_path(JSBSim_FGFDMExec* fdm, char* buf, int buf_len);

/* ── Output filename getter ───────────────────────────────────────── */

/** Get the filename for output channel n.
 *
 *  **C++ origin:** `FGFDMExec::GetOutputFileName(int n)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int jsbsim_get_output_filename(JSBSim_FGFDMExec* fdm, int n, char* buf, int buf_len);

/* ── Info / Debug ─────────────────────────────────────────────────── */

/** Set the debug output level.
 *
 *  **C++ origin:** `FGFDMExec::SetDebugLevel(int level)`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
void jsbsim_set_debug_level(JSBSim_FGFDMExec* fdm, int level);

/** Get the loaded model name.  Writes into buf; returns string length.
 *
 *  **C++ origin:** `FGFDMExec::GetModelName()`
 *  — [`src/FGFDMExec.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGFDMExec.h)
 */
int  jsbsim_get_model_name(JSBSim_FGFDMExec* fdm, char* buf, int buf_len);

/** Get the JSBSim version string.
 *
 *  **C++ origin:** `FGJSBBase::GetVersion()`
 *  — [`src/FGJSBBase.h`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/FGJSBBase.h)
 */
int  jsbsim_get_version(char* buf, int buf_len);

/* ── Ground callback ──────────────────────────────────────────────── */

/** Function-pointer type for a custom GetAGLevel callback.
 *
 *  **C++ origin:** mirrors the virtual method signature of
 *  [`FGGroundCallback::GetAGLevel()`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/input_output/FGGroundCallback.h).
 *
 *  All vectors are Earth-Centered Earth-Fixed (ECEF), distances in feet.
 *
 *  The function must populate the four output arrays and return the
 *  Above-Ground-Level altitude (ft).
 *
 *  @param user_data      Opaque pointer passed through from
 *                        jsbsim_set_ground_callback().
 *  @param time           Simulation time (s).
 *  @param location       [in]  ECEF position of the query point (ft).
 *  @param contact        [out] ECEF position of the ground contact point (ft).
 *  @param normal         [out] Unit surface normal at the contact point.
 *  @param velocity       [out] Linear velocity of the surface (ft/s).
 *  @param ang_velocity   [out] Angular velocity of the surface (rad/s).
 *  @return AGL altitude (ft).
 */
typedef double (*jsbsim_get_agl_fn_t)(
    void*        user_data,
    double       time,
    const double location[3],
    double       contact[3],
    double       normal[3],
    double       velocity[3],
    double       ang_velocity[3]
);

/** Install a custom ground callback.
 *
 *  **C++ origin:**
 *  [`FGInertial::SetGroundCallback(FGGroundCallback*)`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGInertial.h)
 *  via `FGFDMExec::GetInertial()->SetGroundCallback(...)`.
 *
 *  JSBSim takes ownership of the C++ bridge object that wraps @p get_agl.
 *  @p user_data is forwarded to every invocation; its lifetime must be managed
 *  by the caller (i.e. it must remain valid until the callback is replaced or
 *  the FDM is destroyed).
 */
void jsbsim_set_ground_callback(JSBSim_FGFDMExec* fdm,
                                jsbsim_get_agl_fn_t get_agl,
                                void* user_data);

/** Set the terrain elevation (ft MSL) on the current ground callback.
 *
 *  **C++ origin:**
 *  [`FGInertial::SetTerrainElevation(double)`](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGInertial.h)
 *  via `FGFDMExec::GetInertial()->SetTerrainElevation(...)`.
 *
 *  Only effective when the default (sphere-earth) ground callback is active,
 *  or when a custom callback honours SetTerrainElevation().
 */
void jsbsim_set_terrain_elevation(JSBSim_FGFDMExec* fdm, double elevation_ft);

#ifdef __cplusplus
}
#endif
#endif
