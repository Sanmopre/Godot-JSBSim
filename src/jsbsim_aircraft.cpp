#include "jsbsim_aircraft.h"

// Godot
#include "godot_cpp/core/class_db.hpp"


namespace godot 
{

namespace
{

// WGS84
static constexpr double WGS84_A  = 6378137.0;
static constexpr double WGS84_F  = 1.0 / 298.257223563;
static constexpr double WGS84_E2 = WGS84_F * (2.0 - WGS84_F);

[[nodiscard]] Vec3d lla_to_ecef(double lat, double lon, double h) {
    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);

    const double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);

    return {
        (N + h) * cos_lat * cos_lon,
        (N + h) * cos_lat * sin_lon,
        (N * (1.0 - WGS84_E2) + h) * sin_lat
    };
}

[[nodiscard]] Vec3d ecef_to_enu(
    const Vec3d& ecef,
    const Vec3d& ecef0,
    double lat0,
    double lon0) 
  {
    const double dx = ecef.x - ecef0.x;
    const double dy = ecef.y - ecef0.y;
    const double dz = ecef.z - ecef0.z;

    const double sin_lat = std::sin(lat0);
    const double cos_lat = std::cos(lat0);
    const double sin_lon = std::sin(lon0);
    const double cos_lon = std::cos(lon0);

    return {
        -sin_lon * dx + cos_lon * dy,                                            // East
        cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz,          // Up
        -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz          // North
    };
  }

}

void JSBSimAircraft::_bind_methods() 
{
  ClassDB::bind_method(D_METHOD("getRootDir"), &JSBSimAircraft::getRootDir);
	ClassDB::bind_method(D_METHOD("setRootDir", "rootDir"), &JSBSimAircraft::setRootDir);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "rootDir"), "setRootDir", "getRootDir");

  ClassDB::bind_method(D_METHOD("getAircraft"), &JSBSimAircraft::getAircraft);
	ClassDB::bind_method(D_METHOD("setAircraft", "aircraft"), &JSBSimAircraft::setAircraft);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "aircraft"), "setAircraft", "getAircraft");

  ClassDB::bind_method(D_METHOD("getSimulationHertz"), &JSBSimAircraft::getSimulationHertz);
	ClassDB::bind_method(D_METHOD("setSimulationHertz", "simulationHertz"), &JSBSimAircraft::setSimulationHertz);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "simulationHertz", PROPERTY_HINT_RANGE, "30.0,120.0"), "setSimulationHertz", "getSimulationHertz");
}

void JSBSimAircraft::_ready()
{
  // Set the paths for the JSBSim xmls
  fdm.SetRootDir(SGPath(getRootDir().utf8().get_data()));
  fdm.SetAircraftPath(SGPath("aircraft"));
  fdm.SetEnginePath(SGPath("engine"));
  fdm.SetSystemsPath(SGPath("systems"));

  if (!fdm.LoadModel(getAircraft().utf8().get_data())) 
  {
      print_error("JSBSimAircraft -> Failed to load model");
      return;
  }

  // Set simulation step time. This value has to match the Godot configuration
  fdm.Setdt( 1.0 / getSimulationHertz());

  // ---- Initial conditions (example values) ----
  // Many ICs are set through the property system.
  fdm.SetPropertyValue("ic/h-sl-ft", 3000.0);      // altitude (ft)
  fdm.SetPropertyValue("ic/u-fps",   150.0);       // forward speed (ft/s) ~ 89 kts
  fdm.SetPropertyValue("ic/psi-true-deg", 90.0);   // heading (deg)
  fdm.SetPropertyValue("ic/theta-deg", 0.0);       // pitch attitude (deg)
  fdm.SetPropertyValue("ic/phi-deg",   0.0);       // roll attitude (deg)
  fdm.SetPropertyValue("ic/lat-gc-deg", 0.0);
  fdm.SetPropertyValue("ic/long-gc-deg", 0.0);


  // Initialize from those initial conditions without advancing time
  if (!fdm.RunIC()) 
  {
    print_error("JSBSimAircraft -> RunIC failed");
    return;
  }

  // ---- “Fly straight”: center surfaces, apply power ----
  // Normalized commands are typically -1..+1 for surfaces, 0..1 for throttle.
  // (The most robust way is also using the FGFCS API methods.)
  auto fcs = fdm.GetFCS();

  fcs->SetThrottleCmd(0, 0.65);   // engine 0 throttle command (0..1) :contentReference[oaicite:4]{index=4}
  // Center controls:
  fdm.SetPropertyValue("fcs/aileron-cmd-norm",  0.0);
  fdm.SetPropertyValue("fcs/elevator-cmd-norm", 0.0);
  fdm.SetPropertyValue("fcs/rudder-cmd-norm",   0.0);
}

String JSBSimAircraft::getRootDir() const noexcept
{
  return rootDir;
}

void JSBSimAircraft::setRootDir(const String &rootDir) noexcept
{
  this->rootDir = rootDir;
}

String JSBSimAircraft::getAircraft() const noexcept
{
  return aircraft;
}

void JSBSimAircraft::setAircraft(const String &aircraft) noexcept
{
  this->aircraft = aircraft;
}

double JSBSimAircraft::getSimulationHertz() const noexcept
{
    return simulationHertz;
}

void JSBSimAircraft::setSimulationHertz(const double &simulationHertz) noexcept
{
  this->simulationHertz = simulationHertz;
}

void JSBSimAircraft::_physics_process(double delta)
{
    if (!fdm.Run()) {
        print_error("JSBSim step failed");
        return;
    }

    double lat = fdm.GetPropertyValue("position/lat-gc-rad");
    double lon = fdm.GetPropertyValue("position/long-gc-rad");
    double alt_m = fdm.GetPropertyValue("position/h-sl-ft") * 0.3048;

    double roll  = fdm.GetPropertyValue("attitude/phi-rad");
    double pitch = fdm.GetPropertyValue("attitude/theta-rad");
    double yaw   = fdm.GetPropertyValue("attitude/psi-rad");

    // --- Set origin once ---
    if (!origin_set) {
        lat0 = lat;
        lon0 = lon;
        h0   = alt_m;
        ecef0 = lla_to_ecef(lat0, lon0, h0);
        origin_set = true;
    }

    Vec3d ecef = lla_to_ecef(lat, lon, alt_m);
    Vec3d enu  = ecef_to_enu(ecef, ecef0, lat0, lon0);

    set_position(Vector3(
        (float)enu.x,
        (float)enu.y,
        (float)-enu.z
    ));

    Quaternion q_godot = (Quaternion(Vector3(0, 1, 0), (float)(-yaw)) * 
                          Quaternion(Vector3(1, 0, 0), (float)( pitch)) * 
                          Quaternion(Vector3(0, 0, -1), (float)( roll))).normalized();

    Transform3D transform = get_transform();
    transform.origin = get_position();
    transform.basis = Basis(q_godot);
    set_transform(transform);
}

}