#include "gdexample.h"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void GDExample::_bind_methods() {
}

GDExample::GDExample() 
{
	// Initialize any variables here.
	time_passed = 0.0;

    // ---- Initialize JSBSim ----
    fdm.SetRootDir(SGPath("/home/sanmopre/development/testJBSim/jsbsim"));
    fdm.SetAircraftPath(SGPath("aircraft"));
    fdm.SetEnginePath(SGPath("engine"));
    fdm.SetSystemsPath(SGPath("systems"));

    // ---- Load the aircraft model ----
    // This will look for: DataRoot/aircraft/c172p/c172p.xml (typical layout)
    if (!fdm.LoadModel("c172p")) {
        std::cerr << "Failed to load model\n";
        return;
    }

      // ---- Choose a fixed time step ----
  const double dt = 1.0 / 120.0; // 120 Hz physics
  fdm.Setdt(dt);                 // sets integration time step :contentReference[oaicite:2]{index=2}

  // ---- Initial conditions (example values) ----
  // Many ICs are set through the property system.
  fdm.SetPropertyValue("ic/h-sl-ft", 3000.0);      // altitude (ft)
  fdm.SetPropertyValue("ic/u-fps",   150.0);       // forward speed (ft/s) ~ 89 kts
  fdm.SetPropertyValue("ic/psi-true-deg", 90.0);   // heading (deg)
  fdm.SetPropertyValue("ic/theta-deg", 0.0);       // pitch attitude (deg)
  fdm.SetPropertyValue("ic/phi-deg",   0.0);       // roll attitude (deg)

  // Initialize from those initial conditions without advancing time
  if (!fdm.RunIC()) 
  {
    std::cerr << "RunIC failed\n";
  }

  // ---- “Fly straight”: center surfaces, apply power ----
  // Normalized commands are typically -1..+1 for surfaces, 0..1 for throttle.
  // (The most robust way is also using the FGFCS API methods.)
  auto& fcs = *fdm.GetFCS();

  fcs.SetThrottleCmd(0, 0.65);   // engine 0 throttle command (0..1) :contentReference[oaicite:4]{index=4}
  // Center controls:
  fdm.SetPropertyValue("fcs/aileron-cmd-norm",  0.0);
  fdm.SetPropertyValue("fcs/elevator-cmd-norm", 0.0);
  fdm.SetPropertyValue("fcs/rudder-cmd-norm",   0.0);


}

GDExample::~GDExample() {
	// Add your cleanup here.
}

void GDExample::_process(double delta) {
	time_passed += delta;

    if (!fdm.Run())
    {
        std::cerr << "Simulation step failed\n";
        return;
    };

    // Read back state (examples)
    double lat_rad = fdm.GetPropertyValue("position/lat-gc-rad");
    double lon_rad = fdm.GetPropertyValue("position/long-gc-rad");
    double alt_ft  = fdm.GetPropertyValue("position/h-sl-ft");

    double phi_deg   = fdm.GetPropertyValue("attitude/phi-deg");
    double theta_deg = fdm.GetPropertyValue("attitude/theta-deg");
    double psi_deg   = fdm.GetPropertyValue("attitude/psi-deg");


    std::cout << "alt_ft=" << alt_ft
            << " roll=" << phi_deg
            << " pitch=" << theta_deg
            << " hdg=" << psi_deg
            << "\n";
    
}