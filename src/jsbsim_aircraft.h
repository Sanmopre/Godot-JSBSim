#pragma once

// Godot
#include "godot_cpp/classes/node3d.hpp"

// JSBSim
#include "FGFDMExec.h"
#include "models/FGFCS.h"

namespace godot 
{

struct Vec3d {
double x, y, z;
};

class JSBSimAircraft : public Node3D 
{
	GDCLASS(JSBSimAircraft, Node3D)

protected:
	static void _bind_methods();

public:
	JSBSimAircraft() = default;
	~JSBSimAircraft() = default;

public:
	// Implements Node functions
	void _ready() override;
	void _physics_process(double delta) override;

public:
	// Getter and setter functions
	[[nodiscard]] String getRootDir() const noexcept;
	void setRootDir(const String& rootDir) noexcept;

	[[nodiscard]] String getAircraft() const noexcept;
	void setAircraft(const String& aircraft) noexcept;

	[[nodiscard]] double getSimulationHertz() const noexcept;
	void setSimulationHertz(const double& simulationHertz) noexcept;

	[[nodiscard]] double getAileronCommand() const noexcept;
	void setAileronCommand(const double& command) noexcept;

	[[nodiscard]] double getElevatorCommand() const noexcept;
	void setElevatorCommand(const double& command) noexcept;

	[[nodiscard]] double getRudderCommand() const noexcept;
	void setRudderCommand(const double& command) noexcept;

	[[nodiscard]] double getThrottleCommand() const noexcept;
	void setThrottleCommand(const double& command) noexcept;

	[[nodiscard]] double getYOffset() const noexcept;
	void setYOffset(const double& yOffset) noexcept;

	[[nodiscard]] double getTerrainElevationMeters() const noexcept;
	void setTerrainElevationMeters(const double& elevationMeters) noexcept;

private:
	// JSBSim properties
    JSBSim::FGFDMExec fdm;

private:
	String rootDir;
	String aircraft;
	double simulationHertz;
	double yOffset;

private:
	// Controls
	double comandedAileron = 0.0;
	double commandedElevator = 0.0;
	double commandedRudder = 0.0;
	double commandedThrottle = 0.0;

private:
	bool origin_set = false;
	double lat0 = 0.0;
	double lon0 = 0.0;
	double h0   = 0.0;
	Vec3d ecef0;

private:
	double terrainElevationMeters = 0.0;
};

}