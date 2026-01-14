#pragma once

// Godot
#include "godot_cpp/core/class_db.hpp"

namespace godot
{

void initialize_jsbsim_module(ModuleInitializationLevel p_level);
void uninitialize_jsbsim_module(ModuleInitializationLevel p_level);

}
