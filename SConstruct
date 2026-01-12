#!/usr/bin/env python
import os
import sys

env = SConscript("submodules/godot-cpp/SConstruct")

env.Append(CPPPATH=["src/"])
env.Append(CPPPATH=["submodules/jsbsim/src"])

env.Append(LIBPATH=["build/jsbsim-release/src"])
env.Append(LIBS=["JSBSim"])

sources = Glob("src/*.cpp")

if env["platform"] == "macos":
    library = env.SharedLibrary(
        "demo/bin/libgdexample.{}.{}.framework/libgdexample.{}.{}".format(
            env["platform"], env["target"], env["platform"], env["target"]
        ),
        source=sources,
    )
else:
    library = env.SharedLibrary(
        "/home/sanmopre/development/godot_project/bin/libgdexample{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    )

Default(library)