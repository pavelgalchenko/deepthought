# DeepThought

DeepThought is a fork of 42, a simulation framework developed by Eric Stoneking
of NASA Goddard Space Flight Center. DeepThought seeks to enable the rapid
developement and analysis for precision formation flying missions and the
generalized Distributed System Mission (DSM) architecture. The primary contribution
of this fork is with the 42dsm.c module, which adds additional guidance, navigation,
and control functionality to DeepThought. Additionaly, it is recommended to utilze
DSM_GUI in the generation of mission files.

Installation is similar to that of 42, and testing has only been preformed in
the MacOSX and Linux environments. A script is included to run missions, with
the following syntax:

```./run.sh mission_path output_path graphics_option```

where the `mission_path` is the folder containing the InOut directory,
`output_path` is the folder name in `mission_path` to store the report data,
and for `graphics_option`, the value `0` is graphics off and `1` is graphics on.

For exampke, runing the default DSM_DEMO mission

```./run.sh Missions/DSM_DEMO sim_results 1```

# 42 - Spacecraft Simulation

42 is a comprehensive general-purpose simulation of spacecraft attitude and
orbit dynamics. Its primary purpose is to support design and validation of
attitude control systems, from concept studies through integration and test. 42
accurately models multi-body spacecraft attitude dynamics (with rigid and/or
flexible bodies), and both two-body and three-body orbital flight regimes,
modelling environments from low Earth orbit to throughout the solar system. 42
simulates multiple spacecraft concurrently, facilitating studies of rendezvous,
proximity operations, and precision formation flying. It also features
visualization of spacecraft attitude.

Features:

  - Multi-body dynamics (tree topology, rotational and/or translational
    joints)
  - Rigid and/or flexible bodies
  - Multiple spacecraft (prox ops, formation flying, or independent)
  - Inter-spacecraft and spacecraft-surface contact forces support landers,
    rovers, and spacecraft servicing scenarios
  - Two-body or three-body orbits, anywhere in the solar system
  - Optional visualization
  - Socket-based interprocess comm (IPC) interface to other apps
  - Fast setup for concept studies
  - Rigorous and full-featured to support full spacecraft life cycle

## Installation

If you're installing on Windows, see the file "Install-msys.txt" in the Docs
folder.

The compiler will attempt to detect what platform you're on (Linux, OSX, or
Msys), but its success rate isn't great. If you have errors on the first
compile or run, try editing your Makefile to manually set your `42PLATFORM`.

For OpenGL graphics, newer Macs with Retina displays will need the GLFW graphics libraries, available from MacPorts, Homebrew, and probably elsewhere.  Otherwise, you'll need the GLUT libraries, which are also readily available if not already installed on your system.  Graphics are optional, settable in the Makefile by GUIFLAG.

## Getting Started

See the overview, "42 Overview.pdf", in the Docs folder. Also recommended:

- Nomenclature.pdf
- POV Menu.pdf
- Key Bindings.txt
- FSW Models.pdf
- Flight Regimes.pdf

The default folder for inputs and outputs is "InOut". Look there for sample
input files. "Inp_Sim.txt" is the top-level input file.

The input/output folder may be changed for a sim run by running 42 with an
argument. For example, enter this at the command prompt:

    42 Demo

## Common Problems

1) 42 expects the input files to be plain vanilla text files. If your text
editor adds formatting, makes straight quotes into smart quotes, etc, 42 may get
confused. The most common symptom is generating the "Bogus input in
DecodeString" error.

2) Also text-related, 42 is very simple-minded about reading the input files.
Adding extra lines, or accidentally deleting a line, or swapping lines, will
throw things out of synch. Again, the most common symptom is the "Bogus input in
DecodeString" error. Use your debugger to trace back where the error was
generated. (The actual mistake may be at that line, or may be somewhere
upstream.)
