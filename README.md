# DeepThought

DeepThought is a fork of 42, a simulation framework developed by Eric Stoneking of NASA Goddard Space Flight Center. DeepThought seeks to enable the rapid developement and analysis for precision formation flying missions and the generalized Distributed System Mission (DSM) architecture. The primary contribution of this fork is with the 42dsm.c module, which adds additional guidance, navigation, and control functionality to DeepThought. Additionaly, it is recommended to utilze DSM_GUI in the generation of mission files.

Installation is similar to that of 42, and testing has only been preformed in the MacOSX and Linux environments. A script is included to run missions, with the following syntax:

```./run.sh mission_path output_path graphics_option```

where the `mission_path` is the folder containing the InOut directory, `output_path` is the folder name in `mission_path` to store the report data, and for `graphics_option`, the value `0` is graphics off and `1` is graphics on.

For example, runing the default `DSM_DEMO` mission, saving the data to the `sim_results` subdirectory of the `DSM_DEMO` mission directory, and enabling graphics is

```./run.sh Missions/DSM_DEMO sim_results 1```

# 42 - Spacecraft Simulation

42 is a comprehensive general-purpose simulation of spacecraft attitude and orbit dynamics. Its primary purpose is to support design and validation of attitude control systems, from concept studies through integration and test. 42 accurately models multi-body spacecraft attitude dynamics (with rigid and/or flexible bodies), and both two-body and three-body orbital flight regimes, modelling environments from low Earth orbit to throughout the solar system. 42 simulates multiple spacecraft concurrently, facilitating studies of rendezvous, proximity operations, and precision formation flying. It also features visualization of spacecraft attitude.

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
  - JPL SPICE ephemeris for accurate celestial body modelling (optional)

## Installation

DeepThought uses CMake for compilation; for the default release build, run `cmake --workflow --preset build-Release`. This will build DeepThought to the `build/Release` directory within the primary workspace directory. Additionally, the test suite can be run from `cmake --workflow --preset test-Release`. Additional cmake presets can be found in `CMakePresets.json`. 
If you wish to use `cmake` directly, there are various build flags noted in `CMakeLists.txt` and `Kit/CMakeLists.txt`. If you are attempting to build for Windows, good luck; there is some information in `Docs/Install-msys.txt`. The recommended Windows approach would be to use WSL on Windows 10/11.
To buid DeepThought, you require:
  - libfyaml >= 0.9
  - glut
  - make
  - cmake
  - gcc

If you have issues with Cmake, you can attempt to build using the legacy `Makefile.legacy`.

If you run DeepThought through the `run.sh` script, it will search for executable locations in this order:
  - system path (e.g., it will check if the command `deepthought` works)
  - primary workspace directory (`./` relative to the run.sh script)
  - `build/Release`
  - `build/RelWithDebInfo`
  - `build/Debug`

## Getting Started

See the overview, "42 Overview.pdf", in the Docs folder. Also recommended:

- Nomenclature.pdf
- POV Menu.pdf
- Key Bindings.txt
- FSW Models.pdf
- Flight Regimes.pdf

The default folder for inputs and outputs is "InOut". Look there for sample input files. "Inp_Sim.yaml" is the top-level input file.

The input/output folder may be changed for a sim run by running deepthought with flag/value syntax. Available flags are:
  - `-d`: Mission directory. This is the directory `deepthought` will search for the `InOut`, `Model` and other mission configuration directories and files.
  - `-o`: Output directory for data .
  - `-g`: Override graphics behavior in `Inp_Graphics.yaml`; enable graphics with `1`, disable with `0`.

## Planetary Ephemeris Settings (SPICE)
Deepthought is compatible with [SPICE](https://naif.jpl.nasa.gov/naif/), a toolkit used to read and process planetary ephemeris. We recommend a specific set of SPICE kernels to take advantage of all of 42's capabilities and are listed in ```Model/spice_kernels/get_kernels.sh``` (requires `wget`). Running this script will download the kernels and assemble a metakernel in ```Model/spice_kernels/kernels.txt```.

There are some use cases in which you need specialized or specific kernels that replace or add to the default SPICE kernels. In the case different [generic kernels](https://naif.jpl.nasa.gov/pub/naif/generic_kernels/) are desired, simply change the corresponding fields in ```Model/spice_kernels/get_kernels.sh``` and rerun the script; the script will handle downloading the kernels and assembling the new metakernel. If other kernels are desired, you will need to write a new metakernel at ```Model/spice_kernels/kernels.txt``` and download the desired kernels yourself. For more information on how metakernels are formatted and read, as well as other kernel formatting information, please see the [corresponding documentation](https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/C/req/kernel.html#Text%20Kernel%20Specifications) through JPL NAIF.

To build with JPL's SPICE toolkit, either run the `download_spice.sh` (you will require `wget`) script or manually download the correct version for your system and architecture from [HERE](https://naif.jpl.nasa.gov/naif/toolkit_C.html), and unzip it to the `deepthought` directory. If you wish to disable building with SPICE, set `-DSPICE=OFF` when running the `cmake` command.

## Common Problems

1) Deepthought expects the input files to be YAML with a particular format. If you are being given runtime errors about their format, view the input files for the supplied demonstration missions and the YAML file format comment descriptions in `yaml/yamlComments`.
