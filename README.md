# DeepThought

DeepThought is a fork of 42, a simulation framework developed by Eric Stoneking of NASA Goddard Space Flight Center. DeepThought seeks to enable the rapid developement and analysis for precision formation flying missions and the generalized Distributed System Mission (DSM) architecture. The primary contribution of this fork is with the 42dsm.c module, which adds additional guidance, navigation, and control functionality to DeepThought. Additionaly, it is recommended to utilze DSM_GUI in the generation of mission files.

Installation is similar to that of 42, and testing has only been preformed in the MacOSX and Linux environments. A script is included to run missions, with the following syntax:

```./run.sh mission_path output_path graphics_option```

where the `mission_path` is the folder containing the InOut directory, `output_path` is the folder name in `mission_path` to store the report data, and for `graphics_option`, the value `0` is graphics off and `1` is graphics on.

For example, runing the default DSM_DEMO mission

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

DeepThought uses Cmake for compilation, so run `cmake .` and then `make`. There are various build flags noted in the `CMakeLists.txt`. If you are attempting to build for Windows, good luck; there is some information in `Docs/Install-msys.txt`. A more recommended approach would be to use WSL on Windows 10/11.
To buid DeepThought, you require:
  - libfyaml >= 0.9
  - glut
  - make
  - cmake
  - gcc

If you have issues with Cmake, you can attempt to build using the legacy `Makefile.legacy`.

## Getting Started

See the overview, "42 Overview.pdf", in the Docs folder. Also recommended:

- Nomenclature.pdf
- POV Menu.pdf
- Key Bindings.txt
- FSW Models.pdf
- Flight Regimes.pdf

The default folder for inputs and outputs is "InOut". Look there for sample input files. "Inp_Sim.yaml" is the top-level input file.

The input/output folder may be changed for a sim run by running 42 with an argument. For example, enter this at the command prompt:

    42 Demo

## Common Problems

1) Deepthought expects the input files to be YAML with a particular format. If you are being given runtime errors about their format, view the input files for the supplied demonstration missions and the YAML file format comment descriptions in `yaml/yamlComments`.

## Planetary Ephemeris Settings (SPICE)
Deepthought is compatible with [SPICE](https://naif.jpl.nasa.gov/naif/), a toolkit used to read and process planetary ephemeris. We recommend a specific set of SPICE kernels to take advantage of all of 42's capabilities; ```Model/spice_kernels/kernels.txt``` is 42's SPICE metakernel which lists all of the kernels you need for 42's basic functionality. We have provided a shell script (```Model/spice_kernels/get_kernels.sh```, requires `wget`) that automates the download process.

There are some use cases in which you need specialized or specific kernels that replace or add to the default SPICE kernels. In this case, you may replace or add the relevant line in the metakernel that points to the desired SPICE kernel. For more information on how metakernels are formatted and read, please see the [corresponding documentation](https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/C/req/kernel.html#Text%20Kernel%20Specifications) through JPL NAIF.

To build with JPL's SPICE toolkit, either run the `download_spice.sh` (you will require `wget`) script or manually download the correct version for your system and architecture from [HERE](https://naif.jpl.nasa.gov/naif/toolkit_C.html), and unzip it to the `deepthought` directory. If you wish to disable building with SPICE, set `SPICEFLAG` to empty either in the `Makefile` or with a command line override.
