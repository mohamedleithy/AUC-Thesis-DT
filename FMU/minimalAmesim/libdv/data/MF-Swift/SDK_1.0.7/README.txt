MF-Tyre/MF-Swift 2020.2 SDK
===========================

Overview
--------

This SDK provides all resources required to use the C interface to the
MF-Tyre/MF-Swift tire library. The SDK contains a few components
distributed over a few directories.

The 'devel' directory contains the C header files and libraries for
all supported platforms.

The 'examples' subdirectory contains a number of examples for
different use cases. A short description is provided in the 'Example
applications' section at the end of this README.

The 'externals' directory contains the required run-time libraries
needed to support the libraries.

The 'resources' directory contains utility applications to parse tir
property files and example tir files.

Currently, there is no separate documentation. The API is extensively
commented in the 'devel/include/mfs_tire_api.h' header. The examples
show practical usage scenarios. And this README contains an overview
of the licensing mechanisms in use and some information for users
that have been using the STI interface.


License Mechanism
-----------------

Currently, two license mechanisms are in place. One is FlexLM based,
the other one is an internal implementation using 'entitlements'.

With the exception of dSPACE platforms, all supported platforms use
FlexLM licensing. Using the library with FlexLM requires end users to
have a FlexLM license server running and to have the
MADLIC_LICENSE_FILE environment variable set on the host system to
point at the license server. Refer to the MF-Tyre/MF-Swift
installation manual for information on how to set this up.
FlexLM support is built into the library, so no additional steps are
required from an interface point of view.

On dSPACE platforms, file system support and network support are
typically unavailable. Therefore, traditional FlexLM licensing does not
work. On these platforms MF-Tyre/MF-Swift uses a proprietary licensing
mechanism that makes use of 'entitlement' files that users can obtain
through their Siemens sales representative.
From an interface point of view, the calling application is
responsible to provide the contents of the entitlement file by calling
the 'mfs_api_simulation_load_entitlement_data' function at
initialization.


Building against the MF-Tyre/MF-Swift library
---------------------------------------------

Applications using the MF-Tyre/MF-Swift library should include the
mfs_tire_api.h header, which in turn will include some other
supporting includes.

On Windows and Linux platforms, two methods can be used to use the
MF-Tyre/MF-Swift library:

* [Load-time linking] The calling application is directly linked
  against the library at build-time. On Linux, that means directly
  linking in the .so file, on Windows that means linking against the
  import .lib library (found in devel/lib). On Windows, the symbol
  'IMPORT_MFS_API' should be defined.

* [Run-time linking] In this case, the calling application is not
  linked directly against the MF-Tyre/MF-Swift library, but the
  library is loaded by the application after it has started. For this
  the LoadLibrary (Windows) and dlopen (Linux) functions are used. In
  this scenario, the mfs_tire_api.h header provides the prototypes of
  the functions that can be retrieved from the shared library.
  To facilitate run-time linking, the mfswift_tire_runtime_loader.lib
  library is provided (in devel/lib). This library automatically loads
  the mfswift_tire_interface library and maps all the functions into
  a structure that can be immediately used to call the api functions.


Converting from the old STI interface
-------------------------------------

The new MF-Tyre/MF-Swift API provides a more modern and cleaner C API
that will ensure long-term interface stability and easy extensibility
in the future. The 'single function' STI like interface has been
replaced by an object oriented interface. This section describes how
the former 'job flags' map to this new API.

* [jobflag = 1: INQRE] Initialize the logger and obtain varinf size
  - mfs_api_simulation_set_log_handler
  - mfs_api_simulation_get_varinf_size

* [jobflag = 2: INIT] Initialize a tire
  This call should be replaced by a call to mfs_api_tire_create to
  start initialization of a new tire, followed by the required
  mfs_api_tire_initialize_* calls, and a call to mfs_api_tire_init. At
  a minimum, the simulation mode (iswitch) needs to be set (either
  through mfs_api_tire_init, or through the separate functions), and
  the tire properties should be loaded
  (mfs_api_tire_initialize_tire_property_file or
  mfs_api_tire_initialize_tire_property_data).

* [jobflag = 6: INCON] Calculate initial condition for rigid-ring
  Currently, this is done only if needed in the first
  mfs_api_tire_update call. No explicit call is needed for this.

* [jobflag = 0: NORMAL] Perform an iteration of the model
  This call needs to be replaced by three new functions:
  - mfs_api_tire_set_input
  - mfs_api_tire_update
  - mfs_api_tire_get_output

* [jobflag = 99: ENSIM] Terminate the tires/simulation
  Each tire should be terminated with: mfs_api_tire_terminate.
  Once all tires are terminated and the simulation is completed,
  mfs_api_simulation_terminate should be called.

Solver modes
------------

In most use cases, the library will be used in co-simulation mode
with a built in solver. In this mode, the library uses an internal
RK4 integration scheme with a time step of 0.001s to advance the
model state each time step.

Some other use cases can benefit from more control over the integration
process. For those use cases it is also possible to run the library in 
external solver mode.

A note on simulation time
-------------------------

When running the library with the internal solver, the update rate is
fixed at 1000 Hz. The library can be called at any update rate, as long
as time is moving forward. If you need to step back in time, you will
need to use the state reset functionaity as described in the API and
demonstrated in the example: mfs_api_example_state_reset.c.

Internally, the library will use the provided simulation time to
determine if one or more solver steps are necessary. When the update
function is called at a rate higher than 1000Hz, the output of the 
previous step will be repeated for the sub steps.
When called at a lower rate, the internal solver will run as many
updates as needed to advance the state to the appropriate point in time.

Example applications
--------------------

The 'examples' directory contains a collection of simple example
applications that demonstrate various use cases that can be of
interest to users of this SDK. 

The most basic example is 'mfs_api_example_basic.c', which
demonstrates the smallest example to obtain a running simulation. All
other examples are slight variations on this example demonstrating a
specific technique or application.

The following examples should be compiled with the normal 
mfswift_tire_interface library:

* [mfs_api_example_external_road.c] An example on using external road
  callback functionality.

* [mfs_api_example_loaded_radius.c] An example demonstrating how to
  obtain the loaded radius of a tire before running the simulation.

* [mfs_api_example_multiple_tires.c] An example of running multiple
  tires in one simulation.

* [mfs_api_example_opencrg.c] An example of using the internal OpenCRG
  implementation.

* [mfs_api_example_split_config.c] An example of the alternative way
  to initialize the simulation mode of a tire, using multiple functions
  instead of a single iswitch combination.

* [mfs_api_example_state_reset.c] An example of how to reset the
  simulation to a specific state based on an earlier serialization of
  said state.

* [mfs_api_exmple_external_solver.c] An example on how to use the 
  library with an external solver.

This example is intended to be used with the mfswift_tire_runtime_loader:

* [mfs_api_example_runtime_loader.c] An example on how the support 
  library can be used to facilitate run-time loading.

Changelog
---------
1.0.7:  + Added missing functions in mfs_tire_api_runtime_loader

1.0.6:  + Added new function to set temperature and velocity model mode

1.0.5:  + Added support for the Pharlab PXI platform
        + Added the mfs_api_tire_stabilize_rigid_ring function to 
          re-initialize the internal state of the tire at any point
          during the simulation

1.0.4:  + Fixed an issue where mfs_api_tire_get_state_derivative wrote
          outside of the provided array if the size was less than the
          maximum state size

1.0.3:  + Building with -Bsymbolic to avoid interactions with FlexLM
        + Added mfs_simulation_create_with_logger to allow the log callback
          to be initialized immediately
        + Added mfs_api_runtime_loader_create_online to load the online
          version instead of the offline version

1.0.2:  + Moved some data structures from the stack to the heap to avoid
          crashes on Scalexio with external road

1.0.1:  + Added additional information on the usage of libraries though
          the LIBRARY_USAGE.txt file.
        + Fixed build configuration issue with IPG libraries
        
1.0.0:  + First official release of the SDK

0.0.10: + Added functionality to preload the dSPACE hardware id

0.0.9:  + Complete rename of 'tyre' to 'tire'. For compatibility reasons the
          old function names are still part of the binaries. It is advised
          to use the new names as soon as possible.
        + Renamed the header files and libraries to also use 'tire' instead
          of 'tyre'

0.0.8:  + Renamed the global state size functions to remove the simulation
          component from the name
        + Added a global function to obtain the exact state size based
          on a contact and dynamics mode without creating any simulation
          or tire object

0.0.7:  + Removed termination of tire object in case of init failure
        + Changed the moving road orientation matrix to column-major to
          achieve consistency with the input rotation matrix.
          
0.0.6:  + Fixed the mfs_api_tyre_get_property function
        + Tire properties in the INERTIA and DIMENSION section can now be
          obtained through mfs_api_tyre_get_property

0.0.5:  + Added the ability to force the internal solver to make a step by
          setting the input time to -1.0

0.0.4:  + Added functions to set friction and curvature of moving roads
        + mfs_api_tyre_get_property_value can now return the mass and inertias
          corrected for the dynamics mode

0.0.3:  + Added moving road contact
        + Updated description of mfs_api_tyre_get_initial_condition to reflect the
          proper default state of 'initial statics'

0.0.2:  + Added mfs_api_tyre_set_property_value and mfs_api_tyre_get_property_value
        + Renamed and clarified the input structure


This readme was generated for:
MF-Tyre/MF-Swift 2020.2 SDK 1.0.6 from hg rev 76724af552b6
