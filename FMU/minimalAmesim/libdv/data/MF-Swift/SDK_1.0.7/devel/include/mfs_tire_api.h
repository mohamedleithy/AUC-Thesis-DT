/*******************************************************************************
 * (c)Copyright 2019 by Siemens Industry Software and Services B.V.
 * All rights reserved.
 *
 * Simcenter Tire(r) has been developed at Siemens Industry Software and 
 * Services B.V.
 *
 * This document contains proprietary and confidential information of Siemens.
 * The contents of this document may not be disclosed to third parties, copied 
 * or duplicated in any form, in whole or in part, without prior written
 * permission of Siemens.
 *
 * The terms and conditions governing the license of Simcenter Tire(r)
 * software consist solely of those set forth in the written contracts between
 * Siemens or Siemens authorized third parties and its customers. The software 
 * may only be used or copied in accordance with the terms of these contracts.
 *******************************************************************************/

#ifndef MFS_TIRE_API_H
#define MFS_TIRE_API_H

#include "MFS_API.h"

#include <stddef.h>

/*************************************************************************************************
 *                                MF-Tyre/MF-Swift API                                           *
 *************************************************************************************************
 * This header defines the public API of the MF-Tyre/MF-Swift library. Unless otherwise noted
 * all functions return a boolean success value. In most circumstances, an error log message 
 * is sent to the registered log callback before a function returns a failure value.
 * 
 * All functions that take an MFS_API_SIMULATION* or an MFS_API_TIRE* as their first argument
 * expect a valid object. All functions except the runtime functions perform a null pointer
 * check. The runtime functions perform no checks on the pointer and will crash if an invalid
 * object is passed.
 *************************************************************************************************/
typedef struct MFS_API_SIMULATION MFS_API_SIMULATION;
typedef struct MFS_API_ROAD       MFS_API_ROAD;
typedef struct MFS_API_TIRE       MFS_API_TIRE;

typedef int MFS_API_BOOL;
#define MFS_API_TRUE  1
#define MFS_API_FALSE 0

/*************************************************************************************************
 * Callback declarations                                                                         *
 ************************************************************************************************/

/* The type declaration for the log callback function
 *
 * A user defined function with this signature can be registered with the library
 * to be called whenever a log message is produced in the library. The 'message' 
 * argument contains a message that can or should be shown to the user, depending
 * on the severity of the message.
 *
 * Currently, there are four severity levels defined:
 * 0 - DEBUG:   These messages should typically not be shown to the user without specifically enabling 
 *              a debug flag. 
 * 1 - INFO:    These messages should be presented to the user in an easily accessible way (console or log 
 *              file). Among other things, this will show the library version to the user which will be 
 *              needed in case of support questions.
 * 2 - WARNING  Warnings should always be presented to the user in the most pertinent way possible. However,
 *              they do not require termination of the simulation.
 * 3 - ERROR    Errors should always be presented to the user in the most pertinent way possible. They will
 *              require termination of the application. The api function that triggered this message will 
 *              return a failure status.
 */
typedef void (MFS_CALL *mfs_api_log_callback_t)(int         severity,
                                                const char* message);

/* The MFS_API_ROAD_COORDINATE structure describes a point at which the library request road information
 * (height, friction, curvature) when using the external road interface.
 * The coordinates are expressed in the global coordinate frame. In the current implementation the z coordinate
 * is always 0. In the future, this will be set to a reference point on the wheel (the axle for example) in 
 * order to facilitate road methods that deal with multi level roads (overpasses, parking lots, etc.).
 */
typedef struct
{
    double x;
    double y;
    double z;
} MFS_API_ROAD_COORDINATE;

/* The MFS_API_ROAD_DATA structure contains optional variables that can be set for each point
 * next to the road height.
 * When the library calls the road callback, the road data fields are pre-filled with default
 * values (1 for friction, 0 for curvature), so when road methods can't provide either of these
 * attributes they don't have to be set.
 */
typedef struct
{
    double friction_x;
    double friction_y;
    double curvature;
} MFS_API_ROAD_DATA;

/* The mfs_road_callback_t defines the function signature for an external road callback.
 * 
 * When external road is selected and the appropriate callback function (using this signature)
 * is registered, the library will call the provided function any time it requires road height
 * information for a specific point.
 * 
 * The callback will be called with three input arguments:
 *   - time:                  The current simulation time
 *   - number_of_road_points: The amount of road points for which data are requested in the
 *                            current call. This value determines the size of the coordinates,
 *                            road_height, and road_data arrays.
 *                            In the current implementation, number_of_road_points is always
 *                            one. In the future this will change to allow 'bulk' calculations.
 *   - coordinates:           The x and y coordinates in the global reference frame at which
 *                            road height and road data are requested.
 *                            
 * Registered callbacks are required to at least write road height values for the provided
 * coordinates into the road_height array before returning. Writing additional data into
 * the road_data structures is optional.
 * 
 * During road registration, it is possible to register an additional pointer to an object
 * of your choice. Each time the callback is called, this custom data pointer is passed to
 * the callback as well. Any additional information required by the callback can be passed
 * this way. The custom data pointer is stored per tire, so each tire can have its own unique
 * custom data object. In this way, it is possible to use a single callback but still 
 * distinguish between calls from different tires. See the mfs_api_example_external_road.c 
 * example file for a simple example of how to use this mechanism.
 */
typedef MFS_API_BOOL (MFS_CALL *mfs_road_callback_t)(double                         time, 
                                                     size_t                         number_of_road_points,
                                                     const MFS_API_ROAD_COORDINATE* coordinates,
                                                     double*                        road_height,
                                                     MFS_API_ROAD_DATA*             road_data,
                                                     void*                          custom_data);
    
/*************************************************************************************************
 * API Version Related Functions                                                                 *
 *************************************************************************************************
 * These functions can be used to query the current version of the API
 *************************************************************************************************/
typedef int MFS_CALL mfs_api_get_major_version_t(void);
MFS_API mfs_api_get_major_version_t mfs_api_get_major_version;

typedef int MFS_CALL mfs_api_get_minor_version_t(void);
MFS_API mfs_api_get_major_version_t mfs_api_get_minor_version;

typedef int MFS_CALL mfs_api_get_patch_version_t(void);
MFS_API mfs_api_get_major_version_t mfs_api_get_patch_version;

/*************************************************************************************************
 * Enumerations                                                                                  *
 *************************************************************************************************/
typedef enum
{
    MFS_API_ROAD_TYPE_DEFAULT_FLAT = 1,
    MFS_API_ROAD_TYPE_OPENCRG      = 2,
    MFS_API_ROAD_TYPE_EXTERNAL     = 3
} MFS_API_ROAD_TYPE;

typedef enum
{
    MFS_API_TIRE_SIDE_LEFT      = 1,
    MFS_API_TIRE_SIDE_RIGHT     = 2,
    MFS_API_TIRE_SIDE_SYMMETRIC = 3,
    MFS_API_TIRE_SIDE_MIRROR    = 4
} MFS_API_TIRE_SIDE;

typedef enum
{
    MFS_API_CONTACT_MODE_SMOOTH_ROAD  = 1,
    MFS_API_CONTACT_MODE_MOVING_ROAD  = 3,
    MFS_API_CONTACT_MODE_ENVELOPING   = 5
} MFS_API_CONTACT_MODE;

typedef enum
{
    MFS_API_DYNAMICS_MODE_STEADY_STATE        = 0,
    MFS_API_DYNAMICS_MODE_TRANSIENT_LINEAR    = 1,
    MFS_API_DYNAMICS_MODE_TRANSIENT_NONLINEAR = 2,
    MFS_API_DYNAMICS_MODE_RIGID_RING          = 3
} MFS_API_DYNAMICS_MODE;

typedef enum
{
    MFS_API_MF_MODE_VERTICAL_LOAD           = 0,
    MFS_API_MF_MODE_LONGITUDINAL_LOADS      = 1,
    MFS_API_MF_MODE_LATERAL_LOADS           = 2,
    MFS_API_MF_MODE_UNCOMBINED_LOADS        = 3,
    MFS_API_MF_MODE_COMBINED_LOADS          = 4,
    MFS_API_MF_MODE_COMBINED_LOADS_TURNSLIP = 5
} MFS_API_MAGIC_FORMULA_MODE;

typedef enum
{
    MFS_API_TEMPERATURE_MODE_DISABLED           = 0,
    MFS_API_TEMPERATURE_MODE_STATIC             = 1,
    MFS_API_TEMPERATURE_MODE_DYNAMIC_NO_DPI     = 2,
    MFS_API_TEMPERATURE_MODE_DYNAMIC            = 3
} MFS_API_TEMPERATURE_MODE;

typedef enum
{
    MFS_API_SOLVER_MODE_INTERNAL = 0,
    MFS_API_SOLVER_MODE_EXTERNAL = 1
} MFS_API_SOLVER_MODE;

/*************************************************************************************************
 * Global functions                                                                              *
 *************************************************************************************************
 * These functions do not depend on either a simulation or a tire object
 *************************************************************************************************/

/** 
 * Pre-load the HIL hardware id on which this application is running.
 * Mainly intended for platforms that can not query the hardware id
 * in the initialization phase (Scalexio for example).
 * When this function is used, a runtime check will be performed
 * during the simuation step to verify that the hardware id provided
 * matches the hardware id of the board that is running the model.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_pre_load_hardware_id_t(const char* hardware_id);
MFS_API mfs_api_pre_load_hardware_id_t mfs_api_pre_load_hardware_id;

/** Obtain the maximum size of the tire state (the number of differential equations)
 *
 *  This function returns the maximum required array size to store the tire state. This method
 *  is mainly intended for packages that cannot wait until after tire creation to allocate
 *  storage for the state. The actual required state size depends on the mode the tire is running
 *  in and might be smaller. The actual size can be obtained by calling mfs_api_tire_get_state_size.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_get_maximum_state_size_t(size_t* state_size);
MFS_API mfs_api_get_maximum_state_size_t mfs_api_get_maximum_state_size;

/** Obtain the actual size of the tire state for a specific contact and dynamics mode
 *
 *  The recommended way of obtaining the actual state size is to initialize the tire object,
 *  and then call mfs_api_tire_get_state_size. 
 *  This function can be used if the actual size of the state is required before any simulation
 *  or tire objects are created. The contact mode and dynamics mode determine the state size.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_get_state_size_t(MFS_API_CONTACT_MODE contact_mode,
                                                       MFS_API_DYNAMICS_MODE dynamics_mode,
                                                       size_t* state_size);
MFS_API mfs_api_get_state_size_t mfs_api_get_state_size;

/** Returns the required size of a buffer that can hold the internal model state
 *
 *  This function returns the maximum required array size to store the internal tire state. This method
 *  is mainly intended for packages that cannot wait until after tire creation to allocate
 *  storage for the state. The actual required state size depends on the mode the tire is running
 *  in and might be smaller. The actual size can be obtained by calling mfs_api_tire_get_state_size.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_get_maximum_internal_state_size_t(size_t* internal_state_size);
MFS_API mfs_api_get_maximum_internal_state_size_t mfs_api_get_maximum_internal_state_size;

/*************************************************************************************************
 * Simulation Related Functions                                                                  *
 *************************************************************************************************
 * These functions handle settings and functionalities that apply to a complete simulation that 
 * can contain multiple tires.
 *************************************************************************************************/

/** Initialize the mfswift API by creating a global simulation object
 *
 * Create an object that tracks the global simulation state. Currently, only a single simulation
 * object can/should be created per process.
 */
typedef MFS_API_SIMULATION* MFS_CALL mfs_api_simulation_create_t(void);
MFS_API mfs_api_simulation_create_t mfs_api_simulation_create;

/** Initialize the mfswift API by creating a global simulation object
 *
 * Similar as the above function, excepts this version immediately registers the log handler.
 */
typedef MFS_API_SIMULATION* MFS_CALL mfs_api_simulation_create_with_logger_t(mfs_api_log_callback_t logger);
MFS_API mfs_api_simulation_create_with_logger_t mfs_api_simulation_create_with_logger;

/** Finalize the simulation and the API
 *
 * This releases all remaining data structures and allocated memory. The mfs_simulation object
 * does not keep track of any mfs_tire objects. Before calling this method, all tires should
 * have been terminated.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_simulation_terminate_t(MFS_API_SIMULATION* mfs_simulation);
MFS_API mfs_api_simulation_terminate_t mfs_api_simulation_terminate;

/** Register a log callback
 *
 * Calling this function is optional. If no log callback is set, no log messages will
 * be produced. Passing a null pointer as the log callback is allowed and also
 * gives the same behavior as if no logger was set. 
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_simulation_set_log_handler_t(MFS_API_SIMULATION*    mfs_simulation, 
                                                                   mfs_api_log_callback_t logger);
MFS_API mfs_api_simulation_set_log_handler_t mfs_api_simulation_set_log_handler;

/** Obtain the number of entries in the varinf output array
 *
 * The varinf array provides additional outputs next to the forces and moments. The number
 * of entries can increase in future releases. 
 * It is up to the client code to decide whether to use a dynamically sized array based on the
 * size returned by this function, or to use a fixed size array and use this function to check
 * how many entries contain valid results. Please refer to the user manual for a complete description
 * of all entries in the varinf array.
 *
 * When a custom array is used that is larger than varinf_size, at most varinf_size elements are 
 * written into the array, the remainder is zeroed out.
 * When a custom array is used that is smaller than varinf_size, the library writes no more than
 * this custom size. 
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_simulation_get_varinf_size_t(MFS_API_SIMULATION* mfs_simulation,
                                                                   size_t*             varinf_size);
MFS_API mfs_api_simulation_get_varinf_size_t mfs_api_simulation_get_varinf_size;


/** Load a payload file containing an entitlement
 *
 * The library reads the contents of the file pointed to by entitlement_file_path and
 * processes it accordingly. See the license manual for more information on entitlements.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_simulation_load_entitlement_file_t(MFS_API_SIMULATION* mfs_simulation, 
                                                                         const char*         entitlement_file_path);
MFS_API mfs_api_simulation_load_entitlement_file_t mfs_api_simulation_load_entitlement_file;

/** Load a buffer that holds a payload containing an entitlement file
 *
 * The library parses the entitlement data and process it accordingly. See the license manual 
 * for more information on entitlements.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_simulation_load_entitlement_data_t(MFS_API_SIMULATION* mfs_simulation, 
                                                                         const char*         entitlement_data,
                                                                         size_t              entitlement_data_size);
MFS_API mfs_api_simulation_load_entitlement_data_t mfs_api_simulation_load_entitlement_data;


/*************************************************************************************************
 * Tire Initialization Related Functions                                                         *
 *************************************************************************************************
 * These functions are used to set per-tire options. All functions except mfs_api_tire_init can
 * be called multiple times. Calling a function multiple times will overwrite the previous setting
 * with the new setting. Once mfs_api_tire_init has been called, any mfs_api_tire_initialize_* 
 * calls have no effect.
 *************************************************************************************************/
/** Create a new tire object
 *
 * The tire object represents a single tire with its settings in the simulation. All tire objects
 * are self contained, which makes it possible to create, initialize and run tires in multiple
 * (parallel) threads within a single process.
 */
typedef MFS_API_TIRE* MFS_CALL mfs_api_tire_create_t(MFS_API_SIMULATION* mfs_simulation);
MFS_API mfs_api_tire_create_t mfs_api_tire_create;

/** Set the use mode of the simulation
 * 
 * Set the complete use mode of the current tire with the legacy ISWITCH mechanism. See the 
 * definition of the ISWITCH parameter in section 2.6 of the User Manual.
 * The recommended way of setting the use mode is by using the dedicated functions that
 * set specific attributes individually.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_initialize_simulation_mode_t(MFS_API_TIRE* mfs_tire,
                                                                        int           iswitch);
MFS_API mfs_api_tire_initialize_simulation_mode_t mfs_api_tire_initialize_simulation_mode;

/** Set the road mode for the tire 
 *
 * See section 2.1 of the User Manual for a detailed explanation of the different options.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_initialize_road_type_t(MFS_API_TIRE*     mfs_tire, 
                                                                  MFS_API_ROAD_TYPE road_type);
MFS_API mfs_api_tire_initialize_road_type_t mfs_api_tire_initialize_road_type;

/** Set the tire side for the tire
 *
 * See section 2.2 of the User Manual for a detailed explanation of the different options.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_initialize_tire_side_t(MFS_API_TIRE*     mfs_tire, 
                                                                  MFS_API_TIRE_SIDE tire_side);
MFS_API mfs_api_tire_initialize_tire_side_t mfs_api_tire_initialize_tire_side;

/** Set the contact mode for the tire
 *
 * See section 2.3 of the User Manual for a detailed explanation of the different options.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_initialize_contact_mode_t(MFS_API_TIRE*        mfs_tire, 
                                                                     MFS_API_CONTACT_MODE contact_mode);
MFS_API mfs_api_tire_initialize_contact_mode_t mfs_api_tire_initialize_contact_mode;

/** Set the dynamics mode for the tire 
 *
 * See section 2.4 of the User Manual for a detailed explanation of the different options.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_initialize_dynamics_mode_t(MFS_API_TIRE*         mfs_tire,
                                                                      MFS_API_DYNAMICS_MODE dynamics_mode);
MFS_API mfs_api_tire_initialize_dynamics_mode_t mfs_api_tire_initialize_dynamics_mode;

/** Set output mode for the tire 
 *
 * See section 2.5 of the User Manual for a detailed explanation of the different options.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_initialize_magic_formula_mode_t(MFS_API_TIRE*              mfs_tire,
                                                                           MFS_API_MAGIC_FORMULA_MODE mf_mode);
MFS_API mfs_api_tire_initialize_magic_formula_mode_t mfs_api_tire_initialize_magic_formula_mode;

/** Set the path to the tire property file
 *
 * The function accepts either full or relative paths to a tire property (tir) file. 
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_initialize_tire_property_file_t(MFS_API_TIRE* mfs_tire, 
                                                                           const char*   tir_file_path);
MFS_API mfs_api_tire_initialize_tire_property_file_t mfs_api_tire_initialize_tire_property_file;

/** Pass a parsed tire property file
 *
 * This function accepts an array containing the preprocessed contents of a tire property
 * file obtained through the mfswift tirparser.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_initialize_tire_property_data_t(MFS_API_TIRE* mfs_tire, 
                                                                           const double* tir_file_data,
                                                                           size_t        tir_file_data_size);
MFS_API mfs_api_tire_initialize_tire_property_data_t mfs_api_tire_initialize_tire_property_data;


/** Set the temperature and velocity model mode
 *
 *  See the user manual for T&V model details
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_initialize_temperature_and_velocity_mode_t(MFS_API_TIRE*            mfs_tire,
                                                                                      MFS_API_TEMPERATURE_MODE tv_mode);
MFS_API mfs_api_tire_initialize_temperature_and_velocity_mode_t mfs_api_tire_initialize_temperature_and_velocity_mode;

/** Pass the paths of the OpenCRG input files 
 *
 * Pass the road data files for this tire. The OpenCRG height file is required, the friction file 
 * is optional. If no friction data are available or needed, either a null pointer or an empty string
 * can be passed.
 * Calling this function is required when the road type is set to OpenCRG [MFS_API_ROAD_TYPE_OPENCRG]
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_initialize_opencrg_file_paths_t(MFS_API_TIRE* mfs_tire, 
                                                                           const char*   opencrg_height_file_path,
                                                                           const char*   opencrg_friction_file_path);
MFS_API mfs_api_tire_initialize_opencrg_file_paths_t mfs_api_tire_initialize_opencrg_file_paths;

/** Set the external road callback
 * 
 * Register the callback function that will be called whenever the library requires road information.
 * A call to this function is required when the road type is set to External [MFS_API_ROAD_TYPE_EXTERNAL].
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_initialize_external_road_callback_t(MFS_API_TIRE*       mfs_tire,
                                                                               mfs_road_callback_t road_callback,
                                                                               size_t              road_n_points_max,
                                                                               void*               user_data);
MFS_API mfs_api_tire_initialize_external_road_callback_t mfs_api_tire_initialize_external_road_callback;

/** Set the solver mode
 * 
 * The tire model can either integrate the tire state for each time step (fixed at 0.001s) or expose the state
 * and its derivatives which can then be integrated outside of the tire model.
 * MFS_API_SOLVER_MODE_INTERNAL is used by default when this function is not called.
 * When MFS_API_SOLVER_MODE_EXTERNAL is selected, the tire state should be passed to the model with
 * mfs_api_tire_set_state before calling mfs_api_tire_update. After the update, the derivatives can
 * be obtained by calling mfs_api_tire_get_state_derivative.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_initialize_solver_mode_t(MFS_API_TIRE*       mfs_tire,
                                                                    MFS_API_SOLVER_MODE solver_mode);
MFS_API mfs_api_tire_initialize_solver_mode_t mfs_api_tire_initialize_solver_mode;

/** Initialize the current tire
 *
 * Initialize the tire based on the settings that have been set by using the mfs_api_tire_intialize_*
 * functions. This brings the tire into a runnable state if successful. If this function fails,
 * the reason of the failure has been logged through the log callback. The created tire is not yet
 * terminated. The initialization parameters can be revised and this function can be called again.
 *
 * This function can be successfully called once per tire, after the call any calls to
 * mfs_api_tire_intialize_* has no effect. At the end of the simulation mfs_api_tire_terminate
 * should be called to terminated the tire object.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_init_t(MFS_API_TIRE* mfs_tire);
MFS_API mfs_api_tire_init_t mfs_api_tire_init;

/*************************************************************************************************
 * Runtime Tire Related Functions                                                                *
 *************************************************************************************************
 * These functions are used at runtime to manage the input/update/output cycle of the model.
 * All functions require a fully initialized MFS_TIRE object (mfs_api_tire_init was successful),
 * however, to avoid run-time overhead no runtime checks are performed on the MFS_TIRE object.
 * It is the caller's responsibility to ensure a valid object is passed.
 *************************************************************************************************/
/** Initialize the force and torque output values
 *
 * Calling this function is optional, but it can be used to 'prime' the internal force and torque
 * outputs. This can be necessary when for example the loaded radius needs to be obtained before
 * the first model update.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_initialize_outputs_t(MFS_API_TIRE* mfs_tire,
                                                                const double  force[3],
                                                                const double  torque[3]);
MFS_API mfs_api_tire_initialize_outputs_t mfs_api_tire_initialize_outputs;

/** Obtain the size of the tire state (the number of differential equations)
 *
 * This function returns the number of elements in the tire state and state derivative. Arrays
 * passed to mfs_api_tire_set_state and mfs_api_tire_get_state_derivative should have a size
 * of at least the size returned by this function.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_get_state_size_t(MFS_API_TIRE* mfs_tire,
                                                            size_t*       state_size);
MFS_API mfs_api_tire_get_state_size_t mfs_api_tire_get_state_size;

/** Returns the required size of a buffer that can hold the internal model state
 *
 * Some simulation modes require information to be persistent between simulation
 * steps (similar to 'work' arrays). 
 * This size can be used to construct a buffer used in mfs_api_tire_get_state
 * and mfs_api_tire_set_state to save and reset a specific state for the tire
 * At least the iswitch should have been set on this tire
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_get_required_internal_state_size_t(MFS_API_TIRE* mfs_tire,
                                                                              size_t*       size);
MFS_API mfs_api_tire_get_required_internal_state_size_t mfs_api_tire_get_required_internal_state_size;

/** The structure containing all required inputs for the tire model
 *
 * The inputs follow to the default STI conventions. The G postfix indicates a vector expressed
 * in the global reference frame. The WC postfix indicates a vector expressed in the wheel 
 * carrier reference frame. The legacy names of the parameters can be found at the end of the
 * description.
 * 
 * The inputs can be specified in two ways:
 *   - non-rotating wheel carrier: when specifying a wheel_angle and/or a wheel_angular_velocity
 *                                 WC_to_G is treated as a non-rotating frame.
 *   - rotating wheel carrier: when keeping wheel_angle and wheel_angular_velocity at 0, WC_to_G
 *                             is treated as a rotating frame and should describe the complete
 *                             rotation of the tire.
 *                             
 * The WC_to_G_transformation matrix is treated as a column-major matrix.
 */
typedef struct
{   
    double time;                                 /**< Simulation time [s] */    
    double wheel_carrier_position_G[3];          /**< Position of the wheel carrier at the wheel center in the global frame            [m]     (dis)    */    
    double wheel_carrier_velocity_WC[3];         /**< Global velocity of the wheel carrier in the wheel carrier frame                  [m/s]   (vel)    */   
    double wheel_carrier_angular_velocity_WC[3]; /**< Global angular velocity vector of the wheel carrier in the wheel carrier frame   [rad/s] (omega)  */  
    double WC_to_G_transformation[9];            /**< Transformation matrix from wheel carrier frame to global frame                   [-]     (tramat) */    
    double wheel_angle;                          /**< Rotation angle of the wheel wrt the wheel carrier about the spin axis            [rad]   (angtwc) */  
    double wheel_angular_velocity;               /**< Relative angular velocity of the wheel wrt the wheel carrier about the spin axis [rad/s] (omegar) */
} MFS_INPUT_DATA;

/** Set the inputs of the model in preparation of the next time step 
 *
 * Can be called multiple times, each call overwrites the previous values.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_set_input_t(MFS_API_TIRE*         mfs_tire,
                                                       const MFS_INPUT_DATA* mfs_input);
MFS_API mfs_api_tire_set_input_t mfs_api_tire_set_input;

/** Adjust the number of cycles used for the rigid ring initialization routine
 *
 * During the call to mfs_api_tire_get_initial_condition, the rigid ring state can be initialized
 * by running a pre-simulation. This avoids unsteady transient behavior at the beginning of 
 * a simulation. By default, 0 cycles are used, which means that the pre-simulation is disabled.
 * To enable the pre-simulation, it is recommended to set n_cycles to 5000. 
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_set_rigid_ring_initialization_cycles_t(MFS_API_TIRE* mfs_tire,
                                                                                  int           n_cycles);
MFS_API mfs_api_tire_set_rigid_ring_initialization_cycles_t mfs_api_tire_set_rigid_ring_initialization_cycles;

/** Obtain the initial state of the model
 *
 * This function will return the initial state and initial internal state of the tire model 
 * during initialization. Tire initialization with rigid ring initial statics is a presimulation 
 * which therefore makes road calls.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_get_initial_condition_t(MFS_API_TIRE* mfs_tire,
                                                                   double*       initial_state,
                                                                   size_t        initial_state_size,
                                                                   void*         internal_state,
                                                                   size_t        internal_state_size);
MFS_API mfs_api_tire_get_initial_condition_t mfs_api_tire_get_initial_condition;

/** Re-run the rigid ring initial statics algorithm to find a new equilibrium
 *
 * This function is only relevant for use with the internal solver. Before calling this
 * function, the required  input should be set with a cal to mfs_api_tire_set_input.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_stabilize_rigid_ring_t(MFS_API_TIRE* mfs_tire);
MFS_API mfs_api_tire_stabilize_rigid_ring_t mfs_api_tire_stabilize_rigid_ring;

/** Set the tire state in preparation of the next update
 *
 * Can be called multiple times, each call overwrites the previous values. This is only
 * needed when using external solver mode.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_set_state_t(MFS_API_TIRE* mfs_tire,
                                                       const double* deqvar,
                                                       size_t        deqvar_size);
MFS_API mfs_api_tire_set_state_t mfs_api_tire_set_state;

/** Set the internal state of the current tire
 *
 * When using the internal solver, this function resets the complete model to a previous state. 
 * When using the external solver, this call sets the internal (persistent) state.
 * The internal state should have been obtained through a call to mfs_api_tire_get_state.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_set_internal_state_t(MFS_API_TIRE* mfs_tire, 
                                                                const void*   state_data, 
                                                                size_t        state_data_size);
MFS_API mfs_api_tire_set_internal_state_t mfs_api_tire_set_internal_state;

/** Set the state of the moving road for the next update
 *
 * The position, velocity, and angular_velocity specify the location and orientation
 * of the ground plane in the global reference system.
 * The orientation is specified as a rotation matrix that transforms vectors
 * from the road frame to the global frame. The matrix is in column-major order.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_set_moving_road_t(MFS_API_TIRE* mfs_tire,
                                                             const double position[3],
                                                             const double velocity[3],
                                                             const double angular_velocity[3],
                                                             const double orientation[9]);
MFS_API mfs_api_tire_set_moving_road_t mfs_api_tire_set_moving_road;

/** Set the road friction for the moving road
 *
 *  By default, both friction values are 1.0. This function call is only needed when
 *  a different friction value is required.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_set_moving_road_friction_t(MFS_API_TIRE* mfs_tire,
                                                                      double friction_x,
                                                                      double friction_y);
MFS_API mfs_api_tire_set_moving_road_friction_t mfs_api_tire_set_moving_road_friction;

/** Set the road curvature for the moving road
 *
 *  By default, the curvature is 0.0. This function call is only needed when
 *  a different curvature value is required.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_set_moving_road_curvature_t(MFS_API_TIRE* mfs_tire,
                                                                       double curvature);
MFS_API mfs_api_tire_set_moving_road_curvature_t mfs_api_tire_set_moving_road_curvature;

/** Calculate a new model state based on the input set through mfs_api_tire_set_input
 *
 * When the external solver is used, the forces and moments are calculated based on 
 * the current inputs and state. Based on this, the state derivatives are calculated.
 * 
 * When the internal solver is used, the tire forces and moments are calculated for
 * the current inputs and the internal state is advanced in preparation of the next
 * iteration. Negative input times are not accepted (with one exception, noted at 
 * the end)
 * Input time can not move backwards for updates. If this is needed, use the state
 * reset functionality.
 * If the input time has advanced less than 1ms, the output of the previous step
 * is repeated. If the input time advances in a step larger than 1ms, the library
 * internal solver will step forward as much as needed to provide the correct outputs
 * for that time point.
 * 
 * Some applications need to be able to advance the state without moving the simulation
 * time forward (e.g. to find a stable initial condition). To force the model to
 * perform a step, set the input time to -1.0 and call update. 
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_update_t(MFS_API_TIRE* mfs_tire);
MFS_API mfs_api_tire_update_t mfs_api_tire_update;

/** Obtain the model outputs
 *
 * Calling this function multiple times always results in the same values
 * until mfs_api_tire_update is called again.
 *
 * varinf_size should be set to the number of entries available in the varinf array
 * passed into this function. 
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_get_output_t(MFS_API_TIRE* mfs_tire, 
                                                        double        force[3], 
                                                        double        torque[3], 
                                                        double*       varinf,
                                                        size_t        varinf_size);
MFS_API mfs_api_tire_get_output_t mfs_api_tire_get_output;

/** Obtain the state derivative corresponding to the last update call
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_get_state_derivative_t(MFS_API_TIRE* mfs_tire,
                                                                  double*       deqder,
                                                                  size_t        deqder_size);
MFS_API mfs_api_tire_get_state_derivative_t mfs_api_tire_get_state_derivative;

/**  Copy the internal state to the provided buffer
 *
 * The size of the buffer should be at least the size returned by mfs_api_get_required_tire_state_size.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_get_internal_state_t(MFS_API_TIRE* mfs_tire, 
                                                                void*         state_data, 
                                                                size_t        state_data_size);
MFS_API mfs_api_tire_get_internal_state_t mfs_api_tire_get_internal_state;

/** Calculate the loaded radius
 *
 * This function is typically used to obtain the loaded radius before the first call to
 * mfs_api_tire_update. Before this function can be called, omega[y] should be set through
 * mfs_api_tire_set_input and the forces should be initialized by a call to 
 * mfs_api_tire_initialize_outputs.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_get_loaded_radius_t(MFS_API_TIRE* mfs_tire, 
                                                               double*       loaded_radius);
MFS_API mfs_api_tire_get_loaded_radius_t mfs_api_tire_get_loaded_radius;

/** Clean up the tire instance at the end of the simulation
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_terminate_t(MFS_API_TIRE* mfs_tire);
MFS_API mfs_api_tire_terminate_t mfs_api_tire_terminate;

/*************************************************************************************************
 * Runtime Tire Related Functions                                                                *
 *************************************************************************************************/
/** Set a property value of the tire
 *
 * Currently only "tire_properties/OPERATING_CONDITIONS/INFLPRES" is supported.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_set_property_value_t(MFS_API_TIRE* mfs_tire, const char* property_name, double property_value);
MFS_API mfs_api_tire_set_property_value_t mfs_api_tire_set_property_value;

/** Obtain a property value of the tire
 *
 * This function provides access to a number of tire properties and derived properties. The values
 * for these properties can only be obtained after the tire has been initialized since they can
 * depend on the estimation of certain parameters
 * 
 * Currently the tire properties in the DIMENSION and INERTIA sections can be requested. For example:
 *  - "tire_properties/INERTIA/MASS"
 *  - "tire_properties/INERTIA/BELT_MASS"
 *  - "tire_properties/DIMENSION/UNLOADED_RADIUS"
 *
 * Currently three derived mass related properties are supported:
 *  - "derived_properties/mbs_mass"
 *  - "derived_properties/mbs_Ixx"
 *  - "derived_properties/mbs_Iyy"
 *  The values returned reflect the mass properties of the tire that need to be used by the MBS
 *  package in its calculations. In non-rigid ring simulations, this will be the complete mass and
 *  inertia. In rigid-ring simulations these values will represent the mass and inertia without the
 *  contribution of the belt mass and belt inertia.
 */
typedef MFS_API_BOOL MFS_CALL mfs_api_tire_get_property_value_t(MFS_API_TIRE* mfs_tire, const char* property_name, double* property_value);
MFS_API mfs_api_tire_get_property_value_t mfs_api_tire_get_property_value;

#endif /* MFS_TIRE_API_H */
