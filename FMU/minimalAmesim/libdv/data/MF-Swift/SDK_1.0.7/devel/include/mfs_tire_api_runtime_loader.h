/*******************************************************************************
 * (c)Copyright 2020 by Siemens Industry Software and Services B.V.
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

#ifndef MFS_TIRE_API_RUNTIME_LOADER_H
#define MFS_TIRE_API_RUNTIME_LOADER_H

#include "mfs_tire_api.h"

/** The main data structure containing function pointers for all API functions.
 *
 *  After the library has been loaded successfully, all available API functions
 *  will be mapped into this table. It is possible to use a newer version of the
 *  loader with previous versions of the library. In that case, some of the 
 *  functions will not be available (they will be NULL).
 *  The all_loaded flag will be set to MFS_API_TRUE if every function was
 *  mapped successfully. If it is set to MFS_API_FALSE, it is up to the user
 *  to verify that functions being called are valid.
 */
typedef struct MFS_API_FUNCTION_POINTERS
{
    mfs_api_get_major_version_t* mfs_api_get_major_version;
    mfs_api_get_major_version_t* mfs_api_get_minor_version;
    mfs_api_get_major_version_t* mfs_api_get_patch_version;

    mfs_api_get_maximum_state_size_t*          mfs_api_get_maximum_state_size;
    mfs_api_get_state_size_t*                  mfs_api_get_state_size;
    mfs_api_get_maximum_internal_state_size_t* mfs_api_get_maximum_internal_state_size;

    mfs_api_simulation_create_t*                mfs_api_simulation_create;
    mfs_api_simulation_create_with_logger_t*    mfs_api_simulation_create_with_logger;
    mfs_api_simulation_terminate_t*             mfs_api_simulation_terminate;
    mfs_api_simulation_set_log_handler_t*       mfs_api_simulation_set_log_handler;
    mfs_api_simulation_get_varinf_size_t*       mfs_api_simulation_get_varinf_size;
    mfs_api_simulation_load_entitlement_file_t* mfs_api_simulation_load_entitlement_file;
    mfs_api_simulation_load_entitlement_data_t* mfs_api_simulation_load_entitlement_data;

    mfs_api_tire_create_t*                            mfs_api_tire_create;
    mfs_api_tire_initialize_simulation_mode_t*        mfs_api_tire_initialize_simulation_mode;
    mfs_api_tire_initialize_road_type_t*              mfs_api_tire_initialize_road_type;
    mfs_api_tire_initialize_tire_side_t*              mfs_api_tire_initialize_tire_side;
    mfs_api_tire_initialize_contact_mode_t*           mfs_api_tire_initialize_contact_mode;
    mfs_api_tire_initialize_dynamics_mode_t*          mfs_api_tire_initialize_dynamics_mode;
    mfs_api_tire_initialize_magic_formula_mode_t*     mfs_api_tire_initialize_magic_formula_mode;
    mfs_api_tire_initialize_tire_property_file_t*     mfs_api_tire_initialize_tire_property_file;
    mfs_api_tire_initialize_tire_property_data_t*     mfs_api_tire_initialize_tire_property_data;

    mfs_api_tire_initialize_temperature_and_velocity_mode_t* mfs_api_tire_initialize_temperature_and_velocity_mode;

    mfs_api_tire_initialize_opencrg_file_paths_t*     mfs_api_tire_initialize_opencrg_file_paths;
    mfs_api_tire_initialize_external_road_callback_t* mfs_api_tire_initialize_external_road_callback;
    mfs_api_tire_initialize_solver_mode_t*            mfs_api_tire_initialize_solver_mode;
    mfs_api_tire_init_t*                              mfs_api_tire_init;
    mfs_api_pre_load_hardware_id_t*                   mfs_api_pre_load_hardware_id;

    mfs_api_tire_initialize_outputs_t*                   mfs_api_tire_initialize_outputs;
    mfs_api_tire_get_state_size_t*                       mfs_api_tire_get_state_size;
    mfs_api_tire_get_required_internal_state_size_t*     mfs_api_tire_get_required_internal_state_size;
    mfs_api_tire_set_input_t*                            mfs_api_tire_set_input;
    mfs_api_tire_stabilize_rigid_ring_t*                 mfs_api_tire_stabilize_rigid_ring;
    mfs_api_tire_set_rigid_ring_initialization_cycles_t* mfs_api_tire_set_rigid_ring_initialization_cycles;
    mfs_api_tire_get_initial_condition_t*                mfs_api_tire_get_initial_condition;

    mfs_api_tire_set_state_t*                            mfs_api_tire_set_state;
    mfs_api_tire_set_internal_state_t*                   mfs_api_tire_set_internal_state;
    mfs_api_tire_set_moving_road_t*                      mfs_api_tire_set_moving_road;
    mfs_api_tire_set_moving_road_friction_t*             mfs_api_tire_set_moving_road_friction;
    mfs_api_tire_set_moving_road_curvature_t*            mfs_api_tire_set_moving_road_curvature;
    mfs_api_tire_update_t*                               mfs_api_tire_update;
    mfs_api_tire_get_output_t*                           mfs_api_tire_get_output;
    mfs_api_tire_get_state_derivative_t*                 mfs_api_tire_get_state_derivative;
    mfs_api_tire_get_internal_state_t*                   mfs_api_tire_get_internal_state;
    mfs_api_tire_get_loaded_radius_t*                    mfs_api_tire_get_loaded_radius;
    mfs_api_tire_terminate_t*                            mfs_api_tire_terminate;

    mfs_api_tire_set_property_value_t* mfs_api_tire_set_property_value;
    mfs_api_tire_get_property_value_t* mfs_api_tire_get_property_value;
    /* If all functions have a valid mapping, the all_loaded flag will be MFS_API_TRUE. */
    MFS_API_BOOL all_loaded;
} MFS_API_FUNCTION_POINTERS;

/** The main data structure for the runtime loader */
typedef struct MFS_API_RUNTIME_LOADER MFS_API_RUNTIME_LOADER;

/** Load the tire library based on the default name and locations
 *
 *  The default locations are as follows (in order):
 *   1. In the directory of the shared object that is using the loader.
 *   2. Using the default system search paths
 *  If a valid ojbect (non-NULL) is returned, the tire library has been loaded
 *  and will remain in memory until mfs_api_runtime_loader_terminate is called.
 *  If the loader is unable to load the tire library, NULL will be returned
 *  Additionally, if the loaded library is not a tire library, NULL wil be returned as well
 */
MFS_API_RUNTIME_LOADER* mfs_api_runtime_loader_create(void);

/** Load the online version of the library based on the default name and locations
 *
 * This function works the same as mfs_api_runtime_loader_create, but loads the
 * 'online' version.
 */
MFS_API_RUNTIME_LOADER* mfs_api_runtime_loader_create_online(void);

/** Load the tire library using the provided path
 *
 * This version of create will try to load the tire library given by the exact path.
 */
MFS_API_RUNTIME_LOADER* mfs_api_runtime_loader_create_with_path(const char* library_path);

/** Obtain addresses to the API functions from a loaded tire library */
MFS_API_FUNCTION_POINTERS mfs_api_runtime_loader_get_function_pointers(const MFS_API_RUNTIME_LOADER* mfs_runtime_loader);

/** Unload the tire library and clean up the data structure
 *
 *  Calling this function invalidates all the function pointers. 
 */
void mfs_api_runtime_loader_terminate(MFS_API_RUNTIME_LOADER* mfs_runtime_loader);

#endif /* MFS_TIRE_API_RUNTIME_LOADER_H */
