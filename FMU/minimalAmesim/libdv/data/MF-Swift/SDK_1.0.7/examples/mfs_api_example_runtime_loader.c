#include <mfs_tire_api_runtime_loader.h>

#include <stdio.h>

static const char* tir_file = "Siemens_car205_60R15.tir";

static const double dt = 0.001; /* [s] */

#define VARINF_SIZE 60

void MFS_CALL simple_logger(int severity, const char* message)
{
    printf("severity: %d\n", severity);
    printf("msg: %s\n", message);
}

int main()
{  
    MFS_API_TIRE* tire1 = NULL;
    MFS_API_SIMULATION* simulation = NULL;

    MFS_API_FUNCTION_POINTERS fp;

    /* Creating a runtime_loader object with the 'default' create function will first try
     * to locate the tire dll (using the default name) at the same location as the
     * current shared library or executable. If that fails, it will use the normal 
     * operating system mechanism to locate the tire library using the default name.
     * 
     * The second variant of the create function (mfs_api_runtime_loader_create_with_path)
     * can be used to explicitly specify the full path to the tire library (include the
     * file name). This can be useful if your user is able to select his preferred tire
     * library through a GUI.
     * 
     * If a pointer is returned, the tire library has been loaded successfully. A NULL
     * pointer is returned in three cases:
     *      - The library was not found
     *      - The library could not be loaded properly
     *      - The library that was loaded is not a valid tire library
     */
    MFS_API_RUNTIME_LOADER* runtime_loader = mfs_api_runtime_loader_create();

    if (!runtime_loader)
    {
        return 1;
    }

    /* Once the library has been loaded correctly, the API functions can be mapped
     * into a MFS_API_FUNCTION_POINTERS object */
    fp = mfs_api_runtime_loader_get_function_pointers(runtime_loader);

    /* Next to all the API function pointers, the MFS_API_FUNCTION_POINTERS object 
     * contains a boolean flag to indicate that all functions were loaded properly.
     * When the flag is MFS_API_TRUE, you are able to call any function without having
     * to check the validity of the pointer first.
     * When the flag equals MFS_API_FALSE, it indicates that one or more functions 
     * were not loaded. This mainly happens when an 'older' version of the tire library
     * is used. In this situation, it is up to the user of the API to ensure that the
     * functions required for proper operation are available.
     * In this example, we decide to simply terminate the application if not all 
     * functions were loaded.
     */
    if (!fp.all_loaded)
    {
        mfs_api_runtime_loader_terminate(runtime_loader);
        return 1;
    }

    /* When linking the shared library during compilation, the functions can be called
     *  directly. The main difference when using the runtime loader is that now all the 
     *  functions are 'stored' in the structure, which means all function calls are prefixed
     *  with the function table.
     */
    simulation = fp.mfs_api_simulation_create();
    fp.mfs_api_simulation_set_log_handler(simulation, simple_logger);

    tire1 = fp.mfs_api_tire_create(simulation);

    fp.mfs_api_tire_initialize_simulation_mode(tire1, 11113);
    fp.mfs_api_tire_initialize_tire_property_file(tire1, tir_file);

    if(fp.mfs_api_tire_init(tire1))
    {
        int i = 0;

        MFS_INPUT_DATA input_data = {
            0,                 /* time                              [s]              */
            { 0.0, 0.0, 0.29}, /* wheel_carrier_position_G          (dis)    [m]     */
            {10.0, 0.0, 0.0 }, /* wheel_carrier_velocity_WC         (vel)    [m/s]   */
            { 0.0, 0.0, 0.0 }, /* wheel_carrier_angular_velocity_WC (omega)  [rad/s] */
            { 1.0, 0.0, 0.0,   /* WC_to_G_transformation            (tramat) [-]     */
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0 },
            0,                 /* wheel_angle              (angtwc) [rad]   */
            0.0                /* wheel_angular_velocity   (omegar) [rad/s] */
        };

        double forces[3]  = {0};          /* [N]  */
        double torques[3] = {0};          /* [Nm] */
        double varinf[VARINF_SIZE] = {0}; /* [-]  */

        for (i=0; i<10; ++i)
        {
            fp.mfs_api_tire_set_input(tire1, &input_data);
            fp.mfs_api_tire_update(tire1);
            fp.mfs_api_tire_get_output(tire1, forces, torques, varinf, VARINF_SIZE);

            printf("Forces: %f\t%f\t%f\n",    forces[0],  forces[1],  forces[2]);
            printf("Torques: %f\t%f\t%f\n\n", torques[0], torques[1], torques[2]);

            input_data.time += dt;
        }
    }
    else
    {
        printf("Failed to initialize tire\n");
    }

    fp.mfs_api_tire_terminate(tire1);
    fp.mfs_api_simulation_terminate(simulation);

    mfs_api_runtime_loader_terminate(runtime_loader);

    return 0;
}
