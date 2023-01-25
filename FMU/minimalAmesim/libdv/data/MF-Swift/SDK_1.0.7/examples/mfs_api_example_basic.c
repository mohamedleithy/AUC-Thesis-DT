#include <mfs_tire_api.h>

#include <stdio.h>

static const char* tir_file = "Siemens_car205_60R15.tir";

/* The time step used for the simulation.
 * 
 * Currently, the model only supports a fixed time step of 1000Hz.
 * Longer time steps are not supported. If a lower simulation rate
 * is required, multiple update calls will be required (see below)
 */
static const double dt = 0.001; /* [s] */

/* In this example we use a fixed size array to obtain the varinf data from
 * the library. This example uses a size of 60 (which is the current number
 * of entries). The library can deal with any size. The actual number of 
 * entries can be obtained by calling mfs_api_simulation_get_varinf_size.
 * If the array you provide is smaller than the value returned, you only get 
 * a subset of the varinf outputs. If the array is larger, it will be padded
 * with zeros.
 * Depending on your use case, you can also use mfs_api_simulation_get_varinf_size
 * to get the size of a dynamically allocated buffer that will always be fitted
 * to the actual varinf size.
 */
#define VARINF_SIZE 60

void MFS_CALL simple_logger(int severity, const char* message)
{
    printf("severity: %d\n", severity);
    printf("msg: %s\n", message);
}

int main()
{  
    MFS_API_TIRE* tire1 = NULL;
    
    /* Initialize the simulation by creating the global simulation object and registering
     * a log callback that will be called any time the library logs a message.
     *
     * The MFS_API_SIMULATION object tracks the global (tire independent) state of a
     * simulation. Currently, MF-Tyre/MF-Swift only supports a single MFS_API_SIMULATION 
     * object per process.
     */
    MFS_API_SIMULATION* simulation = mfs_api_simulation_create();
    mfs_api_simulation_set_log_handler(simulation, simple_logger);

    /* Create a new tire
     *
     * The MFS_API_TIRE object represents a single tire in the simulation. It mainly serves
     * as a handle to a specific tire.
     */
    tire1 = mfs_api_tire_create(simulation);

    /* mfs_api_tire_initialize_* functions are used to set specific tire properties for
     * a specific tire. The order in which the functions are called is not important.
     * Calling a function multiple times with different arguments overwrites what
     * has been set before.
     *
     * All initialization settings are applied once the mfs_api_tire_init function (below)
     * is called. 
     */
    mfs_api_tire_initialize_simulation_mode(tire1, 11134);
    mfs_api_tire_initialize_tire_property_file(tire1, tir_file);

    /* Simcenter Tire 2020.2 onwards support temperature and velocity modelling. With a supported
     * tire property file, T&V model can be initialized to required mode by using 
     * mfs_api_tire_initialize_temperature_and_velocity_mode.
     */
    //mfs_api_tire_initialize_temperature_and_velocity_mode(tire1, MFS_API_TEMPERATURE_MODE_DYNAMIC);

    /* Apply the configuration as set by the mfs_api_tire_initialize_* functions. After this
     * call the tire is ready to be used to calculate forces and moments.
     *
     * After calling this function, calls to mfs_api_tire_initialize_* have no effect anymore.
     */
    if(mfs_api_tire_init(tire1))
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

        /* For demonstration purposes, we'll only run a few iterations */
        for (i=0; i<10; ++i)
        {
            /* Transfer the input data to the tire */
            mfs_api_tire_set_input(tire1, &input_data);
            /* Calculate the new state */
            mfs_api_tire_update(tire1);
            /* Retrieve the output */
            mfs_api_tire_get_output(tire1, forces, torques, varinf, VARINF_SIZE);

            printf("Forces: %f\t%f\t%f\n",    forces[0],  forces[1],  forces[2]);
            printf("Torques: %f\t%f\t%f\n\n", torques[0], torques[1], torques[2]);

            /* The time input is important. Currently, the model runs at a fixed rate of 
             * 1000Hz. The mfs_api_tire_update behavior depends on the input time. If 
             * update is called with a time increase smaller than 1ms, the previous values
             * are returned. Updates larger than 1ms are currently not supported.
             * 
             * If the model model needs to be embedded in a simulation with a lower update
             * frequency, mfs_api_tire_update needs to be called multiple times in a row.
             */
            input_data.time += dt;
        }
    }
    else
    {
        printf("Failed to initialize tire\n");
    }
    
    mfs_api_tire_terminate(tire1);
    mfs_api_simulation_terminate(simulation);

    return 0;
}