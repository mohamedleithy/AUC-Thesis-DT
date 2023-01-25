#include <mfs_tire_api.h>

#include <stdio.h>

static const char* tir_file = "Siemens_car205_60R15.tir";
static const double dt = 0.001;
#define VARINF_SIZE 60

void MFS_CALL simple_logger(int severity, const char* message)
{
    printf("severity: %d\n", severity);
    printf("msg: %s\n", message);
}

int main()
{  
    MFS_API_TIRE* tire1 = NULL;
    MFS_API_TIRE* tire2 = NULL;
    
    MFS_API_SIMULATION* simulation = mfs_api_simulation_create();
    mfs_api_simulation_set_log_handler(simulation, simple_logger);

    /* Create two new tires */
    tire1 = mfs_api_tire_create(simulation);
    tire2 = mfs_api_tire_create(simulation);

    /* As before we initialize the tires. Tires can be run in different modes with 
     * different tire property files. The main limitation is that they all need to share
     * the same road type (the Exxxx digit).
     */
    mfs_api_tire_initialize_simulation_mode(tire1, 11134);
    mfs_api_tire_initialize_tire_property_file(tire1, tir_file);

    mfs_api_tire_initialize_simulation_mode(tire2, 11134);
    mfs_api_tire_initialize_tire_property_file(tire2, tir_file);

    if(mfs_api_tire_init(tire1) && mfs_api_tire_init(tire2))
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

        double forces[3]           = {0};
        double torques[3]          = {0};
        double varinf[VARINF_SIZE] = {0};

        for (i=0; i<10; ++i)
        {
            /* In this example, we use the same input for both tires, in reality, inputs will 
             * normally be different
             */
            mfs_api_tire_set_input(tire1, &input_data);
            mfs_api_tire_update(tire1);
            mfs_api_tire_get_output(tire1, forces, torques, varinf, VARINF_SIZE);

            printf("Forces tire1: %f\t%f\t%f\n",    forces[0],  forces[1],  forces[2]);

            mfs_api_tire_set_input(tire2, &input_data);
            mfs_api_tire_update(tire2);
            mfs_api_tire_get_output(tire2, forces, torques, varinf, VARINF_SIZE);

            printf("Forces tire2: %f\t%f\t%f\n\n",    forces[0],  forces[1],  forces[2]);

            input_data.time += dt;
        }
    }
    else
    {
        printf("Failed to initialize tire\n");
    }
    
    mfs_api_tire_terminate(tire1);
    mfs_api_tire_terminate(tire2);
    mfs_api_simulation_terminate(simulation);

    return 0;
}
