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

    MFS_API_SIMULATION* simulation = mfs_api_simulation_create();
    mfs_api_simulation_set_log_handler(simulation, simple_logger);

    tire1 = mfs_api_tire_create(simulation);

    /* For this example, we select the steady state dynamics mode to
     * clearly show the effect of using the loaded radius to initialize
     * the model.
     */
    mfs_api_tire_initialize_simulation_mode(tire1, 11104);
    mfs_api_tire_initialize_tire_property_file(tire1, tir_file);

    if(mfs_api_tire_init(tire1))
    {
        int i = 0;
        double loaded_radius = 0.0;

        MFS_INPUT_DATA input_data = {
            0,                 /* time                              [s]              */
            { 0.0, 0.0, 0.19}, /* wheel_carrier_position_G          (dis)    [m]     */
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

        /* To calculate the loaded radius, the initial wheel loads need to be set.
         * In this case, we will calculate the radius under a vertical load of 6000N
         * Calling mfs_api_tire_initialize_outputs is only necessary for calculations
         * that need to be done before the first mfs_api_tire_update and that depend
         * on the forces and torques.
         */
        forces[2] = 6000;
        mfs_api_tire_initialize_outputs(tire1, forces, torques);
        /* The loaded radius also depends on omega_y. Therefore, the inputs need to 
         * be set before calling mfs_api_tire_get_loaded_radius
         */
        mfs_api_tire_set_input(tire1, &input_data);
        /* At this point, the loaded radius can be calculated. */
        mfs_api_tire_get_loaded_radius(tire1, &loaded_radius);  

        /* As an example, we set the vertical position of the tire axle to be equal
         * to the loaded radius. This should result in an Fz of 6000N when we start
         * running the model.
         */
        input_data.wheel_carrier_position_G[2] = loaded_radius;

        for (i=0; i<10; ++i)
        {
            mfs_api_tire_set_input(tire1, &input_data);
            mfs_api_tire_update(tire1);
            mfs_api_tire_get_output(tire1, forces, torques, varinf, VARINF_SIZE);

            printf("Forces: %f\t%f\t%f\n",    forces[0],  forces[1],  forces[2]);
            printf("Torques: %f\t%f\t%f\n\n", torques[0], torques[1], torques[2]);

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
