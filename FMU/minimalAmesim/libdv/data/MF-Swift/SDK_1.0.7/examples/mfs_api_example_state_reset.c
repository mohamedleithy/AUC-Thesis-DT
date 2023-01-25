#include <mfs_tire_api.h>

#include <stdio.h>
#include <stdlib.h>

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

    mfs_api_tire_initialize_simulation_mode(tire1, 11134);
    mfs_api_tire_initialize_tire_property_file(tire1, tir_file);

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

        double forces[3]           = {0};
        double torques[3]          = {0};
        double varinf[VARINF_SIZE] = {0};

        size_t state_size = 0;
        void* state = NULL;

        /* Before we can obtain the tire state, we need to allocate a buffer large
         * enough to hold the tire state. This is implementation defined and can not
         * be used directly by applications. Therefore, it will be stored as a void*
         */
        mfs_api_tire_get_required_internal_state_size(tire1, &state_size);
        state = malloc(state_size);

        if (state)
        {
            for (i=0; i<10; ++i)
            {
                /* We will obtain the state at the beginning of the fourth iteration */
                if (4 == i)
                {
                    mfs_api_tire_get_internal_state(tire1, state, state_size);
                }
                mfs_api_tire_set_input(tire1, &input_data);
                mfs_api_tire_update(tire1);
                mfs_api_tire_get_output(tire1, forces, torques, varinf, VARINF_SIZE);

                input_data.time += dt;

                printf("t: %f\n", input_data.time);
                printf("Forces: %f\t%f\t%f\n",    forces[0],  forces[1],  forces[2]);
                printf("Torques: %f\t%f\t%f\n\n", torques[0], torques[1], torques[2]);
            }

            printf("\nResetting state\n\n");
            /* Restore the state and reset the current time */
            mfs_api_tire_set_internal_state(tire1, state, state_size);
            input_data.time = 4*dt;

            /* Re-run the model starting from the restored state */
            for (i=4; i<10; ++i)
            {
                mfs_api_tire_set_input(tire1, &input_data);
                mfs_api_tire_update(tire1);
                mfs_api_tire_get_output(tire1, forces, torques, varinf, VARINF_SIZE);

                input_data.time += dt;

                printf("t: %f\n", input_data.time);

                printf("Forces: %f\t%f\t%f\n",    forces[0],  forces[1],  forces[2]);
                printf("Torques: %f\t%f\t%f\n\n", torques[0], torques[1], torques[2]);
            }
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
