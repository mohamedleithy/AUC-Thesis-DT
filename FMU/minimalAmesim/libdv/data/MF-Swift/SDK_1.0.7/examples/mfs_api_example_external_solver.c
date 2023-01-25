#include <mfs_tire_api.h>

#include <stdio.h>
#include <stdlib.h>

static const char* tir_file = "Siemens_car205_60R15.tir";

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

    /* We will set the solver mode to to external */
    mfs_api_tire_initialize_solver_mode(tire1, MFS_API_SOLVER_MODE_EXTERNAL);

    if(mfs_api_tire_init(tire1))
    {
        size_t i = 0;
        size_t j = 0;

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

        /* We need to properly size the state and derivative arrays */
        size_t state_size = 0;
        size_t internal_state_size = 0;
        double* state = NULL;
        double* state_derivative = NULL;
        char*   internal_state = NULL;

        /* The state size can depend on the selected tire mode */
        mfs_api_tire_get_state_size(tire1, &state_size);
        mfs_api_tire_get_required_internal_state_size(tire1, &internal_state_size);

        state =            (double*) malloc(state_size * sizeof(double));
        state_derivative = (double*) malloc(state_size * sizeof(double));
        internal_state =   (char*)   malloc(internal_state_size);

        if (state && state_derivative && internal_state)
        {
            /* The first step is to initialize the tire and obtain the intial condition
             * based on the input at t=0
             */
            mfs_api_tire_set_input(tire1, &input_data);
            mfs_api_tire_get_initial_condition(tire1, 
                                               state, state_size,
                                               internal_state, internal_state_size);

            for (i=0; i<10; ++i)
            {
                /* In addition to setting the inputs and getting the outputs, we need to set
                 * the differential equation state and the internal state before calling update.
                 * After the update call, we also retrieve the derivative and the new internal
                 * state.
                 */
                mfs_api_tire_set_input(tire1, &input_data);
                mfs_api_tire_set_state(tire1, state, state_size);
                mfs_api_tire_set_internal_state(tire1, internal_state, internal_state_size);
                mfs_api_tire_update(tire1);
                mfs_api_tire_get_output(tire1, forces, torques, varinf, VARINF_SIZE);
                mfs_api_tire_get_state_derivative(tire1, state_derivative, state_size);
                mfs_api_tire_get_internal_state(tire1, internal_state, internal_state_size);

                printf("Forces: %f\t%f\t%f\n",    forces[0],  forces[1],  forces[2]);
                printf("Torques: %f\t%f\t%f\n\n", torques[0], torques[1], torques[2]);

                /* Here we use a simple forward euler with a small time step for demonstration
                 * purposes only. The model is tuned for a RK4 solver with a 0.001s time step */
                for (j=0; j<state_size; ++j)
                {
                    state[j] += state_derivative[j] * 0.0001;
                }

                input_data.time += 0.0001;
            }

            free(state);
            free(state_derivative);
            free(internal_state);
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
