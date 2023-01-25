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

/* As an example, we define a custom road data structure. A pointer to an instance of this
 * structure will be passed to the road registration function. Each time your registered
 * road function is called, this pointer will be passed as well.
 * This allows you to pass any data required for proper operation of your road method
 * to the road method at runtime. As an example, if required, a tire id can be stored in 
 * order to allow a single callback function to service multiple tires at the same time
 */
typedef struct
{
    int internal_tire_id;
    double height_offset;
} CustomRoadData;

/* This example implementation simply uses the height_offset specified in the custom data
 * object to set the road height for each point
 */
MFS_API_BOOL MFS_CALL external_road(
    double time, 
    size_t number_of_road_points, 
    const MFS_API_ROAD_COORDINATE* coordinates, 
    double* road_height, 
    MFS_API_ROAD_DATA* road_data, 
    void* custom_data
    )
{
    CustomRoadData* custom_road_data = (CustomRoadData*) custom_data;

    (void)(time);
    (void)(number_of_road_points);
    (void)(coordinates);

    road_height[0] = custom_road_data->height_offset;

    road_data->friction_x = 1.0;
    road_data->friction_y = 1.0;
    road_data->curvature  = 0.0;

    return MFS_API_TRUE;
}

int main()
{  
    MFS_API_TIRE* tire1 = NULL;

    /* In this example, we use a simple custom data structure to pass customized data
     * into the external road callback.
     */
    CustomRoadData road_data = {1, 0.1};
    
    MFS_API_SIMULATION* simulation = mfs_api_simulation_create();

    mfs_api_simulation_set_log_handler(simulation, simple_logger);
    tire1 = mfs_api_tire_create(simulation);

    /* Ensure the first (Exxxx) iswitch digit is set to 3 to enable usage of the external road,
     * the contact method (xxBxx) should be either smooth road (1) or enveloping (5)
     */
    mfs_api_tire_initialize_simulation_mode(tire1, 31134);
    mfs_api_tire_initialize_tire_property_file(tire1, tir_file);

    /* Register the callback function together with a pointer to our custom data structure. */
    mfs_api_tire_initialize_external_road_callback(tire1, external_road, 1, (void*)&road_data);

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
