enum Movement_Type
{
    UNSPECIFIED = 0,
    CENTER_OF_GRAVITY_TRAJ = 1,
    LEGS_TRAJ = 2,
    ROBOT_TRAJ = 3,
};
typedef enum Movement_Type Movement_Type;

#ifndef Droideka_Movement_h
#define Droideka_Movement_h

#include <Droideka_Position.h>
#include "../Droideka/utils/structs.h"
#include <Arduino.h>

class Droideka_Movement
{
public:
    Movement_Type type = UNSPECIFIED;
    bool started = false;
    bool finished = false;
    unsigned int iter = 0;
    unsigned long start = 0;
    unsigned long time_span = 0;
    bool next_pos_calc = false;

    Droideka_Position start_position;
    Droideka_Position next_position;
    Droideka_Position end_position;

    int leg_order[LEG_NB];
    int moving_leg_nb = 0;
    unsigned long delta_time;

    float tx[TIME_SAMPLE];
    float ty[TIME_SAMPLE];
    float tz[TIME_SAMPLE];
    float alpha[TIME_SAMPLE];
    float reverse_tx[TIME_SAMPLE];
    float reverse_ty[TIME_SAMPLE];
    float reverse_tz[TIME_SAMPLE];
    float reverse_alpha[TIME_SAMPLE];

    float shoulder_pos[LEG_NB][2] = {
        {-BODY_WIDTH / 2, BODY_LENGTH / 2},
        {BODY_WIDTH / 2, BODY_LENGTH / 2},
        {-BODY_WIDTH / 2, -BODY_LENGTH / 2},
        {BODY_WIDTH / 2, -BODY_LENGTH / 2}};
    float shoulder_mult[LEG_NB][2] = {
        {-1, 1},
        {1, 1},
        {-1, -1},
        {1, -1}};

    Droideka_Movement();
    Droideka_Movement(Droideka_Position start_position_, float trans_x[TIME_SAMPLE], float trans_y[TIME_SAMPLE], float trans_z[TIME_SAMPLE], float rot_angle[TIME_SAMPLE], unsigned long span);
    Droideka_Movement(Droideka_Position start_position_, int16_t throttle_longitudinal, int16_t throttle_lateral, int16_t throttle_vertical, int16_t throttle_angle, bool lifting_legs);
    ErrorCode establish_cog_movement(int throttle_longitudinal, int throttle_lateral);
    ErrorCode establish_cog_movement(int16_t throttle_longitudinal, int16_t throttle_lateral, int16_t throttle_vertical, int16_t throttle_angle);
    Droideka_Position get_future_position(Droideka_Position start_pos, float trans_x, float trans_y, float trans_z, float rot_angle, int one_leg = -1);
    Droideka_Position get_final_position(Droideka_Position start_pos);
    // Droideka_Position get_lifted_position(int leg, Droideka_Position start_pos, Droideka_Position end_pos, int time_);
    // ErrorCode establish_legs_movement(bool lifting_legs);
};
#endif