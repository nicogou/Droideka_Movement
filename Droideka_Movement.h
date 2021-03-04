#ifndef Droideka_Movement_h
#define Droideka_Movement_h

#include <Droideka_Position.h>
#include "../Droideka/utils/structs.h"
#include <Arduino.h>

class Droideka_Movement
{
public:
    Droideka_Position start_position;
    Droideka_Position end_position;
    Droideka_Position positions[TIME_SAMPLE];
    bool valid_movement = false;
    bool stable_movement = false; // A implémenter.

    unsigned long start_walk_time;

    int leg_order[LEG_NB];
    int first_leg_to_move;
    bool reverse_angle;
    int shoulder_right;
    int shoulder_left;
    bool leg_lifted[LEG_NB];
    int moving_leg_nb = 0;
    unsigned long delta_time;
    float alpha_max;
    float alpha_min;

    float tx[TIME_SAMPLE];
    float ty[TIME_SAMPLE];
    float alpha[TIME_SAMPLE];
    float reverse_tx[TIME_SAMPLE];
    float reverse_ty[TIME_SAMPLE];
    float reverse_alpha[TIME_SAMPLE];

    float middle_point[2];

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
    Droideka_Movement(Droideka_Position start_position_, int throttle_longitudinal, int throttle_lateral);
    ErrorCode establish_cog_movement(int throttle_longitudinal, int throttle_lateral);
    ErrorCode establish_cog_movement_advanced(int throttle_longitudinal, int throttle_lateral, int throttle_angle);
    ErrorCode establish_cog_movement_stable(int throttle_longitudinal, int throttle_lateral, int throttle_angle);
    ErrorCode establish_cog_movement_next_level(int throttle_longitudinal, int throttle_lateral, int throttle_angle);
    bool establish_stableness(float move_longitudinal, float move_lateral, float move_angle);
    bool find_extreme_alpha(float throttle_longitudinal, float throttle_lateral, float move_longitudinal, float move_lateral);
    void establish_leg_order(float throttle_longitudinal, float throttle_lateral);
    Droideka_Position get_future_position(Droideka_Position start_pos, float trans_x[TIME_SAMPLE], float trans_y[TIME_SAMPLE], float angle[TIME_SAMPLE], unsigned long time_elapsed, int one_leg = -1);
    Droideka_Position get_final_position(Droideka_Position start_pos);
    Droideka_Position get_lifted_position(int leg, Droideka_Position start_pos, Droideka_Position end_pos, unsigned long time_);
    ErrorCode establish_legs_movement();
};
#endif