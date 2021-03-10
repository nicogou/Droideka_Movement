enum Movement_Type
{
    UNSPECIFIED = 0,
    CENTER_OF_GRAVITY_TRAJ = 1,
    LEGS_TRAJ = 2,
    ROBOT_TRAJ = 3,
    DIRECT_FOOT_MVMT = 4,
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
    unsigned int nb_iter = TIME_SAMPLE;
    unsigned long start = 0;
    unsigned long time_span = 0;
    unsigned long time_iter[TIME_SAMPLE];
    int8_t leg_id = -1;
    bool next_pos_calc = false;

    Droideka_Position start_position;
    Droideka_Position next_position;
    Droideka_Position end_position;

    int8_t leg_order[LEG_NB] = {1, 2, 3, 4};
    int8_t moving_leg_nb = 0;
    unsigned long delta_time;

    float params[12][TIME_SAMPLE];
    float reverse_params[12][TIME_SAMPLE];

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
    Droideka_Movement(Droideka_Position start_position_, float theta[TIME_SAMPLE], float rho[TIME_SAMPLE], float height[TIME_SAMPLE], unsigned long span, int8_t one_leg = -1);
    Droideka_Movement(Droideka_Position start_position_, Droideka_Position end_position_, unsigned long span, int8_t one_leg = -1);
    Droideka_Movement(Droideka_Position start_position_, int16_t throttle_longitudinal, int16_t throttle_lateral, int16_t throttle_vertical, int16_t throttle_angle, unsigned long span, bool lifting_legs);
    void add_position(Droideka_Position start_position_, Droideka_Position pos, unsigned long span, int8_t one_leg = -1);
    ErrorCode establish_cog_movement(int throttle_longitudinal, int throttle_lateral);
    ErrorCode establish_cog_movement(int16_t throttle_longitudinal, int16_t throttle_lateral, int16_t throttle_vertical, int16_t throttle_angle);
    Droideka_Position get_future_position(Droideka_Position start_pos, int ii);                                                       // Fonction générale appelée par la classe Droideka en fonction du mouvement.
    Droideka_Position get_future_position(int ii);                                                                                    // Séquence de Droideka_Position.
    Droideka_Position get_future_position(Droideka_Position start_pos, float trans_x, float trans_y, float trans_z, float rot_angle); // Trajectoire du centre de gravité
    Droideka_Position get_future_position(float theta, float rho, float height, int8_t one_leg = -1);                                 // Trajectoire des jambes ou de la jambe spécifiée si leg != -1.
    Droideka_Position get_future_position(Droideka_Position start_pos, Droideka_Position end_pos, unsigned int ii);                   // Marche.
    Droideka_Position get_final_position(Droideka_Position start_pos);
    float *get_lifted_position(int8_t leg, Droideka_Position start_pos, Droideka_Position end_pos, int time_);
    ErrorCode establish_legs_movement(bool lifting_legs);
    void stable_movement();
};
#endif