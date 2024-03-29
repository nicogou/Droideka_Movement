enum Movement_Type
{
    UNSPECIFIED = 0,
    COG_TRAJ = 1,
    STABLE_GAIT = 2,
    TROT_GAIT = 3,
    SEQUENCE = 4,
};
typedef enum Movement_Type Movement_Type;

enum MovementSequence
{
    STARTING_SEQUENCE = 0,
    INTERMEDIATE_SEQUENCE = 1,
    FINISHING_SEQUENCE = 2,
};
typedef enum MovementSequence MovementSequence;

#ifndef Droideka_Movement_h
#define Droideka_Movement_h

#include <Droideka_Position.h>
#include "../Droideka/utils/structs.h"
#include <Arduino.h>

#define NB 3

class Droideka_Movement
{
public:
    Movement_Type type = UNSPECIFIED;
    bool started = false;
    bool finished = false;
    bool paused = false;
    unsigned int iter = 0;
    unsigned int nb_iter = TIME_SAMPLE;
    unsigned long start = 0;
    unsigned long time_span = 0;
    unsigned long time_iter[TIME_SAMPLE];
    int8_t leg_id = -1;
    bool next_pos_calc = false;

    float M[LEG_NB][2];
    float M_prime[LEG_NB][2];
    int8_t nb = NB;
    float cog[NB + 1][2];     // First index = {0, 0}; Last index = {deplacement_x, deplacement_y}; In-between index = center of gravity of the triangles formed by the three touching legs.
    float factor = 1.0 / 5.0; // 1/10 avait marché lors d'essais préliminaires.
    float deplacement[2];     // {x, y}
    float direction;
    float last_direction = 0;
    float rotation;
    float last_rotation;
    float next_longitudinal = 0;
    float next_lateral = 0;
    float next_angle = 0;
    float longitudinal = 0;
    float lateral = 0;
    float angle = 0;
    int sections[3 * NB + 1];

    float default_pos[LEG_NB][3] = {
        {THETA_IDLE, X_IDLE, Y_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_TOUCHING}};
    Droideka_Position default_position = Droideka_Position(default_pos);

    Droideka_Position start_position;
    Droideka_Position next_position;
    Droideka_Position end_position;

    MovementSequence seq = STARTING_SEQUENCE;
    MovementSequence next_seq = STARTING_SEQUENCE;

    int8_t leg_order[LEG_NB] = {1, 2, 3, 4};
    unsigned long delta_time = 28;
    unsigned long lifting_leg_time = 14;

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
    Droideka_Movement(Movement_Type mvmt_type, Droideka_Position start_position_, float throttle_longitudinal, float throttle_lateral, float throttle_vertical, float throttle_angle, unsigned long span);
    ErrorCode establish_cog_movement(float throttle_longitudinal_zeroed, float throttle_lateral_zeroed, float throttle_vertical_zeroed, float throttle_angle_zeroed);

    Droideka_Movement(Droideka_Position start_position_, Droideka_Position end_position_, unsigned long span);
    void add_position(Droideka_Position pos, unsigned long span);
    Droideka_Position get_future_position(Droideka_Position start_pos, int ii);                                                       // Fonction générale appelée par la classe Droideka en fonction du mouvement.
    Droideka_Position get_future_position(int ii);                                                                                    // Séquence de Droideka_Position.
    Droideka_Position get_future_position(Droideka_Position start_pos, float trans_x, float trans_y, float trans_z, float rot_angle); // Trajectoire du centre de gravité
    Droideka_Position get_future_position(Droideka_Position start_pos, Droideka_Position end_pos, unsigned int ii);                   // Marche.
    Droideka_Position get_final_position(Droideka_Position start_pos);
    float *get_lifted_position(int8_t leg, Droideka_Position start_pos, Droideka_Position end_pos, int time_, int time_start_lifting, int time_end_lifting);
    ErrorCode establish_legs_order(float direction);
    bool compare_directions();
    void establish_deplacement(float throttle_longitudinal_zeroed, float throttle_lateral_zeroed, float throttle_angle_zeroed);
    void stable_movement();
    void trotting_movement();
    void keep_going();
    float P(int n, float t);
};
#endif