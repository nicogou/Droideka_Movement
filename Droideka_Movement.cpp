#include <Droideka_Movement.h>

Droideka_Movement::Droideka_Movement()
{
}

Droideka_Movement::Droideka_Movement(Droideka_Position start_position_, int16_t throttle_longitudinal, int16_t throttle_lateral, int16_t throttle_vertical, int16_t throttle_angle, unsigned long span, bool lifting_legs)
{
    start_position = start_position_;
    time_span = span * 1000;

    if (!lifting_legs)
    {
        type = CENTER_OF_GRAVITY_TRAJ;
        establish_cog_movement(throttle_longitudinal, throttle_lateral, throttle_vertical, throttle_angle);
        end_position = get_final_position(start_position);
    }
    else
    {
        type = ROBOT_TRAJ;
        establish_cog_movement(0, 0);
        end_position = get_final_position(start_position);
        end_position.print_position("End position");
    }
    //establish_legs_movement(lifting_legs);
}

Droideka_Movement::Droideka_Movement(Droideka_Position start_position_, float trans_x[TIME_SAMPLE], float trans_y[TIME_SAMPLE], float trans_z[TIME_SAMPLE], float rot_angle[TIME_SAMPLE], unsigned long span)
{
    type = CENTER_OF_GRAVITY_TRAJ;

    start_position = start_position_;
    time_span = span * 1000;

    for (unsigned int ii = 0; ii < nb_iter; ii++)
    {
        params[0][ii] = trans_x[ii];
        params[1][ii] = trans_y[ii];
        params[2][ii] = trans_z[ii];
        params[3][ii] = rot_angle[ii];
        for (int jj = 4; jj < 12; jj++)
        {
            params[jj][ii] = 0;
        }

        time_iter[ii] = (ii + 1) * time_span / nb_iter;
    }
}

Droideka_Movement::Droideka_Movement(Droideka_Position start_position_, float theta[TIME_SAMPLE], float rho[TIME_SAMPLE], float height[TIME_SAMPLE], unsigned long span, int8_t one_leg = -1)
{
    type = LEGS_TRAJ;

    start_position = start_position_;
    leg_id = one_leg;
    time_span = span * 1000;

    for (unsigned int ii = 0; ii < nb_iter; ii++)
    {
        params[0][ii] = theta[ii];
        params[1][ii] = rho[ii];
        params[2][ii] = height[ii];
        for (int jj = 3; jj < 12; jj++)
        {
            params[jj][ii] = 0;
        }

        time_iter[ii] = (ii + 1) * time_span / nb_iter;
    }
}

Droideka_Movement::Droideka_Movement(Droideka_Position start_position_, Droideka_Position end_position_, unsigned long span, int8_t one_leg = -1)
{
    type = DIRECT_FOOT_MVMT;

    start_position = start_position_;

    for (unsigned int ii = 0; ii < TIME_SAMPLE; ii++)
    {
        for (int jj = 0; jj < 12; jj++)
        {
            params[jj][ii] = 0;
        }
        time_iter[ii] = 0;
    }
    nb_iter = 0;
    add_position(start_position, end_position_, span, one_leg);
}

void Droideka_Movement::add_position(Droideka_Position start_position_, Droideka_Position pos, unsigned long span, int8_t one_leg = -1)
{
    if (type == DIRECT_FOOT_MVMT)
    {
        for (int ii = 0; ii < LEG_NB; ii++)
        {
            for (int jj = 0; jj < 3; jj++)
            {
                params[(LEG_NB - 1) * ii + jj][nb_iter] = pos.legs[ii][jj];
                if (one_leg != -1 && ii != one_leg)
                {
                    params[(LEG_NB - 1) * ii + jj][nb_iter] = start_position_.legs[ii][jj];
                }
            }
        }

        time_span += span * 1000;
        time_iter[nb_iter] = time_span;
        nb_iter++;
    }
}

ErrorCode Droideka_Movement::establish_cog_movement(int16_t throttle_longitudinal, int16_t throttle_lateral, int16_t throttle_vertical, int16_t throttle_angle)
{
    for (unsigned int ii = 0; ii < nb_iter; ii++)
    {
        params[0][ii] = ((float)throttle_lateral - 105.0) * (2.0 - (-2.0)) / (792.0 - 105.0) + -2.0;
        params[1][ii] = ((float)throttle_longitudinal - 831.0) * (2.0 - (-2.0)) / (140.0 - 831.0) + -2.0;
        params[2][ii] = ((float)throttle_vertical - 825.0) * (-2.0 - (2.0)) / (137.0 - 825.0) + 2.0;
        params[3][ii] = ((float)throttle_angle - 185.0) * (20.0 - (-20.0)) / (863.0 - 185.0) + -20.0;

        time_iter[ii] = (ii + 1) * time_span / nb_iter;
    }

    leg_order[3] = 1;
    leg_order[1] = 2;
    leg_order[2] = 3;
    leg_order[0] = 4;

    moving_leg_nb = 4;
    delta_time = nb_iter / (moving_leg_nb * 2);
    // stable_movement();

    return NO_ERROR;
}

ErrorCode Droideka_Movement::establish_cog_movement(int throttle_longitudinal, int throttle_lateral)
{
    for (unsigned int ii = 0; ii < nb_iter; ii++)
    {
        time_iter[ii] = (ii + 1) * time_span / nb_iter;
    }
    leg_order[3] = 1;
    leg_order[1] = 2;
    leg_order[2] = 3;
    leg_order[0] = 4;

    moving_leg_nb = 2;
    delta_time = nb_iter / (moving_leg_nb * 2);
    stable_movement();

    return NO_ERROR;
}

Droideka_Position Droideka_Movement::get_future_position(Droideka_Position start_pos, int ii)
{
    if (type == CENTER_OF_GRAVITY_TRAJ)
    {
        return get_future_position(start_pos, params[0][ii], params[1][ii], params[2][ii], params[3][ii]);
    }
    else if (type == LEGS_TRAJ)
    {
        return get_future_position(params[0][ii], params[1][ii], params[2][ii], leg_id);
    }
    else if (type == DIRECT_FOOT_MVMT)
    {
        return get_future_position(ii);
    }
    else if (type == ROBOT_TRAJ)
    {
        return get_future_position(start_pos, end_position, ii);
    }
}

Droideka_Position Droideka_Movement::get_future_position(int iteration)
{
    float temp[LEG_NB][3];
    for (int ii = 0; ii < LEG_NB; ii++)
    {
        for (int jj = 0; jj < 3; jj++)
        {
            temp[ii][jj] = params[(LEG_NB - 1) * ii + jj][iteration];
        }
    }
    Droideka_Position final_pos(temp);
    return final_pos;
}

Droideka_Position Droideka_Movement::get_future_position(float theta, float rho, float height, int8_t one_leg = -1)
{
    float temp[LEG_NB][3];
    for (int ii = 0; ii < LEG_NB; ii++)
    {
        if (one_leg != -1)
        {
            ii = one_leg;
        }

        temp[ii][0] = theta;
        temp[ii][1] = rho;
        temp[ii][2] = height;

        if (one_leg != -1)
        {
            ii = LEG_NB;
        }
    }
    Droideka_Position final_pos(temp);
    return final_pos;
}

Droideka_Position Droideka_Movement::get_future_position(Droideka_Position start_pos, float trans_x, float trans_y, float trans_z, float rot_angle)
{
    float temp[LEG_NB][2];           // x and y final coordinates of each feet
    float temp_trans[LEG_NB][2];     // translation vector of the shoulder.
    float temp_final_pos[LEG_NB][3]; // used to build the Droideka_Position object by calculating rho, theta and z thanks to x and y stored in temp.
    float feet_start[LEG_NB][2];     // Feet coordinates at the start

    for (int ii = 0; ii < LEG_NB; ii++)
    {
        temp_trans[ii][0] = shoulder_pos[ii][0] * (cos(PI * rot_angle / 180) - 1) - shoulder_pos[ii][1] * sin(PI * rot_angle / 180) + trans_x;
        temp_trans[ii][1] = shoulder_pos[ii][0] * sin(PI * rot_angle / 180) - shoulder_pos[ii][1] * (cos(PI * rot_angle / 180) - 1) + trans_y;

        feet_start[ii][0] = shoulder_mult[ii][0] * (start_pos.legs[ii][1] * cos(PI * start_pos.legs[ii][0] / 180));
        feet_start[ii][1] = shoulder_mult[ii][1] * (start_pos.legs[ii][1] * sin(PI * start_pos.legs[ii][0] / 180));

        temp[ii][0] = (feet_start[ii][0] - temp_trans[ii][0]) * cos(PI * rot_angle / 180) + (feet_start[ii][1] - temp_trans[ii][1]) * sin(PI * rot_angle / 180);
        temp[ii][1] = (feet_start[ii][1] - temp_trans[ii][1]) * cos(PI * rot_angle / 180) + (feet_start[ii][0] - temp_trans[ii][0]) * sin(PI * rot_angle / 180);

        temp_final_pos[ii][2] = start_pos.legs[ii][2] + trans_z;
        temp_final_pos[ii][1] = sqrt(temp[ii][0] * temp[ii][0] + temp[ii][1] * temp[ii][1]);
        if (temp_final_pos[ii][1] == 0) // Si x et y sont nuls
        {
            temp_final_pos[ii][0] = 0; // TODO : réfléchir à cette valeur. Est-il possible de déterminer theta si x et y sont nuls?
        }
        else if (temp[ii][0] == 0) // Si rho est non nul, alors si x est nul, y est non nul et on peut diviser par y.
        {
            temp_final_pos[ii][0] = temp[ii][1] / abs(temp[ii][1]) * 90; // Si x est nul, rho vaut + ou - 90°, determiné par le signe de y.
        }
        else
        {
            temp_final_pos[ii][0] = shoulder_mult[ii][0] * shoulder_mult[ii][1] * 180 * atan(temp[ii][1] / temp[ii][0]) / PI; // Dans le cas général, tan(theta) = y/x.
        }
    }
    Droideka_Position final_pos(temp_final_pos);
    return final_pos;
}

Droideka_Position Droideka_Movement::get_future_position(Droideka_Position start_pos, Droideka_Position end_pos, unsigned int ii)
{
    float temp[LEG_NB][3];
    unsigned long time_leg_starts_lifting;
    unsigned long time_leg_touches_ground_again;
    Droideka_Position temp_current_pos = get_future_position(start_pos, params[0][ii], params[1][ii], params[2][ii], params[3][ii]);
    Droideka_Position temp_future_pos = get_future_position(end_pos, reverse_params[0][ii], reverse_params[1][ii], reverse_params[2][ii], reverse_params[3][ii]); // Calculates the position of the legs after the leg has been lifted and put back on the ground.
    // Droideka_Position temp_future_pos = get_future_position(start_pos, params[0][ii], params[1][ii], params[2][ii], params[3][ii]); // Calculates the position of the legs after the leg has been lifted and put back on the ground.

    for (int jj = 0; jj < LEG_NB; jj++)
    {
        time_leg_starts_lifting = (leg_order[jj] - 1) * nb_iter / (nb) + delta_time;
        time_leg_touches_ground_again = (leg_order[jj]) * nb_iter / (nb)-1;

        if (ii < time_leg_starts_lifting)
        {
            for (int kk = 0; kk < 3; kk++)
            {
                temp[jj][kk] = temp_current_pos.legs[jj][kk];
            }
        }
        else if (ii >= time_leg_starts_lifting && ii < time_leg_touches_ground_again)
        {
            if (leg_order[jj] == 1 || leg_order[jj] == 2) // On ne bouge que les deux premières jambes.
            {
                for (int kk = 0; kk < 3; kk++)
                {
                    temp[jj][kk] = get_lifted_position(jj, temp_current_pos, temp_future_pos, ii)[kk];
                }
            }
            else
            {
                for (int kk = 0; kk < 3; kk++)
                {
                    temp[jj][kk] = temp_current_pos.legs[jj][kk];
                }
            }
        }
        else if (ii >= time_leg_touches_ground_again && ii <= nb_iter)
        {
            if (leg_order[jj] == 1 || leg_order[jj] == 2) // On ne bouge que les deux premières jambes.
            {
                for (int kk = 0; kk < 3; kk++)
                {
                    temp[jj][kk] = temp_future_pos.legs[jj][kk];
                }
            }
            else
            {
                for (int kk = 0; kk < 3; kk++)
                {
                    temp[jj][kk] = temp_current_pos.legs[jj][kk];
                }
            }
        }
    }
    Droideka_Position result = Droideka_Position(temp);
    return result;
}

Droideka_Position Droideka_Movement::get_final_position(Droideka_Position start_pos)
{
    float temp[LEG_NB][3];
    Droideka_Position temp_pos = get_future_position(start_pos, params[0][TIME_SAMPLE - 1], params[1][TIME_SAMPLE - 1], params[2][TIME_SAMPLE - 1], params[3][TIME_SAMPLE - 1]);
    if (type != ROBOT_TRAJ)
    {
        return temp_pos;
    }
    else
    {
        if (seq == STARTING_SEQUENCE)
        {
            for (int ii = 0; ii < LEG_NB; ii++)
            {
                if (leg_order[ii] == 1 || leg_order[ii] == 2) // On ne bouge que les deux premières jambes.
                {
                    for (int jj = 0; jj < 3; jj++)
                    {
                        temp[ii][jj] = get_future_position(default_position, -params[0][TIME_SAMPLE - 1], -params[1][TIME_SAMPLE - 1], -params[2][TIME_SAMPLE - 1], -params[3][TIME_SAMPLE - 1]).legs[ii][jj];
                    }
                }
                else
                {
                    for (int jj = 0; jj < 3; jj++)
                    {
                        temp[ii][jj] = temp_pos.legs[ii][jj];
                    }
                }
            }
            return Droideka_Position(temp);
        }
        if (seq == INTERMEDIATE_SEQUENCE)
        {
            for (int ii = 0; ii < LEG_NB; ii++)
            {
                if (leg_order[ii] == 1 || leg_order[ii] == 2) // On ne bouge que les deux premières jambes.
                {
                    for (int jj = 0; jj < 3; jj++)
                    {
                        temp[ii][jj] = get_future_position(default_position, -params[0][TIME_SAMPLE - 1] / 2.0, -params[1][TIME_SAMPLE - 1] / 2.0, -params[2][TIME_SAMPLE - 1], -params[3][TIME_SAMPLE - 1]).legs[ii][jj];
                    }
                }
                else
                {
                    for (int jj = 0; jj < 3; jj++)
                    {
                        temp[ii][jj] = temp_pos.legs[ii][jj];
                    }
                }
            }
            return Droideka_Position(temp);
        }
        if (seq == FINISHING_SEQUENCE)
        {
            return default_position;
        }
    }
}

float *Droideka_Movement::get_lifted_position(int8_t leg, Droideka_Position debut_pos, Droideka_Position fin_pos, int time_)
{
    static float res[3];
    float debut_time = ((float)leg_order[leg] - 1) * (float)TIME_SAMPLE / ((float)nb) + (float)delta_time;
    float fin_time = (float)leg_order[leg] * (float)TIME_SAMPLE / ((float)nb) - 1;
    float interval_time = (float)fin_time - (float)debut_time;
    float time_from_lifting = (float)time_ - (float)debut_time;

    // Between the lifting and putting back of the leg, theta and X are linear, wheras Y follows a quadratic curve (arbitrarily defined)

    for (int ii = 0; ii < 2; ii++)
    {
        res[ii] = (fin_pos.legs[leg][ii] - debut_pos.legs[leg][ii]) / interval_time * time_from_lifting + debut_pos.legs[leg][ii];
    }
    res[2] = Y_NOT_TOUCHING - (Y_NOT_TOUCHING - Y_TOUCHING) * ((time_from_lifting - interval_time / 2) / (interval_time / 2)) * ((time_from_lifting - interval_time / 2) / (interval_time / 2));

    return res;
}

void Droideka_Movement::stable_movement()
{
    int8_t index;
    float M_default[LEG_NB][2];
    moving_leg_nb = 2;

    deplacement[0] = 0.0;
    deplacement[1] = 2.0;
    if (seq == INTERMEDIATE_SEQUENCE)
    {
    }
    else
    {
        deplacement[0] = deplacement[0] / 2.0;
        deplacement[1] = deplacement[1] / 2.0;
    }

    for (int8_t ii = 0; ii < LEG_NB; ii++)
    {
        index = leg_order[ii] - 1;
        for (int8_t jj = 0; jj < 2; jj++)
        {
            M[index][0] = shoulder_pos[ii][0] + shoulder_mult[ii][0] * start_position.legs[ii][1] * cos(PI * start_position.legs[ii][0] / 180.0);
            M[index][1] = shoulder_pos[ii][1] + shoulder_mult[ii][1] * start_position.legs[ii][1] * sin(PI * start_position.legs[ii][0] / 180.0);
            M_prime[index][jj] = M[index][jj] + deplacement[jj];
            if (seq == INTERMEDIATE_SEQUENCE)
            {
                M_prime[index][jj] = M[index][jj] + 2.0 * deplacement[jj];
            }
        }
    }

    //Block 1
    for (int8_t ii = 0; ii < LEG_NB; ii++)
    {
        for (int8_t jj = 0; jj < 2; jj++)
        {
            Serial.print(ii);
            Serial.print("\t");
            Serial.print(jj);
            Serial.print("\t\t");
            Serial.print(M[ii][jj]);
            Serial.print("\t\t");
            Serial.print(M_prime[ii][jj]);
            Serial.println();
        }
    }

    for (int8_t jj = 0; jj < 2; jj++)
    {
        cog[0][jj] = 0.0;
        cog[1][jj] = (M[1][jj] + M[2][jj]) / 2 + factor * (M[3][jj] - (M[2][jj] + M[1][jj]) / 2);
        cog[2][jj] = (M_prime[0][jj] + M[3][jj]) / 2 + factor * (M[2][jj] - (M_prime[0][jj] + M[3][jj]) / 2);
        cog[3][jj] = deplacement[jj];

        // cog[3][jj] = (M_prime[0][jj] + M[3][jj]) / 2 + factor * (M_prime[1][jj] - (M_prime[0][jj] + M[3][jj]) / 2);
        // cog[4][jj] = (M_prime[1][jj] + M_prime[2][jj]) / 2 + factor * (M_prime[0][jj] - (M_prime[2][jj] + M_prime[1][jj]) / 2);
        // cog[5][jj] = deplacement[jj];
    }

    // Block 2
    for (int8_t ii = 0; ii < nb + 1; ii++)
    {
        Serial.print(cog[ii][0]);
        Serial.print("\t");
        Serial.print(cog[ii][1]);
        Serial.println();
    }

    // Block 3
    for (int8_t ii = 0; ii < nb; ii++)
    {
        for (int jj = ii * nb_iter / nb; jj < ii * nb_iter / nb + delta_time; jj++)
        {
            params[0][jj] = cog[ii][0] + (cog[ii + 1][0] - cog[ii][0]) * ((float)jj + 1 - (float)ii * (float)nb_iter / (float)nb) / ((float)delta_time);
            params[1][jj] = cog[ii][1] + (cog[ii + 1][1] - cog[ii][1]) * ((float)jj + 1 - (float)ii * (float)nb_iter / (float)nb) / ((float)delta_time);
            params[2][jj] = 0;
            params[3][jj] = 0;
            reverse_params[0][jj] = params[0][jj] - deplacement[0];
            reverse_params[1][jj] = params[1][jj] - deplacement[1];
            reverse_params[2][jj] = 0;
            reverse_params[3][jj] = 0;
        }
        for (int jj = ii * nb_iter / nb + delta_time; jj < (ii + 1) * nb_iter / nb; jj++)
        {
            params[0][jj] = params[0][ii * nb_iter / nb + delta_time - 1];
            params[1][jj] = params[1][ii * nb_iter / nb + delta_time - 1];
            params[2][jj] = 0;
            params[3][jj] = 0;
            reverse_params[0][jj] = reverse_params[0][ii * nb_iter / nb + delta_time - 1];
            reverse_params[1][jj] = reverse_params[1][ii * nb_iter / nb + delta_time - 1];
            reverse_params[2][jj] = 0;
            reverse_params[3][jj] = 0;
        }
    }

    for (int8_t ii = 0; ii < nb_iter; ii++)
    {
        Serial.print(params[0][ii]);
        Serial.print("\t\t");
        Serial.print(params[1][ii]);
        Serial.print("\t\t");
        Serial.print(reverse_params[0][ii]);
        Serial.print("\t\t");
        Serial.println(reverse_params[1][ii]);
    }
}

void Droideka_Movement::keep_going()
{
    if (next_seq == INTERMEDIATE_SEQUENCE || next_seq == FINISHING_SEQUENCE)
    {
        Serial.println("Keep going!");
        started = false;
        finished = false;
        seq = next_seq;
        next_seq = STARTING_SEQUENCE;
        start_position = end_position;
        for (int ii = 0; ii < LEG_NB; ii++)
        {
            leg_order[ii] = leg_order[ii] - 2;
            if (leg_order[ii] <= 0)
            {
                leg_order[ii] += 4;
            }
        }
        stable_movement();
        end_position = get_final_position(start_position);
        end_position.print_position("End Position keep going");
    }
}