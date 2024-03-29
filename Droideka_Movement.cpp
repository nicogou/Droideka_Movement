#include <Droideka_Movement.h>

Droideka_Movement::Droideka_Movement()
{
}

Droideka_Movement::Droideka_Movement(Movement_Type mvmt_type, Droideka_Position start_position_, float throttle_longitudinal, float throttle_lateral, float throttle_vertical, float throttle_angle, unsigned long span)
{
    start_position = start_position_;
    time_span = span * 1000;
    type = mvmt_type;
    next_seq = FINISHING_SEQUENCE;

    establish_cog_movement(throttle_longitudinal, throttle_lateral, throttle_vertical, throttle_angle);
    end_position = get_final_position(start_position);
}

ErrorCode Droideka_Movement::establish_cog_movement(float throttle_longitudinal_zeroed, float throttle_lateral_zeroed, float throttle_vertical_zeroed, float throttle_angle_zeroed)
{
    establish_deplacement(throttle_longitudinal_zeroed, throttle_lateral_zeroed, throttle_angle_zeroed); // TODO: handle vertical move.
    // Serial.print("Deplacement X : ");
    // Serial.print(deplacement[0]);
    // Serial.print("\t Deplacement Y : ");
    // Serial.println(deplacement[1]);
    // Serial.print("Direction : ");
    // Serial.println(direction * 180 / PI);
    // Serial.print("Rotation : ");
    // Serial.println(rotation);
    switch (type)
    {
    case 1: // COG_TRAJ
        return NO_ERROR;
        break;
    case 2: // STABLE_GAIT
        establish_legs_order(direction);
        stable_movement();
        return NO_ERROR;
        break;
    case 3: // TROT_GAIT
        establish_legs_order(direction);
        trotting_movement();
        return NO_ERROR;
        break;
    case 4: // SEQUENCE
        return NO_ERROR;
        break;

    default:
        break;
    }

    return NO_ERROR;
}

Droideka_Movement::Droideka_Movement(Droideka_Position start_position_, Droideka_Position end_position_, unsigned long span)
{
    type = SEQUENCE;

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
    add_position(end_position_, span);
}

void Droideka_Movement::add_position(Droideka_Position pos, unsigned long span)
{
    if (type == SEQUENCE)
    {
        for (int ii = 0; ii < LEG_NB; ii++)
        {
            for (int jj = 0; jj < 3; jj++)
            {
                params[(LEG_NB - 1) * ii + jj][nb_iter] = pos.legs[ii][jj];
            }
        }

        time_span += span * 1000;
        time_iter[nb_iter] = time_span;
        nb_iter++;
        end_position = pos;
    }
}

void Droideka_Movement::establish_deplacement(float throttle_longitudinal_zeroed, float throttle_lateral_zeroed, float throttle_angle_zeroed)
{
    if (throttle_lateral_zeroed == 0 && throttle_longitudinal_zeroed >= 0)
    {
        direction = PI / 2;
    }
    else if (throttle_lateral_zeroed == 0 && throttle_longitudinal_zeroed < 0)
    {
        direction = -PI / 2;
    }
    else
    {
        direction = atan2(throttle_longitudinal_zeroed, throttle_lateral_zeroed);
    }
    deplacement[0] = 2.0 * max(abs(throttle_longitudinal_zeroed), throttle_lateral_zeroed) * cos(direction);
    deplacement[1] = 2.0 * max(abs(throttle_longitudinal_zeroed), throttle_lateral_zeroed) * sin(direction);
    rotation = throttle_angle_zeroed * 10.0; // en degrés.
    longitudinal = throttle_longitudinal_zeroed;
    lateral = throttle_lateral_zeroed;
    angle = throttle_angle_zeroed;
    next_longitudinal = throttle_longitudinal_zeroed;
    next_lateral = throttle_lateral_zeroed;
    next_angle = throttle_angle_zeroed;
}

ErrorCode Droideka_Movement::establish_legs_order(float direction)
{
    if (type == STABLE_GAIT)
    {
        if (direction >= 0 && direction < PI / 4)
        {
            leg_order[2] = 1;
            leg_order[3] = 2;
            leg_order[0] = 3;
            leg_order[1] = 4;
        }
        else if (direction >= PI / 4 && direction < PI / 2)
        {
            leg_order[2] = 1;
            leg_order[0] = 2;
            leg_order[3] = 3;
            leg_order[1] = 4;
        }
        else if (direction >= PI / 2 && direction < 3 * PI / 4)
        {
            leg_order[3] = 1;
            leg_order[1] = 2;
            leg_order[2] = 3;
            leg_order[0] = 4;
            if (rotation > 0)
            {
                leg_order[2] = 1;
                leg_order[0] = 2;
                leg_order[3] = 3;
                leg_order[1] = 4;
            }
        }
        else if (direction >= 3 * PI / 4 && direction < PI)
        {
            leg_order[3] = 1;
            leg_order[2] = 2;
            leg_order[1] = 3;
            leg_order[0] = 4;
        }
        else if (direction >= -PI / 4 && direction < 0)
        {
            leg_order[0] = 1;
            leg_order[1] = 2;
            leg_order[2] = 3;
            leg_order[3] = 4;
        }
        else if (direction >= -PI / 2 && direction < -PI / 4)
        {
            leg_order[0] = 1;
            leg_order[2] = 2;
            leg_order[1] = 3;
            leg_order[3] = 4;
        }
        else if (direction >= -3 * PI / 4 && direction < -PI / 2)
        {
            leg_order[1] = 1;
            leg_order[3] = 2;
            leg_order[0] = 3;
            leg_order[2] = 4;
        }
        else if (direction >= -PI && direction < -3 * PI / 4)
        {
            leg_order[1] = 1;
            leg_order[0] = 2;
            leg_order[3] = 3;
            leg_order[2] = 4;
        }
    }
    if (type == TROT_GAIT)
    {
        leg_order[0] = 1;
        leg_order[3] = 1;
        leg_order[1] = 3;
        leg_order[2] = 3;
    }
    return NO_ERROR;
}

Droideka_Position Droideka_Movement::get_future_position(Droideka_Position start_pos, int ii)
{
    if (type == COG_TRAJ)
    {
        // return get_future_position(start_pos, params[0][ii], params[1][ii], params[2][ii], params[3][ii]);
    }
    else if (type == TROT_GAIT)
    {
        return get_future_position(start_pos, end_position, ii);
    }
    else if (type == SEQUENCE)
    {
        return get_future_position(ii);
    }
    else if (type == STABLE_GAIT)
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

Droideka_Position Droideka_Movement::get_future_position(Droideka_Position start_pos, float trans_x, float trans_y, float trans_z, float rot_angle)
{
    float temp[LEG_NB][2];           // x and y final coordinates of each feet
    float temp_trans[LEG_NB][2];     // translation vector of the shoulder.
    float temp_final_pos[LEG_NB][3]; // used to build the Droideka_Position object by calculating rho, theta and z thanks to x and y stored in temp.
    float feet_start[LEG_NB][2];     // Feet coordinates at the start

    for (int ii = 0; ii < LEG_NB; ii++)
    {
        temp_trans[ii][0] = shoulder_pos[ii][0] * (cos(PI * rot_angle / 180) - 1) - shoulder_pos[ii][1] * sin(PI * rot_angle / 180) + trans_x;
        temp_trans[ii][1] = shoulder_pos[ii][0] * sin(PI * rot_angle / 180) + shoulder_pos[ii][1] * (cos(PI * rot_angle / 180) - 1) + trans_y;

        feet_start[ii][0] = shoulder_mult[ii][0] * (start_pos.legs[ii][1] * cos(PI * start_pos.legs[ii][0] / 180));
        feet_start[ii][1] = shoulder_mult[ii][1] * (start_pos.legs[ii][1] * sin(PI * start_pos.legs[ii][0] / 180));

        temp[ii][0] = shoulder_mult[ii][0] * ((feet_start[ii][0] - temp_trans[ii][0]) * cos(PI * rot_angle / 180) + (feet_start[ii][1] - temp_trans[ii][1]) * sin(PI * rot_angle / 180));
        temp[ii][1] = shoulder_mult[ii][1] * ((feet_start[ii][1] - temp_trans[ii][1]) * cos(PI * rot_angle / 180) - (feet_start[ii][0] - temp_trans[ii][0]) * sin(PI * rot_angle / 180));

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
            temp_final_pos[ii][0] = 180 * atan(temp[ii][1] / temp[ii][0]) / PI; // Dans le cas général, tan(theta) = y/x.
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
        if (leg_order[jj] == 1 || leg_order[jj] == 2)
        {
            time_leg_starts_lifting = sections[3 * (leg_order[jj] - 1) + 2];
            time_leg_touches_ground_again = sections[3 * (leg_order[jj] - 1 + 1) + 1] - 1;
        }
        else
        {
            time_leg_starts_lifting = nb_iter + 1;
            time_leg_touches_ground_again = time_leg_starts_lifting;
        }

        if (ii < time_leg_starts_lifting)
        {
            for (int kk = 0; kk < 3; kk++)
            {
                temp[jj][kk] = temp_current_pos.legs[jj][kk];
            }
        }
        else if (ii >= time_leg_starts_lifting && ii < time_leg_touches_ground_again)
        {
            for (int kk = 0; kk < 3; kk++)
            {
                temp[jj][kk] = get_lifted_position(jj, temp_current_pos, temp_future_pos, ii, time_leg_starts_lifting, time_leg_touches_ground_again)[kk];
            }
        }
        else if (ii >= time_leg_touches_ground_again && ii <= nb_iter)
        {
            for (int kk = 0; kk < 3; kk++)
            {
                temp[jj][kk] = temp_future_pos.legs[jj][kk];
            }
        }
    }
    Droideka_Position result = Droideka_Position(temp);
    return result;
}

Droideka_Position Droideka_Movement::get_final_position(Droideka_Position start_pos)
{
    float temp[LEG_NB][3];
    float temp_params[4];
    Droideka_Position temp_pos = get_future_position(start_pos, params[0][TIME_SAMPLE - 1], params[1][TIME_SAMPLE - 1], params[2][TIME_SAMPLE - 1], params[3][TIME_SAMPLE - 1]);
    if (type == STABLE_GAIT || type == TROT_GAIT)
    {
        if (seq == STARTING_SEQUENCE)
        {
            temp_params[0] = -params[0][TIME_SAMPLE - 1] * cos(PI * params[3][TIME_SAMPLE - 1] / 180) - params[1][TIME_SAMPLE - 1] * sin(PI * params[3][TIME_SAMPLE - 1] / 180);
            temp_params[1] = params[0][TIME_SAMPLE - 1] * sin(PI * params[3][TIME_SAMPLE - 1] / 180) - params[1][TIME_SAMPLE - 1] * cos(PI * params[3][TIME_SAMPLE - 1] / 180);
            temp_params[2] = -params[2][TIME_SAMPLE - 1];
            temp_params[3] = -params[3][TIME_SAMPLE - 1];
            for (int ii = 0; ii < LEG_NB; ii++)
            {
                if (leg_order[ii] == 1 || leg_order[ii] == 2) // On ne bouge que les deux premières jambes.
                {
                    for (int jj = 0; jj < 3; jj++)
                    {
                        temp[ii][jj] = get_future_position(default_position, temp_params[0], temp_params[1], temp_params[2], temp_params[3]).legs[ii][jj];
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
            temp_params[0] = -params[0][TIME_SAMPLE - 1] / 2.0 * cos(PI * params[3][TIME_SAMPLE - 1] / 2.0 / 180) - params[1][TIME_SAMPLE - 1] / 2.0 * sin(PI * params[3][TIME_SAMPLE - 1] / 2.0 / 180);
            temp_params[1] = params[0][TIME_SAMPLE - 1] / 2.0 * sin(PI * params[3][TIME_SAMPLE - 1] / 2.0 / 180) - params[1][TIME_SAMPLE - 1] / 2.0 * cos(PI * params[3][TIME_SAMPLE - 1] / 2.0 / 180);
            temp_params[2] = -params[2][TIME_SAMPLE - 1] / 2.0;
            temp_params[3] = -params[3][TIME_SAMPLE - 1] / 2.0;
            for (int ii = 0; ii < LEG_NB; ii++)
            {
                if (leg_order[ii] == 1 || leg_order[ii] == 2) // On ne bouge que les deux premières jambes.
                {
                    for (int jj = 0; jj < 3; jj++)
                    {
                        temp[ii][jj] = get_future_position(default_position, temp_params[0], temp_params[1], temp_params[2], temp_params[3]).legs[ii][jj];
                    }
                }
                else
                {
                    for (int jj = 0; jj < 3; jj++)
                    {
                        temp[ii][jj] = temp_pos.legs[ii][jj];
                        // temp[ii][jj] = get_future_position(default_position, params[0][TIME_SAMPLE - 1] / 2.0, params[1][TIME_SAMPLE - 1] / 2.0, params[2][TIME_SAMPLE - 1], params[3][TIME_SAMPLE - 1]).legs[ii][jj];
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
    else
    {
        return temp_pos;
    }
}

float *Droideka_Movement::get_lifted_position(int8_t leg, Droideka_Position debut_pos, Droideka_Position fin_pos, int time_, int time_start_lifting, int time_end_lifting)
{
    static float res[3];
    // float debut_time = ((float)leg_order[leg] - 1) * (float)TIME_SAMPLE / ((float)nb) + (float)delta_time;
    // float fin_time = (float)leg_order[leg] * (float)TIME_SAMPLE / ((float)nb) - 1;
    float debut_time = (float)time_start_lifting;
    float fin_time = (float)time_end_lifting;
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
    // float M_default[LEG_NB][2];

    // start_position.print_position("Start position");

    if (seq == INTERMEDIATE_SEQUENCE)
    {
    }
    else
    {
        deplacement[0] = deplacement[0] / 2.0;
        deplacement[1] = deplacement[1] / 2.0;
        rotation = rotation / 2.0;
    }

    for (int8_t ii = 0; ii < LEG_NB; ii++)
    {
        index = leg_order[ii] - 1;

        M[index][0] = shoulder_pos[ii][0] + shoulder_mult[ii][0] * start_position.legs[ii][1] * cos(PI * start_position.legs[ii][0] / 180.0);
        M[index][1] = shoulder_pos[ii][1] + shoulder_mult[ii][1] * start_position.legs[ii][1] * sin(PI * start_position.legs[ii][0] / 180.0);
        M_prime[index][0] = M[index][0] * cos(PI * rotation / 180) - M[index][1] * sin(PI * rotation / 180) + deplacement[0];
        M_prime[index][1] = M[index][1] * cos(PI * rotation / 180) + M[index][0] * sin(PI * rotation / 180) + deplacement[1];
        if (seq == INTERMEDIATE_SEQUENCE)
        {
            M_prime[index][0] = M[index][0] * cos(PI * rotation / 180) - M[index][1] * sin(PI * rotation / 180) + 2.0 * deplacement[0];
            M_prime[index][1] = M[index][1] * cos(PI * rotation / 180) + M[index][0] * sin(PI * rotation / 180) + 2.0 * deplacement[1];
        }
    }

    // // Block 1
    // for (int8_t ii = 0; ii < LEG_NB; ii++)
    // {
    //     for (int8_t jj = 0; jj < 2; jj++)
    //     {
    //         Serial.print(ii);
    //         Serial.print("\t");
    //         Serial.print(jj);
    //         Serial.print("\t\t");
    //         Serial.print(M[ii][jj]);
    //         Serial.print("\t\t");
    //         Serial.print(M_prime[ii][jj]);
    //         Serial.println();
    //     }
    // }

    for (int8_t jj = 0; jj < 2; jj++)
    {
        cog[0][jj] = 0.0;
        cog[1][jj] = (M[1][jj] + M[2][jj]) / 2 + factor * (M[3][jj] - (M[2][jj] + M[1][jj]) / 2);
        cog[2][jj] = (M_prime[0][jj] + M[3][jj]) / 2 + factor * (M[2][jj] - (M_prime[0][jj] + M[3][jj]) / 2);
        cog[3][jj] = deplacement[jj];
    }

    float distances[NB];
    float total_distance = 0.0;
    for (int ii = 0; ii < nb; ii++)
    {
        distances[ii] = sqrt((cog[ii + 1][0] - cog[ii][0]) * (cog[ii + 1][0] - cog[ii][0]) + (cog[ii + 1][1] - cog[ii][1]) * (cog[ii + 1][1] - cog[ii][1]));
        total_distance += distances[ii];
    }
    float alpha1, alpha2, alpha3;
    alpha1 = 2.5 * distances[0] / total_distance;
    alpha2 = alpha1 + 2.5 * distances[1] / total_distance;
    alpha3 = alpha2 + 2.5 * distances[2] / total_distance;

    // Block 2
    // Serial.println("COG coordinates");
    // for (int8_t ii = 0; ii < nb + 1; ii++)
    // {
    //     Serial.print(cog[ii][0]);
    //     Serial.print("\t");
    //     Serial.print(cog[ii][1]);
    //     Serial.println();
    // }

    if (seq == STARTING_SEQUENCE)
    {
        sections[0] = 0;
        sections[1] = sections[0];
        sections[2] = alpha1 * delta_time + 0.0 * lifting_leg_time;
        sections[3] = alpha1 * delta_time + 0.5 * lifting_leg_time;
        sections[4] = alpha1 * delta_time + 1.0 * lifting_leg_time;
        sections[5] = alpha2 * delta_time + 1.0 * lifting_leg_time;
        sections[6] = alpha2 * delta_time + 1.5 * lifting_leg_time;
        sections[7] = alpha2 * delta_time + 2.0 * lifting_leg_time;
        sections[8] = alpha3 * delta_time + 2.0 * lifting_leg_time;
        sections[9] = sections[8];
    }
    else if (seq == FINISHING_SEQUENCE)
    {
        sections[0] = 0;
        sections[1] = sections[0];
        sections[2] = alpha1 * delta_time + 0.0 * lifting_leg_time;
        sections[3] = alpha1 * delta_time + 0.5 * lifting_leg_time;
        sections[4] = alpha1 * delta_time + 1.0 * lifting_leg_time;
        sections[5] = alpha2 * delta_time + 1.0 * lifting_leg_time;
        sections[6] = alpha2 * delta_time + 1.5 * lifting_leg_time;
        sections[7] = alpha2 * delta_time + 2.0 * lifting_leg_time;
        sections[8] = alpha3 * delta_time + 2.0 * lifting_leg_time;
        sections[9] = sections[8];
    }
    else if (seq == INTERMEDIATE_SEQUENCE)
    {
        sections[0] = 0;
        sections[1] = sections[0];
        sections[2] = alpha1 * delta_time + 0.0 * lifting_leg_time;
        sections[3] = alpha1 * delta_time + 0.5 * lifting_leg_time;
        sections[4] = alpha1 * delta_time + 1.0 * lifting_leg_time;
        sections[5] = alpha2 * delta_time + 1.0 * lifting_leg_time;
        sections[6] = alpha2 * delta_time + 1.5 * lifting_leg_time;
        sections[7] = alpha2 * delta_time + 2.0 * lifting_leg_time;
        sections[8] = alpha3 * delta_time + 2.0 * lifting_leg_time;
        sections[9] = sections[8];
    }
    for (unsigned int ii = 0; ii < nb_iter; ii++)
    {
        time_iter[ii] = (ii + 1) * time_span / nb_iter;
    }

    float t;
    for (int jj = sections[0]; jj < sections[9]; jj++)
    {
        t = 3.0 * ((float)jj + 1.0) / ((float)TIME_SAMPLE) + 1.0;
        params[0][jj] = cog[0][0] * P(1, t) + cog[1][0] * P(2, t) + cog[2][0] * P(3, t) + cog[3][0] * P(4, t);
        params[1][jj] = cog[0][1] * P(1, t) + cog[1][1] * P(2, t) + cog[2][1] * P(3, t) + cog[3][1] * P(4, t);
        params[2][jj] = 0;
        params[3][jj] = rotation * ((float)jj + 1) / ((float)sections[9]);
        reverse_params[0][jj] = params[0][jj] - deplacement[0];
        reverse_params[1][jj] = params[1][jj] - deplacement[1];
        reverse_params[2][jj] = 0;
        reverse_params[3][jj] = params[3][jj] - rotation;
    }

    // for (int8_t ii = 0; ii < nb_iter; ii++)
    // {
    //     Serial.print(params[0][ii]);
    //     Serial.print("\t\t");
    //     Serial.print(params[1][ii]);
    //     Serial.print("\t\t");
    //     Serial.print(reverse_params[0][ii]);
    //     Serial.print("\t\t");
    //     Serial.print(reverse_params[1][ii]);
    //     Serial.print("\t\t");
    //     Serial.print(params[3][ii]);
    //     Serial.print("\t\t");
    //     Serial.println(reverse_params[3][ii]);
    // }
}

float Droideka_Movement::P(int n, float t)
{
    float t1, t2, t3, div;
    switch (n)
    {
    case 1:
        t1 = 2.0;
        t2 = 3.0;
        t3 = 4.0;
        div = -1.0 / 6.0;
        break;
    case 2:
        t1 = 1.0;
        t2 = 3.0;
        t3 = 4.0;
        div = 1.0 / 2.0;
        break;
    case 3:
        t1 = 1.0;
        t2 = 2.0;
        t3 = 4.0;
        div = -1.0 / 2.0;
        break;
    case 4:
        t1 = 1.0;
        t2 = 2.0;
        t3 = 3.0;
        div = 1.0 / 6.0;
        break;
    default:
        Serial.println("ERROR: " + String(n) + " is not equal to 1, 2, 3 or 4!!!");
        while (true)
        {
        }
        break;
    }
    return div * (t - t1) * (t - t2) * (t - t3);
}

void Droideka_Movement::trotting_movement()
{
    if (seq == INTERMEDIATE_SEQUENCE)
    {
    }
    else
    {
        deplacement[0] = deplacement[0] / 2.0;
        deplacement[1] = deplacement[1] / 2.0;
        rotation = rotation / 2.0;
    }

    sections[0] = 0;
    sections[1] = sections[0];
    sections[2] = sections[1];
    sections[3] = sections[2];
    sections[4] = 2.5 * delta_time + 2.0 * lifting_leg_time;
    sections[5] = sections[4];
    sections[6] = sections[5];
    sections[7] = sections[6];
    sections[8] = sections[7];
    sections[9] = sections[8];

    for (unsigned int ii = 0; ii < nb_iter; ii++)
    {
        time_iter[ii] = (ii + 1) * time_span / nb_iter;
    }

    for (int jj = sections[0]; jj < sections[9]; jj++)
    {
        params[0][jj] = deplacement[0] * ((float)jj + 1) / ((float)sections[9]);
        params[1][jj] = deplacement[1] * ((float)jj + 1) / ((float)sections[9]);
        params[2][jj] = 0;
        params[3][jj] = rotation * ((float)jj + 1) / ((float)sections[9]);
        reverse_params[0][jj] = params[0][jj] - deplacement[0];
        reverse_params[1][jj] = params[1][jj] - deplacement[1];
        reverse_params[2][jj] = 0;
        reverse_params[3][jj] = params[3][jj] - rotation;
    }
}

bool Droideka_Movement::compare_directions()
{
    float limits[8][2] = {{PI, 3.0 * PI / 4},
                          {3.0 * PI / 4, PI / 2},
                          {PI / 2, PI / 4},
                          {PI / 4, 0.0},
                          {0.0, -PI / 4},
                          {-PI / 4, -PI / 2},
                          {-PI / 2, -3.0 * PI / 4},
                          {-3.0 * PI / 4, -PI}};
    for (int ii = 0; ii < 8; ii++)
    {
        if (direction >= limits[ii][1] && direction < limits[ii][0])
        {
            if (last_direction >= limits[ii][1] && last_direction < limits[ii][0])
            {
                return true;
            }
        }
    }
    return false;
}

void Droideka_Movement::keep_going()
{
    if (seq == STARTING_SEQUENCE || seq == INTERMEDIATE_SEQUENCE)
    {
        if (next_seq == INTERMEDIATE_SEQUENCE || next_seq == FINISHING_SEQUENCE)
        {
            // Serial.println("Keep going!");
            // Serial.print(seq);
            // Serial.print("\t");
            // Serial.println(next_seq);
            if (next_seq == FINISHING_SEQUENCE)
            {
                next_longitudinal = longitudinal;
                next_lateral = lateral;
                next_angle = angle;
            }
            longitudinal = next_longitudinal;
            lateral = next_lateral;
            angle = next_angle;
            // Serial.print("Next long : ");
            // Serial.print(longitudinal);
            // Serial.print("\tNext lat : ");
            // Serial.print(lateral);
            // Serial.print("\tNext ang : ");
            // Serial.println(angle);
            last_direction = direction;
            establish_deplacement(longitudinal, lateral, angle);
            // Serial.print("Next dep X : ");
            // Serial.print(deplacement[0]);
            // Serial.print("\tNext dep Y : ");
            // Serial.print(deplacement[1]);
            // Serial.print("\tNext rot : ");
            // Serial.println(rotation);
            // Serial.print("Next Direction : ");
            // Serial.println(direction * 180 / PI);
            if (compare_directions() == true)
            {
                for (int ii = 0; ii < LEG_NB; ii++)
                {
                    leg_order[ii] = leg_order[ii] - 2;
                    if (leg_order[ii] <= 0)
                    {
                        leg_order[ii] += 4;
                    }
                }
            }
            else
            {
                establish_legs_order(direction);
            }
            started = false;
            finished = false;
            seq = next_seq;
            if (next_seq == INTERMEDIATE_SEQUENCE)
            {
                next_seq = INTERMEDIATE_SEQUENCE;
                next_longitudinal = longitudinal;
                next_lateral = lateral;
                next_angle = angle;
            }
            else
            {
                next_seq = STARTING_SEQUENCE;
            }
            start_position = end_position;
            iter = 0;
            start = 0;

            if (type == STABLE_GAIT)
            {
                stable_movement();
            }
            else if (type == TROT_GAIT)
            {
                trotting_movement();
            }
            end_position = get_final_position(start_position);
            // end_position.print_position("End Position keep going");
        }
    }
}