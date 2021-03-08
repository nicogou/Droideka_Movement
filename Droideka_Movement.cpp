#include <Droideka_Movement.h>

Droideka_Movement::Droideka_Movement() {}

Droideka_Movement::Droideka_Movement(Droideka_Position start_position_, int16_t throttle_longitudinal, int16_t throttle_lateral, int16_t throttle_vertical, int16_t throttle_angle, bool lifting_legs)
{
    start_position = start_position_;
    if (!lifting_legs)
    {
        establish_cog_movement(throttle_longitudinal, throttle_lateral, throttle_vertical, throttle_angle);
    }
    else
    {
        establish_cog_movement(0, 0);
    }
    end_position = get_final_position(start_position);
    //establish_legs_movement(lifting_legs);
}

Droideka_Movement::Droideka_Movement(Droideka_Position start_position_, float trans_x[TIME_SAMPLE], float trans_y[TIME_SAMPLE], float trans_z[TIME_SAMPLE], float rot_angle[TIME_SAMPLE], unsigned long span)
{
    type = CENTER_OF_GRAVITY_TRAJ;

    start_position = start_position_;
    time_span = span * 1000;

    for (int ii = 0; ii < nb_iter; ii++)
    {
        param1[ii] = trans_x[ii];
        param2[ii] = trans_y[ii];
        param3[ii] = trans_z[ii];
        param4[ii] = rot_angle[ii];

        time_iter[ii] = (ii + 1) * time_span / nb_iter;
    }
}

Droideka_Movement::Droideka_Movement(Droideka_Position start_position_, float theta[TIME_SAMPLE], float rho[TIME_SAMPLE], float height[TIME_SAMPLE], unsigned long span, int one_leg = -1)
{
    type = LEGS_TRAJ;

    start_position = start_position_;
    leg_id = one_leg;
    time_span = span * 1000;

    for (int ii = 0; ii < nb_iter; ii++)
    {
        param1[ii] = theta[ii];
        param2[ii] = rho[ii];
        param3[ii] = height[ii];
        param4[ii] = 0;

        time_iter[ii] = (ii + 1) * time_span / nb_iter;
    }
}

Droideka_Movement::Droideka_Movement(Droideka_Position start_position_, Droideka_Position end_position_, int which_leg, unsigned long span)
{
    type = DIRECT_FOOT_MVMT;

    start_position = start_position_;

    for (int ii = 0; ii < TIME_SAMPLE; ii++)
    {
        param1[ii] = 0;
        param2[ii] = 0;
        param3[ii] = 0;
        param4[ii] = 0;

        time_iter[ii] = 0;
    }
    nb_iter = 0;
    add_position(end_position_, which_leg, span);
}

void Droideka_Movement::add_position(Droideka_Position pos, int which_leg, unsigned long span)
{
    if (type == DIRECT_FOOT_MVMT)
    {
        param1[nb_iter] = pos.legs[which_leg][0];
        param2[nb_iter] = pos.legs[which_leg][1];
        param3[nb_iter] = pos.legs[which_leg][2];
        time_span += span * 1000;
        time_iter[nb_iter] = time_span;
        nb_iter++;
    }
}

ErrorCode Droideka_Movement::establish_cog_movement(int16_t throttle_longitudinal, int16_t throttle_lateral, int16_t throttle_vertical, int16_t throttle_angle)
{
    for (int ii = 0; ii < TIME_SAMPLE; ii++)
    {
        param1[ii] = ((float)throttle_lateral - 105.0) * (2.0 - (-2.0)) / (792.0 - 105.0) + -2.0;
        param2[ii] = ((float)throttle_longitudinal - 831.0) * (2.0 - (-2.0)) / (140.0 - 831.0) + -2.0;
        param3[ii] = ((float)throttle_vertical - 825.0) * (-2.0 - (2.0)) / (137.0 - 825.0) + 2.0;
        param4[ii] = ((float)throttle_angle - 185.0) * (20.0 - (-20.0)) / (863.0 - 185.0) + -20.0;
    }
    // for (int ii = 0; ii < TIME_SAMPLE; ii++)
    // {
    //     param2[ii] = 3.0 * ((float)ii + 1.0) / (float)TIME_SAMPLE;
    //     param1[ii] = -2.5 * sin(2 * 3.141592 * param2[ii] / 4.0);
    //     param3[ii] = Y_TOUCHING;
    //     param4[ii] = 0;
    // }
    leg_order[3] = 1;
    leg_order[1] = 2;
    leg_order[2] = 3;
    leg_order[0] = 4;

    moving_leg_nb = 4;
    delta_time = TIME_SAMPLE / (moving_leg_nb * 2);
    // }

    for (int ii = 0; ii < TIME_SAMPLE; ii++)
    {
        reverse_param1[ii] = -param1[ii];
        reverse_param2[ii] = -param2[ii];
        reverse_param3[ii] = -param3[ii];
        reverse_param4[ii] = -param4[ii];
    }

    return NO_ERROR;
}

ErrorCode Droideka_Movement::establish_cog_movement(int throttle_longitudinal, int throttle_lateral)
{
    // if (throttle_longitudinal > 0 && abs(throttle_longitudinal) > abs(throttle_lateral))
    // // Moving forward.
    // {
    // for (int ii = 0; ii < TIME_SAMPLE; ii++)
    // {
    //     param1[ii] = 0;
    //     param2[ii] = 0;
    //     param3[ii] = 2.0 * ((float)ii + 1.0) / (float)TIME_SAMPLE;
    //     param4[ii] = 0;
    // }
    for (int ii = 0; ii < TIME_SAMPLE; ii++)
    {
        param2[ii] = 4.0 * ((float)ii + 1.0) / (float)TIME_SAMPLE;
        param1[ii] = -2.5 * sin(2 * 3.141592 * param2[ii] / 4.0);
        param3[ii] = 0;
        param4[ii] = 0;
    }
    leg_order[3] = 1;
    leg_order[1] = 2;
    leg_order[2] = 3;
    leg_order[0] = 4;

    moving_leg_nb = 4;
    delta_time = TIME_SAMPLE / (moving_leg_nb * 2);
    // }

    for (int ii = 0; ii < TIME_SAMPLE; ii++)
    {
        reverse_param1[ii] = -param1[ii];
        reverse_param2[ii] = -param2[ii];
        reverse_param3[ii] = -param3[ii];
        reverse_param4[ii] = -param4[ii];
    }

    return NO_ERROR;
}

Droideka_Position Droideka_Movement::get_future_position(Droideka_Position start_pos, int ii)
{
    if (type == CENTER_OF_GRAVITY_TRAJ)
    {
        return get_future_position(start_pos, param1[ii], param2[ii], param3[ii], param4[ii]);
    }
    else if (type == LEGS_TRAJ)
    {
        return get_future_position(param1[ii], param2[ii], param3[ii], leg_id);
    }
    else if (type == DIRECT_FOOT_MVMT)
    {
        return get_future_position(ii);
    }
}

Droideka_Position Droideka_Movement::get_future_position(int iteration)
{
    float temp[LEG_NB][3];
    for (int ii = 0; ii < LEG_NB; ii++)
    {

        temp[ii][0] = param1[iteration];
        temp[ii][1] = param2[iteration];
        temp[ii][2] = param3[iteration];
        Serial.println(ii);
        Serial.println(temp[ii][0]);
        Serial.println(temp[ii][1]);
        Serial.println(temp[ii][2]);
    }
    Droideka_Position final_pos(temp);
    return final_pos;
}

Droideka_Position Droideka_Movement::get_future_position(float theta, float rho, float height, int one_leg = -1)
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

Droideka_Position Droideka_Movement::get_final_position(Droideka_Position start_pos)
{
    return get_future_position(start_pos, param1[TIME_SAMPLE - 1], param2[TIME_SAMPLE - 1], param3[TIME_SAMPLE - 1], param4[TIME_SAMPLE - 1]);
}

// Droideka_Position Droideka_Movement::get_lifted_position(int leg, Droideka_Position debut_pos, Droideka_Position fin_pos, int time_)
// {
//     float debut_time = ((float)leg_order[leg] - 1) * (float)TIME_SAMPLE / (float)moving_leg_nb + (float)delta_time;
//     float fin_time = (float)leg_order[leg] * (float)TIME_SAMPLE / (float)moving_leg_nb - 1;
//     float interval_time = (float)fin_time - (float)debut_time;
//     float time_from_lifting = (float)time_ - (float)debut_time;

//     // Between the lifting and putting back of the leg, theta and X are linear, wheras Y follows a quadratic curve (arbitrarily defined)

//     float temp[LEG_NB][3];
//     for (int ii = 0; ii < 2; ii++)
//     {
//         temp[leg][ii] = (fin_pos.legs[leg][ii] - debut_pos.legs[leg][ii]) / interval_time * time_from_lifting + debut_pos.legs[leg][ii];
//     }
//     temp[leg][2] = Y_NOT_TOUCHING - (Y_NOT_TOUCHING - Y_TOUCHING) * ((time_from_lifting - interval_time / 2) / (interval_time / 2)) * ((time_from_lifting - interval_time / 2) / (interval_time / 2));

//     Droideka_Position result(temp);
//     return result;
// }

// ErrorCode Droideka_Movement::establish_legs_movement(bool lifting_legs)
// {
//     float temp[LEG_NB][3];
//     unsigned long time_leg_starts_lifting;
//     unsigned long time_leg_touches_ground_again;

//     for (int ii = 0; ii < TIME_SAMPLE; ii++)
//     {
//         Droideka_Position temp_current_pos = get_future_position(start_position, param1[ii], param2[ii], param3[ii], param4[ii]); // Calculates the position of the legs before the leg is lifted.

//         if (lifting_legs)
//         {
//             Droideka_Position temp_future_pos = get_future_position(start_position, reverse_param1[TIME_SAMPLE - 1 - ii], reverse_param2[TIME_SAMPLE - 1 - ii], reverse_param3[TIME_SAMPLE - 1 - ii], reverse_param4[TIME_SAMPLE - 1 - ii]); // Calculates the position of the legs after the leg has been lifted and put back on the ground.
//             for (int jj = 0; jj < LEG_NB; jj++)
//             {
//                 time_leg_starts_lifting = (leg_order[jj] - 1) * TIME_SAMPLE / moving_leg_nb + delta_time;
//                 time_leg_touches_ground_again = (leg_order[jj]) * TIME_SAMPLE / moving_leg_nb - 1;

//                 if (ii < time_leg_starts_lifting)
//                 {
//                     for (int kk = 0; kk < 3; kk++)
//                     {
//                         temp[jj][kk] = temp_current_pos.legs[jj][kk];
//                     }
//                 }
//                 else if (ii >= time_leg_starts_lifting && ii < time_leg_touches_ground_again)
//                 {
//                     for (int kk = 0; kk < 3; kk++)
//                     {
//                         temp[jj][kk] = get_lifted_position(jj, temp_current_pos, temp_future_pos, ii).legs[jj][kk];
//                     }
//                 }
//                 else if (ii >= time_leg_touches_ground_again && ii <= TIME_SAMPLE)
//                 {
//                     for (int kk = 0; kk < 3; kk++)
//                     {
//                         temp[jj][kk] = temp_future_pos.legs[jj][kk];
//                     }
//                 }
//             }
//             positions[ii] = Droideka_Position(temp);
//         }
//         else
//         {
//             positions[ii] = temp_current_pos;
//         }
//     }
//     return NO_ERROR;
// }