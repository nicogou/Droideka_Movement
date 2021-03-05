#include <Droideka_Movement.h>

Droideka_Movement::Droideka_Movement() {}

Droideka_Movement::Droideka_Movement(Droideka_Position start_position_, int throttle_longitudinal, int throttle_lateral)
{
    start_position = start_position_;
    establish_cog_movement(throttle_longitudinal, throttle_lateral);
    // end_position = get_final_position(start_position);
    // establish_legs_movement();
    // valid_movement = true;
    // for (int ii = 0; ii < TIME_SAMPLE; ii++)
    // {
    //     valid_movement *= positions[ii].valid_position; //If one of the positions in the movement is not valid, then the movement is invalid.
    // }
    // start_walk_time = micros();

    // for (int ii = 0; ii < TIME_SAMPLE; ii++)
    // {
    //     Serial.println(ii);
    //     for (int jj = 0; jj < LEG_NB; jj++)
    //     {
    //         for (int kk = 0; kk < 3; kk++)
    //         {
    //             Serial.print(positions[ii].legs[jj][kk]);
    //             Serial.print("\t");
    //         }
    //         Serial.println();
    //     }
    //     Serial.print("Valid : ");
    //     Serial.println(positions[ii].valid_position);
    //     Serial.println();
    //     Serial.println();
    // }
}

ErrorCode Droideka_Movement::establish_cog_movement(int throttle_longitudinal, int throttle_lateral)
{
    // if (throttle_longitudinal > 0 && abs(throttle_longitudinal) > abs(throttle_lateral))
    // // Moving forward.
    // {
    //     for (int ii = 0; ii < TIME_SAMPLE; ii++)
    //     {
    //         tx[ii] = 2 * sqrt(2) * (ii + 1) / TIME_SAMPLE; //MAX_LONGITUDINAL_COG_MOVE * ii / TIME_SAMPLE;
    //         ty[ii] = 0;
    //         alpha[ii] = 0;
    //     }
    //     leg_order[3] = 1;
    //     leg_order[1] = 2;
    //     leg_order[2] = 3;
    //     leg_order[0] = 4;

    //     for (int ii = 0; ii < LEG_NB; ii++)
    //     {
    //         leg_lifted[ii] = false;
    //     }
    //     moving_leg_nb = 4;
    //     delta_time = TIME_SAMPLE / (moving_leg_nb * 4);
    // }

    // for (int ii = 0; ii < TIME_SAMPLE; ii++)
    // {
    //     reverse_tx[ii] = tx[TIME_SAMPLE - ii - 1] * -1;
    //     reverse_ty[ii] = ty[TIME_SAMPLE - ii - 1] * -1;
    //     reverse_alpha[ii] = alpha[TIME_SAMPLE - ii - 1] * -1;
    // }

    for (int ii = 0; ii < TIME_SAMPLE; ii++)
    {
        float t = (float)ii + 1.0;
        float d = 2.0;
        float T = (float)TIME_SAMPLE;
        float sep = 10.0;
        if (ii < TIME_SAMPLE / sep)
        {
            tx[ii] = 0;
            ty[ii] = sep * d * t / T;
        }
        if (ii >= TIME_SAMPLE / sep && ii < 2 * TIME_SAMPLE / sep)
        {
            tx[ii] = -sep * d / T * (t - TIME_SAMPLE / sep);
            ty[ii] = d;
        }
        if (ii >= 2 * TIME_SAMPLE / sep && ii < 4 * TIME_SAMPLE / sep)
        {
            tx[ii] = -d;
            ty[ii] = -sep * d / T * (t - 3 * TIME_SAMPLE / sep);
        }
        if (ii >= 4 * TIME_SAMPLE / sep && ii < 6 * TIME_SAMPLE / sep)
        {
            tx[ii] = sep * d / T * (t - 5 * TIME_SAMPLE / sep);
            ty[ii] = -d;
        }
        if (ii >= 6 * TIME_SAMPLE / sep && ii < 8 * TIME_SAMPLE / sep)
        {
            tx[ii] = d;
            ty[ii] = sep * d / T * (t - 7 * TIME_SAMPLE / sep);
        }
        if (ii >= 8 * TIME_SAMPLE / sep && ii < 9 * TIME_SAMPLE / sep)
        {
            tx[ii] = -sep * d / T * (t - 9 * TIME_SAMPLE / sep);
            ty[ii] = d;
        }
        if (ii >= 9 * TIME_SAMPLE / sep && ii < 10 * TIME_SAMPLE / sep)
        {
            tx[ii] = 0;
            ty[ii] = -sep * d / T * (t - 10 * TIME_SAMPLE / sep);
        }
        alpha[ii] = 0;
    }

    return NO_ERROR;
}

ErrorCode Droideka_Movement::establish_cog_movement_advanced(int throttle_longitudinal, int throttle_lateral, int throttle_angle)
{

    float move_x = throttle_lateral * MAX_LONGITUDINAL_COG_MOVE / 100; // throttle_longitudinal is between -100 and 100.
    float move_y = throttle_longitudinal * MAX_LATERAL_COG_MOVE / 100; // throttle_lateral is between -100 and 100.
    float move_angle = throttle_angle * MAX_ANGLE_COG_MOVE / 100;      // MAX_ANGLE_COG to be determined.

    if (move_angle == 0 || move_y == 0)
    {
        for (int ii = 0; ii < TIME_SAMPLE; ii++)
        {
            ty[ii] = move_y * ii / TIME_SAMPLE;
            tx[ii] = move_x * ii / TIME_SAMPLE;
            alpha[ii] = move_angle * ii / TIME_SAMPLE;
        }
    }
    else
    {
        for (int ii = 0; ii < TIME_SAMPLE; ii++)
        {
            ty[ii] = move_x * ii / TIME_SAMPLE;
            tx[ii] = ty[ii] * move_x / move_y + ty[ii] / move_y * (1 - ty[ii] / move_y) * (ty[ii] * (2 * move_x / move_y - tan(PI / 2 + move_angle)) - move_x);
            alpha[ii] = move_angle * ii / TIME_SAMPLE;
        }
    }

    if (abs(throttle_longitudinal) > abs(throttle_lateral) && throttle_lateral > 0 && throttle_longitudinal > 0)
    {
        leg_order[2] = 1;
        leg_order[0] = 2;
        leg_order[3] = 3;
        leg_order[1] = 4;
    }
    else if (abs(throttle_longitudinal) > abs(throttle_lateral) && throttle_lateral < 0 && throttle_longitudinal > 0)
    {
        leg_order[3] = 1;
        leg_order[1] = 2;
        leg_order[2] = 3;
        leg_order[0] = 4;
    }
    else if (abs(throttle_longitudinal) > abs(throttle_lateral) && throttle_lateral > 0 && throttle_longitudinal < 0)
    {
        leg_order[0] = 1;
        leg_order[2] = 2;
        leg_order[1] = 3;
        leg_order[3] = 4;
    }
    else if (abs(throttle_longitudinal) > abs(throttle_lateral) && throttle_lateral < 0 && throttle_longitudinal < 0)
    {
        leg_order[1] = 1;
        leg_order[3] = 2;
        leg_order[0] = 3;
        leg_order[2] = 4;
    }
    else if (abs(throttle_longitudinal) < abs(throttle_lateral) && throttle_lateral > 0 && throttle_longitudinal > 0)
    {
        leg_order[2] = 1;
        leg_order[3] = 2;
        leg_order[0] = 3;
        leg_order[1] = 4;
    }
    else if (abs(throttle_longitudinal) < abs(throttle_lateral) && throttle_lateral < 0 && throttle_longitudinal > 0)
    {
        leg_order[3] = 1;
        leg_order[2] = 2;
        leg_order[1] = 3;
        leg_order[0] = 4;
    }
    else if (abs(throttle_longitudinal) < abs(throttle_lateral) && throttle_lateral > 0 && throttle_longitudinal < 0)
    {
        leg_order[0] = 1;
        leg_order[1] = 2;
        leg_order[2] = 3;
        leg_order[3] = 4;
    }
    else if (abs(throttle_longitudinal) < abs(throttle_lateral) && throttle_lateral < 0 && throttle_longitudinal < 0)
    {
        leg_order[1] = 1;
        leg_order[0] = 2;
        leg_order[3] = 3;
        leg_order[2] = 4;
    }

    for (int ii = 0; ii < LEG_NB; ii++)
    {
        leg_lifted[ii] = false;
    }
    moving_leg_nb = 4;
    delta_time = TIME_SAMPLE / (moving_leg_nb * 4);

    for (int ii = 0; ii < TIME_SAMPLE; ii++)
    {
        reverse_tx[ii] = tx[TIME_SAMPLE - ii - 1] * -1;
        reverse_ty[ii] = ty[TIME_SAMPLE - ii - 1] * -1;
        reverse_alpha[ii] = alpha[TIME_SAMPLE - ii - 1] * -1;
    }

    return NO_ERROR;
}

ErrorCode Droideka_Movement::establish_cog_movement_stable(int throttle_longitudinal, int throttle_lateral, int throttle_angle)
{

    float move_x = throttle_lateral * MAX_LONGITUDINAL_COG_MOVE / 100; // throttle_longitudinal is between -100 and 100.
    float move_y = throttle_longitudinal * MAX_LATERAL_COG_MOVE / 100; // throttle_lateral is between -100 and 100.
    float move_angle = throttle_angle * MAX_ANGLE_COG_MOVE / 100;      // MAX_ANGLE_COG_MOVE to be determined.

    establish_leg_order(throttle_longitudinal, throttle_lateral);

    if (establish_stableness(move_x, move_y, move_angle))
    {

        for (int ii = 0; ii < TIME_SAMPLE / 2; ii++)
        {
            ty[ii] = middle_point[1] * ii / (TIME_SAMPLE / 2);
            ty[ii + TIME_SAMPLE / 2] = middle_point[1] + (move_y - middle_point[1]) * ii / (TIME_SAMPLE / 2);
            tx[ii] = middle_point[0] * ii / TIME_SAMPLE;
            tx[ii + TIME_SAMPLE / 2] = middle_point[0] + (move_x - middle_point[0]) * ii / (TIME_SAMPLE / 2);
            alpha[ii] = move_angle * ii / TIME_SAMPLE;
            alpha[ii + TIME_SAMPLE / 2] = move_angle * ii / TIME_SAMPLE;
        }

        for (int ii = 0; ii < LEG_NB; ii++)
        {
            leg_lifted[ii] = false;
        }
        moving_leg_nb = 4;
        delta_time = TIME_SAMPLE / (moving_leg_nb * 4);

        for (int ii = 0; ii < TIME_SAMPLE; ii++)
        {
            reverse_tx[ii] = tx[TIME_SAMPLE - ii - 1] * -1;
            reverse_ty[ii] = ty[TIME_SAMPLE - ii - 1] * -1;
            reverse_alpha[ii] = alpha[TIME_SAMPLE - ii - 1] * -1;
        }

        return NO_ERROR;
    }
}

ErrorCode Droideka_Movement::establish_cog_movement_next_level(int throttle_longitudinal, int throttle_lateral, int throttle_angle)
{
    float move_x = throttle_lateral * MAX_LONGITUDINAL_COG_MOVE / 100; // throttle_longitudinal is between -100 and 100.
    float move_y = throttle_longitudinal * MAX_LATERAL_COG_MOVE / 100; // throttle_lateral is between -100 and 100.
    float move_angle;

    if (find_extreme_alpha(throttle_longitudinal, throttle_lateral, move_y, move_x))
    {
        if (throttle_angle >= 0)
        {
            move_angle = throttle_angle * alpha_max / 100;
        }
        else
        {
            move_angle = throttle_angle * alpha_min / 100;
        }
    }

    if (establish_stableness(move_x, move_y, move_angle))
    {
        for (int ii = 0; ii < TIME_SAMPLE / 2; ii++)
        {
            ty[ii] = middle_point[1] * ii / (TIME_SAMPLE / 2);
            ty[ii + TIME_SAMPLE / 2] = middle_point[1] + (move_y - middle_point[1]) * ii / (TIME_SAMPLE / 2);
            tx[ii] = middle_point[0] * ii / TIME_SAMPLE;
            tx[ii + TIME_SAMPLE / 2] = middle_point[0] + (move_x - middle_point[0]) * ii / (TIME_SAMPLE / 2);
            alpha[ii] = move_angle * ii / TIME_SAMPLE;
            alpha[ii + TIME_SAMPLE / 2] = move_angle * ii / TIME_SAMPLE;
        }

        for (int ii = 0; ii < LEG_NB; ii++)
        {
            leg_lifted[ii] = false;
        }
        moving_leg_nb = 4;
        delta_time = TIME_SAMPLE / (moving_leg_nb * 4);

        for (int ii = 0; ii < TIME_SAMPLE; ii++)
        {
            reverse_tx[ii] = tx[TIME_SAMPLE - ii - 1] * -1;
            reverse_ty[ii] = ty[TIME_SAMPLE - ii - 1] * -1;
            reverse_alpha[ii] = alpha[TIME_SAMPLE - ii - 1] * -1;
        }

        return NO_ERROR;
    }
}

bool Droideka_Movement::establish_stableness(float move_longitudinal, float move_lateral, float move_angle)
{
    float start_global_position[LEG_NB][2];
    float final_global_position[LEG_NB][2];
    float pairs[2][2][2]; // 0 = first pair, 1 = second pair, 3 = CoG deltas
    float abc[2][3];      // 0 = first pair, 1 = second pair, 3 = CoG deltas. 0 = a, 1 = b, 2 = c (ax + by + c = 0).
    float intersection[2];
    float mid_cog[2];
    float distance;

    for (int ii = 0; ii < LEG_NB; ii++)
    {
        start_global_position[ii][0] = shoulder_pos[ii][0] + shoulder_mult[ii][0] * start_position.legs[ii][1] * cos(start_position.legs[ii][0]);
        start_global_position[ii][1] = shoulder_pos[ii][1] + shoulder_mult[ii][1] * start_position.legs[ii][1] * sin(start_position.legs[ii][0]);
        final_global_position[ii][0] = move_longitudinal + start_global_position[ii][0] * cos(move_angle) - start_global_position[ii][1] * sin(move_angle);
        final_global_position[ii][1] = move_lateral + start_global_position[ii][0] * sin(move_angle) + start_global_position[ii][1] * cos(move_angle);
    }

    pairs[0][0][0] = final_global_position[first_leg_to_move][0];
    pairs[0][0][1] = final_global_position[first_leg_to_move][1];
    pairs[0][1][0] = start_global_position[3 - first_leg_to_move][0];
    pairs[0][1][1] = start_global_position[3 - first_leg_to_move][1];

    pairs[1][0][0] = 0;
    pairs[1][0][1] = 0;
    pairs[1][1][0] = move_longitudinal;
    pairs[1][1][1] = move_lateral;

    for (int ii = 0; ii < 2; ii++)
    {
        abc[ii][0] = -(pairs[ii][1][1] - pairs[ii][0][1]) / (pairs[ii][1][0] - pairs[ii][0][0]);
        abc[ii][1] = 1;
        abc[ii][2] = -pairs[ii][1][1] + (pairs[ii][1][1] - pairs[ii][0][1]) / (pairs[ii][1][0] - pairs[ii][0][0]) * pairs[ii][1][0];
    }

    mid_cog[0] = (pairs[1][0][0] + pairs[1][1][0]) / 2;
    mid_cog[1] = (pairs[1][0][1] + pairs[1][1][1]) / 2;

    intersection[0] = (abc[0][1] * abc[1][2] - abc[1][1] * abc[0][2]) / (abc[0][0] * abc[1][1] - abc[1][0] * abc[0][2]);
    intersection[1] = (abc[1][0] * abc[0][2] - abc[0][0] * abc[1][2]) / (abc[0][0] * abc[1][1] - abc[1][0] * abc[0][2]);

    distance = sqrt((mid_cog[0] - intersection[0]) * (mid_cog[0] - intersection[0]) + (mid_cog[1] - intersection[1]) * (mid_cog[1] - intersection[1]));

    if (intersection[0] < min(pairs[1][0][0], pairs[1][1][0]) || intersection[0] > max(pairs[1][0][0], pairs[1][1][0]) || intersection[1] < min(pairs[1][0][1], pairs[1][1][1]) || intersection[1] > max(pairs[1][0][1], pairs[1][1][1]))
    {
        // Unstable position
        middle_point[0] = 0;
        middle_point[1] = 0;
        stable_movement = false;
        return false;
    }
    else
    {
        // Stable position
        middle_point[0] = intersection[0];
        middle_point[1] = intersection[1];
        stable_movement = true;
        return true;
    }
}

bool Droideka_Movement::find_extreme_alpha(float throttle_longitudinal, float throttle_lateral, float move_longitudinal, float move_lateral)
{
    establish_leg_order(throttle_longitudinal, throttle_lateral);
    float start_global_position[LEG_NB][2];
    float distance_CoG_foot;
    float distance_foot_newCoG;
    float distance_CoG_newCoG;
    float temp_alpha;

    float move_x = throttle_lateral * MAX_LONGITUDINAL_COG_MOVE / 100; // throttle_longitudinal is between -100 and 100.
    float move_y = throttle_longitudinal * MAX_LATERAL_COG_MOVE / 100; // throttle_lateral is between -100 and 100.
    float temp_alpha_diag;

    for (int ii = 0; ii < LEG_NB; ii++)
    {
        start_global_position[ii][0] = shoulder_pos[ii][0] + shoulder_mult[ii][0] * start_position.legs[ii][1] * cos(start_position.legs[ii][0]);
        start_global_position[ii][1] = shoulder_pos[ii][1] + shoulder_mult[ii][1] * start_position.legs[ii][1] * sin(start_position.legs[ii][0]);
    }

    distance_CoG_foot = sqrt(start_global_position[2 - first_leg_to_move][0] * start_global_position[2 - first_leg_to_move][0] + start_global_position[2 - first_leg_to_move][1] * start_global_position[2 - first_leg_to_move][1]);
    distance_CoG_newCoG = sqrt(move_lateral * move_lateral + move_longitudinal * move_longitudinal);
    distance_foot_newCoG = sqrt((move_lateral - start_global_position[2 - first_leg_to_move][0]) * (move_lateral - start_global_position[2 - first_leg_to_move][0]) + (move_longitudinal - start_global_position[2 - first_leg_to_move][1]) * (move_longitudinal - start_global_position[2 - first_leg_to_move][1]));

    temp_alpha = acos((distance_CoG_newCoG * distance_CoG_newCoG - distance_foot_newCoG * distance_foot_newCoG - distance_CoG_foot * distance_CoG_foot) / (2 * distance_CoG_foot * distance_foot_newCoG));
    if (reverse_angle)
    {
        temp_alpha_diag = atan(move_x / move_y) - atan(start_global_position[shoulder_left][0] / start_global_position[shoulder_left][1]);
        alpha_min = min(temp_alpha_diag, temp_alpha);
    }
    else
    {
        temp_alpha_diag = atan(move_y / move_x) - atan(start_global_position[shoulder_right][1] / start_global_position[shoulder_right][0]);
        alpha_max = min(temp_alpha_diag, temp_alpha);
    }

    float a = 1 + start_global_position[2 - first_leg_to_move][1] * start_global_position[2 - first_leg_to_move][1] / (start_global_position[2 - first_leg_to_move][0] * start_global_position[2 - first_leg_to_move][0]);
    float b = -2 * move_lateral - 2 * start_global_position[2 - first_leg_to_move][1] * move_longitudinal / start_global_position[2 - first_leg_to_move][0];
    float c = distance_CoG_newCoG * distance_CoG_newCoG - distance_CoG_foot * distance_CoG_foot;

    float X_plus = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
    float X_minus = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
    float Y_plus = start_global_position[2 - first_leg_to_move][1] / start_global_position[2 - first_leg_to_move][0] * X_plus;
    float Y_minus = start_global_position[2 - first_leg_to_move][1] / start_global_position[2 - first_leg_to_move][0] * X_minus;
    float X;
    float distance_plus = sqrt((start_global_position[first_leg_to_move][0] + move_x - X_plus) * (start_global_position[first_leg_to_move][0] + move_x - X_plus) + (start_global_position[first_leg_to_move][1] + move_y - Y_plus) * (start_global_position[first_leg_to_move][1] + move_y - Y_plus));
    float distance_minus = sqrt((start_global_position[first_leg_to_move][0] + move_x - X_minus) * (start_global_position[first_leg_to_move][0] + move_x - X_minus) + (start_global_position[first_leg_to_move][1] + move_y - Y_minus) * (start_global_position[first_leg_to_move][1] + move_y - Y_minus));
    if (distance_plus <= distance_minus)
    {
        X = X_plus;
    }
    else
    {
        X = X_minus;
    }
    float Y = start_global_position[2 - first_leg_to_move][1] / start_global_position[2 - first_leg_to_move][0] * X;
    // TO BE CONTINUED WITH alpha_min...

    float distance_newCoG_newFoot_NoAlpha = distance_CoG_foot;
    float distance_newCoG_alphaMin = sqrt((move_lateral - X) * (move_lateral - X) + (move_longitudinal - Y) * (move_longitudinal - Y));
    float distance_newFoot_NoAlpha_alphaMin = sqrt((start_global_position[first_leg_to_move][0] + move_lateral - X) * (start_global_position[first_leg_to_move][0] + move_lateral - X) + (start_global_position[first_leg_to_move][1] + move_longitudinal - Y) * (start_global_position[first_leg_to_move][1] + move_longitudinal - Y));

    temp_alpha = acos((distance_newFoot_NoAlpha_alphaMin * distance_newFoot_NoAlpha_alphaMin - distance_newCoG_alphaMin * distance_newCoG_alphaMin - distance_newCoG_newFoot_NoAlpha * distance_newCoG_newFoot_NoAlpha) / (2 * distance_newCoG_alphaMin * distance_newCoG_newFoot_NoAlpha));
    if (reverse_angle)
    {
        temp_alpha_diag = atan(move_y / move_x) - atan(start_global_position[shoulder_right][1] / start_global_position[shoulder_right][0]);
        alpha_max = min(temp_alpha_diag, temp_alpha);
    }
    else
    {
        temp_alpha_diag = atan(move_x / move_y) - atan(start_global_position[shoulder_left][0] / start_global_position[shoulder_left][1]);
        alpha_min = min(temp_alpha_diag, temp_alpha);
    }

    return true;
}

void Droideka_Movement::establish_leg_order(float throttle_longitudinal, float throttle_lateral)
{
    if (abs(throttle_longitudinal) > abs(throttle_lateral) && throttle_lateral >= 0 && throttle_longitudinal >= 0) // Front right
    {
        leg_order[3] = 1;
        leg_order[1] = 2;
        leg_order[2] = 3;
        leg_order[0] = 4;
        first_leg_to_move = 3;
        reverse_angle = false;
        shoulder_left = 0;
        shoulder_right = 1;
    }
    else if (abs(throttle_longitudinal) > abs(throttle_lateral) && throttle_lateral < 0 && throttle_longitudinal >= 0) // Front left
    {
        leg_order[2] = 1;
        leg_order[0] = 2;
        leg_order[3] = 3;
        leg_order[1] = 4;
        first_leg_to_move = 2;
        reverse_angle = true;
        shoulder_left = 0;
        shoulder_right = 1;
    }
    else if (abs(throttle_longitudinal) > abs(throttle_lateral) && throttle_lateral >= 0 && throttle_longitudinal < 0) // Rear right
    {
        leg_order[1] = 1;
        leg_order[3] = 2;
        leg_order[0] = 3;
        leg_order[2] = 4;
        first_leg_to_move = 1;
        reverse_angle = true;
        shoulder_left = 3;
        shoulder_right = 2;
    }
    else if (abs(throttle_longitudinal) > abs(throttle_lateral) && throttle_lateral < 0 && throttle_longitudinal < 0) // Rear left
    {
        leg_order[0] = 1;
        leg_order[2] = 2;
        leg_order[1] = 3;
        leg_order[3] = 4;
        first_leg_to_move = 0;
        reverse_angle = false;
        shoulder_left = 3;
        shoulder_right = 2;
    }
    else if (abs(throttle_longitudinal) < abs(throttle_lateral) && throttle_lateral > 0 && throttle_longitudinal > 0) // Right Up
    {
        leg_order[0] = 1;
        leg_order[1] = 2;
        leg_order[2] = 3;
        leg_order[3] = 4;
        first_leg_to_move = 0;
        reverse_angle = false;
        shoulder_left = 1;
        shoulder_right = 3;
    }
    else if (abs(throttle_longitudinal) < abs(throttle_lateral) && throttle_lateral < 0 && throttle_longitudinal > 0) // Left Up
    {
        leg_order[1] = 1;
        leg_order[0] = 2;
        leg_order[3] = 3;
        leg_order[2] = 4;
        first_leg_to_move = 1;
        reverse_angle = true;
        shoulder_left = 2;
        shoulder_right = 0;
    }
    else if (abs(throttle_longitudinal) < abs(throttle_lateral) && throttle_lateral > 0 && throttle_longitudinal < 0) // Right Down
    {
        leg_order[2] = 1;
        leg_order[3] = 2;
        leg_order[0] = 3;
        leg_order[1] = 4;
        first_leg_to_move = 2;
        reverse_angle = true;
        shoulder_left = 1;
        shoulder_right = 3;
    }
    else if (abs(throttle_longitudinal) < abs(throttle_lateral) && throttle_lateral < 0 && throttle_longitudinal < 0) // Left Down
    {
        leg_order[3] = 1;
        leg_order[2] = 2;
        leg_order[1] = 3;
        leg_order[0] = 4;
        first_leg_to_move = 3;
        reverse_angle = false;
        shoulder_left = 2;
        shoulder_right = 0;
    }
}

Droideka_Position Droideka_Movement::get_future_position(Droideka_Position start_pos, float trans_x[TIME_SAMPLE], float trans_y[TIME_SAMPLE], float angle[TIME_SAMPLE], int time_elapsed, int one_leg = -1)
{
    if (time_elapsed < 0 || time_elapsed > TIME_SAMPLE)
    {
        return start_pos; // Not sure this is right if time_elapsed > TIME_SAMPLE.
    }

    float temp[LEG_NB][2];           // x and y final coordinates of each feet
    float temp_trans[LEG_NB][2];     // translation vector of the shoulder.
    float temp_final_pos[LEG_NB][3]; // used to build the Droideka_Position object by calculating rho, theta and z thanks to x and y stored in temp.
    float feet_start[LEG_NB][2];     // Feet coordinates at the start

    for (int ii = 0; ii < LEG_NB; ii++)
    {
        if (one_leg != -1)
        {
            ii = one_leg;
        }

        temp_trans[ii][0] = shoulder_pos[ii][0] * (cos(PI * angle[time_elapsed] / 180) - 1) - shoulder_pos[ii][1] * sin(PI * angle[time_elapsed] / 180) + tx[time_elapsed];
        temp_trans[ii][1] = shoulder_pos[ii][0] * sin(PI * angle[time_elapsed] / 180) - shoulder_pos[ii][1] * (cos(PI * angle[time_elapsed] / 180) - 1) + ty[time_elapsed];

        feet_start[ii][0] = shoulder_mult[ii][0] * (start_pos.legs[ii][1] * cos(PI * start_pos.legs[ii][0] / 180));
        feet_start[ii][1] = shoulder_mult[ii][1] * (start_pos.legs[ii][1] * sin(PI * start_pos.legs[ii][0] / 180));

        temp[ii][0] = (feet_start[ii][0] - temp_trans[ii][0]) * cos(PI * angle[time_elapsed] / 180) + (feet_start[ii][1] - temp_trans[ii][1]) * sin(PI * angle[time_elapsed] / 180);
        temp[ii][1] = (feet_start[ii][1] - temp_trans[ii][1]) * cos(PI * angle[time_elapsed] / 180) + (feet_start[ii][0] - temp_trans[ii][0]) * sin(PI * angle[time_elapsed] / 180);

        //temp[ii][0] = shoulder_pos[ii][0] * (cos(PI * angle[time_elapsed] / 180) - 1) + shoulder_pos[ii][1] * sin(PI * angle[time_elapsed] / 180) + shoulder_mult[ii][0] * sqrt((start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][0] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) * (start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][0] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) + (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][0] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed]) * (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][0] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed])) * cos(shoulder_mult[ii][0] * shoulder_mult[ii][1] * atan((start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][0] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) / (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][0] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed])) - PI * angle[time_elapsed] / 180);
        //temp[ii][1] = shoulder_pos[ii][1] * (cos(PI * angle[time_elapsed] / 180) - 1) + shoulder_pos[ii][0] * sin(PI * angle[time_elapsed] / 180) + shoulder_mult[ii][0] * sqrt((start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][0] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) * (start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][0] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) + (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][0] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed]) * (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][0] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed])) * sin(shoulder_mult[ii][0] * shoulder_mult[ii][1] * atan((start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][0] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) / (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][0] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed])) - PI * angle[time_elapsed] / 180);

        temp_final_pos[ii][2] = start_pos.legs[ii][2];
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

        if (one_leg != -1)
        {
            ii = LEG_NB;
        }
    }
    Droideka_Position final_pos(temp_final_pos);
    return final_pos;
}

Droideka_Position Droideka_Movement::get_final_position(Droideka_Position start_pos)
{
    return get_future_position(start_pos, tx, ty, alpha, TIME_SAMPLE - 1);
}

Droideka_Position Droideka_Movement::get_lifted_position(int leg, Droideka_Position start_pos, Droideka_Position end_pos, unsigned long time_)
{
    unsigned long debut_time = (leg_order[leg] - 1) * TIME_SAMPLE / moving_leg_nb + delta_time;
    unsigned long fin_time = leg_order[leg] * TIME_SAMPLE / moving_leg_nb;
    unsigned long interval_time = fin_time - debut_time;
    unsigned long time_from_lifting = time_ - debut_time;

    Droideka_Position debut_pos(get_future_position(start_pos, tx, ty, alpha, debut_time, leg).legs);
    Droideka_Position fin_pos(get_future_position(end_pos, reverse_tx, reverse_ty, reverse_alpha, TIME_SAMPLE - 1 - fin_time, leg).legs);

    // Between the lifting and putting back of the leg, theta and X are linear, wheras Y follows a quadratic curve (arbitrarily defined)

    float temp[LEG_NB][3];
    for (int ii = 0; ii < 2; ii++)
    {
        temp[leg][ii] = (fin_pos.legs[leg][ii] - debut_pos.legs[leg][ii]) / interval_time * time_from_lifting + debut_pos.legs[leg][ii];
    }
    temp[leg][2] = Y_NOT_TOUCHING - (Y_NOT_TOUCHING - Y_TOUCHING) * ((time_from_lifting - interval_time / 2) / (interval_time / 2)) * ((time_from_lifting - interval_time / 2) / (interval_time / 2));

    Droideka_Position result(temp);
    return result;
}

ErrorCode Droideka_Movement::establish_legs_movement()
{
    float temp[LEG_NB][3];
    unsigned long time_leg_starts_lifting;
    unsigned long time_leg_touches_ground_again;

    for (int ii = 0; ii < TIME_SAMPLE; ii++)
    {
        Droideka_Position temp_current_pos = get_future_position(start_position, tx, ty, alpha, ii);                                        // Calculates the position of the legs before the leg is lifted.
        Droideka_Position temp_future_pos = get_future_position(end_position, reverse_tx, reverse_ty, reverse_alpha, TIME_SAMPLE - 1 - ii); // Calculates the position of the legs after the leg has been lifted and put back on the ground.

        for (int jj = 0; jj < LEG_NB; jj++)
        {
            time_leg_starts_lifting = (leg_order[jj] - 1) * TIME_SAMPLE / moving_leg_nb + delta_time;
            time_leg_touches_ground_again = (leg_order[jj]) * TIME_SAMPLE / moving_leg_nb;

            if (ii <= time_leg_starts_lifting)
            {
                for (int kk = 0; kk < 3; kk++)
                {
                    temp[jj][kk] = temp_current_pos.legs[jj][kk];
                }
            }
            else if (ii > time_leg_starts_lifting && ii <= time_leg_touches_ground_again)
            {
                for (int kk = 0; kk < 3; kk++)
                {
                    temp[jj][kk] = get_lifted_position(jj, start_position, end_position, ii).legs[jj][kk];
                }
            }
            else if (ii > time_leg_touches_ground_again && ii <= TIME_SAMPLE)
            {
                for (int kk = 0; kk < 3; kk++)
                {
                    temp[jj][kk] = temp_future_pos.legs[jj][kk];
                }
            }
        }
        positions[ii] = Droideka_Position(temp);
    }
    return NO_ERROR;
}