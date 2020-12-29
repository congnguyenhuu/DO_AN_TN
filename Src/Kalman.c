/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include "Kalman.h"



// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    Kalman->rate = newRate - Kalman->bias;
    Kalman->angle += dt * Kalman->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    Kalman->P[0][0] += dt * (dt*Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = Kalman->P[0][0] + Kalman->R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - Kalman->angle; // Angle difference
    /* Step 6 */
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};

void setAngle(Kalman_t *Kalman, float sangle) { Kalman->angle = sangle; }; // Used to set angle, this should be set as the starting angle
float getRate(Kalman_t *Kalman) { return Kalman->rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void setQangle(Kalman_t *Kalman, float sQ_angle) { Kalman->Q_angle = sQ_angle; };
void setQbias(Kalman_t *Kalman, float sQ_bias) { Kalman->Q_bias = sQ_bias; };
void setRmeasure(Kalman_t *Kalman, float sR_measure) { Kalman->R_measure = sR_measure; };

float getQangle(Kalman_t *Kalman) { return Kalman->Q_angle; };
float getQbias(Kalman_t *Kalman) { return Kalman->Q_bias; };
float getRmeasure(Kalman_t *Kalman) { return Kalman->R_measure; };
