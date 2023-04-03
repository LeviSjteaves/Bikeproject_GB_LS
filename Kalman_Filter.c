#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#define RADIUS_OF_THE_EARTH 6371000.0

/**
 * State estimator using a Kalman filter
 * 
 * Estimated states
 * @param X             State variable -  X position [m]
 * @param Y             State variable - Y position [m]
 * @param Psi           State variable - track/yaw angle [rad]
 * @param roll          State variable - roll angle [rad]
 * @param rollRate      State variable - time derivative of roll angle [rad/s]
 * @param delta         State variable - Steering [rad]
 * @param v             State variable - Speed [m/s]
 * 
 * Input:
 * @param dot_delta     Input of the model
 * 
 * Measurements:
 * @param latitude      GPS latitude [rad]
 * @param longitude     GPS longitude [rad]
 * @param X_GPS         GPS X position [m]
 * @param Y_GPS         GPS Y position[m]
 * @param a_y           Accelerometer Y value [m/s²]
 * @param w_x           Accelerometer roll rate (around X axis) [rad/s]
 * @param w_z           Accelerometer around Z axis value [rad/s]
 * @param delta_enc     Encoder steering value [rad]
 * @param speed         Approximative current speed of bike, e.g. reference speed [m/s]
 * 
 * Parameters: 
 * @param Kalman_Gain1  Kalman gain including GPS measurements
 * @param Kalman_Gain2  Kalman gain excluding GPS measurements
 * @param A_d           Discrete linear bike model (A matrix)
 * @param B_d           Discrete linear bike model (B matrix)
 * @param C1            Measurement model (C matrix) when GPS meas. are included
 * @param C2            Measurement model (C matrix) when GPS meas. are excluded
 * @param D1            Measurement model (D matrix) when GPS meas. are included
 * @param D2            Measurement model (D matrix) when GPS meas. are excluded
 * @param Ts            Time step [s]
 * 
 */

// Main function
extern void Kalman_filter(double *X, double *Y, double *Psi, double *roll, double *rollRate, double *delta, double *v, 
                          double *dot_delta, double latitude, double longitude, double X_GPS, double Y_GPS, double a_y, double w_x,
                          double w_z, double delta_enc, double speed, double Kalman_gain1, double Kalman_Gain2, double A_d,
                          double B_d, double C1, double C2, double D1, double D2, double Ts,)
{
    double Est_States[7];
    double Est_States_l[7];
    double Est_states_l[7];
    double Est_states[7];
    double y[7];

    // Initialize the states vector at time t-1
    Est_States[0] = *X;
    Est_States[1] = *Y;
    Est_States[2] = *Psi;
    Est_States[3] = *roll;
    Est_States[4] = *rollRate;
    Est_States[5] = *delta;
    Est_States[6] = *v;

    // Transform the GPS lat/long into X/Y measurements
    [X_GPS,Y_GPS] = transform_latlog_to_XY(longitude, latitude)

    // Wrap the measurements into an array:
    y[0] = X_GPS;
    y[1] = Y_GPS;
    y[2] = a_y;
    y[3] = w_x;
    y[4] = w_z;
    y[5] = delta_enc;
    y[6] = speed;

    // 1. Transformation of states at time t-1 and measurements at time t into local frame
    Est_States_l = transform_global_to_local(Est_States);

    // 2. Time update
    Est_States_l = time_update(Est_States_l, dot_delta, A_d, B_d);

    // 3. Measurement update 
    Est_states_l = measurement_update(Est_States_l, *dot_delta, y, Kalman_Gain1, Kalman_Gain2, C1, C2, D1, D2);

    // 4. Transform estimated states at time t to global frame
    Est_states = transform_local_to_global(Est_states_l,Est_States);

    // Output the estimated states
    *X = Est_states[0];
    *Y = Est_states[1];
    *Psi = Est_states[2];
    *roll = Est_states[3];
    *rollRate = Est_states[4];
    *delta = Est_states[5];
    *v = Est_states[6];

}


/**
 * Sets *value to *lastValue if *value is NaN.
 * Otherwise *lastValue is set to *value.
 * 
 * @param value Current value
 * @param lastValue The last known good value
 * 
*/
static void useLastValueIfNaN(double *value, double *lastValue)
{
    if (isnan(*value))
    {
        *value = *lastValue;
    }
    else
    {
        *lastValue = *value;
    }
}

// Transform the GPS measurement (longitude/latitude) into X/Y measurement
double transform_latlog_to_XY(double longitude, double latitude)
{
    static bool initializedLatLon = false;
    static double latitude0, longitude0;
    static double lastLatitude = NAN;
    static double lastLongitude = NAN;

    useLastValueIfNaN(&latitude, &lastLatitude);
    useLastValueIfNaN(&longitude, &lastLongitude);

    if (!initializedLatLon && !isnan(latitude) && !isnan(longitude))
    {
        latitude0 = latitude;
        longitude0 = longitude;

        initializedLatLon = true;
    }
    if (initializedLatLon)  // Check if the transformation is correctly done
    {
        X_GPS = RADIUS_OF_THE_EARTH * (longitude - longitude0) * cos(latitude0);
        Y_GPS = RADIUS_OF_THE_EARTH * (latitude - latitude0);
    }
    return X_GPS, Y_GPS;
}

// Transform from global to local frame
double transform_global_to_local(double Est_States)
{
    double Est_States_l[7];

    Est_States_l[0] = Est_States[0] * cos(Est_States[2]) + Est_States[1] * sin(Est_States[2]);
    Est_States_l[1] = -Est_States[0] * sin(Est_States[2]) + Est_States[1] * cos(Est_States[2]);
    Est_States_l[2] = 0;
    Est_States_l[3] = Est_States[3];
    Est_States_l[4] = Est_States[4];
    Est_States_l[5] = Est_States[5];
    Est_States_l[6] = Est_States[6];

    return Est_States_l
}

// Transform from global to local frame
double transform_local_to_global(double Est_states_l,double Est_States)
{
    double Est_states[7];

    Est_states[0] = Est_states_l[0] * cos(Est_States[2]) + Est_states_l[1] * sin(Est_States[2]);
    Est_states[1] = -Est_states_l[0] * sin(Est_States[2]) + Est_states_l[1] * cos(Est_States[2]);
    Est_states[2] = Est_states_l[2] + Est_States[2];
    Est_states[3] = Est_states_l[3];
    Est_states[4] = Est_states_l[4];
    Est_states[5] = Est_states_l[5];
    Est_states[6] = Est_states_l[6];

    return Est_states
}

// Time update from t-1 to t
double time_update(double Est_States_l, double *dot_delta, double A_d, double B_d)
{
    double result1[7];
    double result2[7];
    double result3[7];

    // A_d*x
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            result1[i] += A_d[i][j] * Est_States_l[j];
        }
    }

    // B_d*u
    for(int k = 0, k < 7, k++){
        result2[k] = B_d[k] * (*dot_delta);
    }

    // A_d*x + B_d*u
    result3; = result1 + result2;

    return result3
}

// Measurement update using the sensor data at time t
double measurement update(double Est_States_l, double *dot_delta, double y, double Kalman_Gain1, double Kalman_Gain2, double C1, 
                          double C2, double D1, double D2)
{
    double Est_states_l[7];
    double y_pred[7];

    //TODO: THINK ABOUT HOW TO DIFFERENCIATE SAMPLING RATES HERE
    // Est_states_l = Est_States_l;

    // Measurement prediction
    y_pred = C1 * Est_States_l + D1 * (*dot_delta);

    // Measurement update
    Est_states_l = Est_States_l + K1 * (y - y_pred);

    return Est_states_l;
}
