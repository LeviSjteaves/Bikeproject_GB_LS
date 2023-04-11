+#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdint.h>



/**
 * Trajectory controller
 * 
 * Input:
 * @param traj          Trajectory reference [X_ref Y_ref Psi_ref]
 * @param X_est         X position estimated state from kalman filter
 * @param Y_est         Y position estimated state from kalman filter
 * @param Psi_est       Heading estimated state from kalman filter
 * 
 * Output:
 * @param Roll_ref      Roll reference fot the balance control
 * 
 * Parameters: 
 * @param bike_params             Gravity parameter     [g lr lf lambda]
 * @param traj_params             Trajectory parameters [k1 k2 e1_max]
 * @param v                       Velocity
 * 
 * 
 */


// Generate the matrices due to Labview
extern void transform_mat(double *input, double (*matrix)[7]) {
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            matrix[i][j] = input[7*i + j];
        }
    }
}

// Transform the GPS measurement (longitude/latitude) into X/Y measurement
extern void transform_latlog_to_XY_local(double longitude, double latitude, double* X_GPS, double* Y_GPS, double Est_States[7])
{
    static bool initializedLatLon = false;
    static double latitude0, longitude0;
    static double lastLatitude = NAN;
    static double lastLongitude = NAN;

    // X and Y in global frame
    double X_GPS_g = 0;
    double Y_GPS_g = 0;

    useLastValueIfNaN(&latitude, &lastLatitude);
    useLastValueIfNaN(&longitude, &lastLongitude);

    if (!initializedLatLon && !isnan(latitude) && !isnan(longitude))
    {
        latitude0 = latitude;
        longitude0 = longitude;

        initializedLatLon = true;
    }
    if (initializedLatLon)  
    {
        X_GPS_g = RADIUS_OF_THE_EARTH * (longitude - longitude0) * cos(latitude0);
        Y_GPS_g = RADIUS_OF_THE_EARTH * (latitude - latitude0);
        
        // Transform to local
        *X_GPS = X_GPS_g * cos(Est_States[2]) + Y_GPS_g * sin(Est_States[2]);
        *Y_GPS = -X_GPS_g * sin(Est_States[2]) + Y_GPS_g * cos(Est_States[2]);
    }
}

// Transform from global to local frame
extern void transform_global_to_local(double Est_States[7], double Est_States_l[7])
{
    Est_States_l[0] = Est_States[0] * cos(Est_States[2]) + Est_States[1] * sin(Est_States[2]);
    Est_States_l[1] = -Est_States[0] * sin(Est_States[2]) + Est_States[1] * cos(Est_States[2]);
    Est_States_l[2] = 0;
    Est_States_l[3] = Est_States[3];
    Est_States_l[4] = Est_States[4];
    Est_States_l[5] = Est_States[5];
    Est_States_l[6] = Est_States[6];
}

// Time update from t-1 to t
extern void time_update(double Est_States_l[7], double dot_delta, double (*A_d)[7], double B_d[7],double Est_States_l_1[7])
{
    double result1[7] = {0,0,0,0,0,0,0};
    double result2[7] = {0,0,0,0,0,0,0};

    // A_d*x
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            result1[i] += A_d[i][j] * Est_States_l[j];
        }
    }

    // B_d*u
    for(int k = 0; k < 7; k++)
    {
        result2[k] = B_d[k] * (dot_delta)  ;
    }

    // Time update
    for(int h = 0; h < 7; h++) 
    {
        Est_States_l_1[h] = result1[h] + result2[h];
    }
   
}


// Main function
extern void trajectory_controller(double *traj_flat, double X_est, double Y_est, double Psi_est, double *bike_params,
                                  double *traj_params, double v, double *Roll_ref)
{

    static double closest_point;
    static double ids;
  
    // Unpack parameters
    g  = bike_params[0];
    lr = bike_params[1];
    lf = bike_params[2];
    lambda = bike_params[3];

    k1 = traj_params[0];
    k2 = traj_params[1];
    e1_max = traj_params[2];

    // Second point in reference is selected as current selected closest point
    closest_point = 1;

    

 }