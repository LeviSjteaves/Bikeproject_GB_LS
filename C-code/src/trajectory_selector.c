#include <stdint.h>



/**
 * Trajectory controller
 * 
 * Input:
 * @param traj              Trajectory reference [X_ref Y_ref Psi_ref]
 * @param closest_point     Closest point index in the trajectory
 * 
 * Output:
 * @param X_loc             Local X coordinates from reference trajectory
 * @param Y_loc             Local Y coordinates from refence trajectory
 * @param Psi_loc           Local Heading from reference trajectory 
 *  
 * Parameters:
 * @param hor_dis           Horizon distance
 * @param ref_dis           Distance between the points
 */


// Main function
extern void trajectory_selector(double *traj, double closest_point, double *X_loc, double *Y_loc, double *Psi_loc, double Ns, double reset)
{
    static double ids;

    // Initialize the variables in first iteration
    if(reset == 0)
    {
        ids = 0;
        closest_point = 1;
    }

    // Calculate the initial point of the local_traj_ref
    ids = ids - 1 + closest_point;

    int M = ids + Ns;                   // index of the last point of the local trajectort
    int n = sizeof(X)

    if (M > n)                      // if reached near end of traj, the local_traj becomes shorter
    {
        M = n;
    }

    // TODO: what happens when we reach the end of the trajectory

    // Transform the traj matrix into usable information
    int size = sizeof(traj) / 3;
    double X_traj[size];
    double Y_traj[size];
    double Psi_traj[size];
    double counter = 0;

    for(int j = 0; j < size; j++)
    {
        X_traj[counter] = traj[j];
        Y_traj[counter] = traj[j+size];
        Psi_traj[counter] = traj[j+2*size];
        counter += 1;
    }

    // Select the local_traj from traj
    counter = 0;
    for (int i = ids-1; i < M; i++)
    {
        *X_loc[counter] = X_traj[i];
        *Y_loc[counter] = Y_traj[i];
        *Psi_loc[counter] = Psi_traj[i];
        counter += 1 ;
    }

}