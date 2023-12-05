// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import frc.robot.Constants;
import frc.robot.Constants.RobotConstants.DrivetrainConstants;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Drive {
//TODO:change to english(it is in spanish)

    //chassis

    double[][] origin_pt = new double[4][2] ; // x,y coord of module point in vehicle coord sys
    // module 0 is in +x,+y quadrant
    // module 1 is in -x,+y quadrant
    // module 2 is in -x,-y quadrant
    // module 3 is in +x,-y quadrant

    public double angle;//ungulo entre donde el robot mira y donde el robot esta derecho en el fild en radianes(gyro)(rad)
    public double[] centerVel = new double[2];//Velocity in m/s from the center (in the direction where the robot is going)
    public double angularVel;//veloity in degrees/s of how fast the robot spins 

    //wheel
    public double[] steeringAngle = new double[4];//es lo mismo que angle pero solo en las ruedas(in degrees)
    public double[] wheelAngularVel = new double[4]; //velocidad de la rueda cuando gira no para avanzar si no para rotar en degrees/sec
    
    public double[] lastSteeringAngle = new double[4];//Last Steering Angle 
    public double[] lastWheelAngularVel = new double[4]; //Last wheel angular velocity in degrees/sec

    public double[] desiredSteeringAngle = new double[4];//es a donde tu quieres que valla la rueda in degrees
    public double[] DesiredWeelAngularVel = new double[4];//es a cuantos degrees/sec tu quieres que valla la rueda 
    
    public double[] RecommendedSteeringAnglesDeg = new double[4] ;//angle in degree
    public double[] RecommendedWheelAngularVel = new double[4] ;//degrees/sec

    public void assign_veh_values_for_square_vehicle(double d, double wheel_d)
    {
        // set the wheel diameter
        DrivetrainConstants.WHEEL_RADIUS_METERS = wheel_d ;
        
        // define the locations of the four swerve modules in the xv, yv coord sys
        
        this.origin_pt[0][0] =  d ;   // module 0 is in the +x,+y quadrant
        this.origin_pt[0][1] =  d ;
       
        this.origin_pt[1][0] = -d ;   // module 1 is in the -x,+y quadrant
        this.origin_pt[1][1] =  d ;
        
        this.origin_pt[2][0] = -d ;   // module 2 is in the -x,-y quadrant
        this.origin_pt[2][1] = -d ;
  
        this.origin_pt[3][0] =  d ;   // module 3 is in the +x,-y quadrant
        this.origin_pt[3][1] = -d ;
    }
void initialize_steering_angles_and_wheel_speeds(double[] steering_ang_deg,
    double[] wheel_speeds_deg_per_sec)
{
int i ;
for (i=0 ; i<4 ; ++i)
{
this.steeringAngle[i] = steering_ang_deg[i] ;
this.wheelAngularVel[i] = wheel_speeds_deg_per_sec[i] ;
}
}
void calc_swerve_steering_and_speed()
{
int i ;
double sv, cv ;

cv = Math.cos(this.angle) ;
sv = Math.sin(this.angle) ;

// calculate the vehicle velocity in the vehicle coord sys
double[] vO_in_veh = new double [2] ;
vO_in_veh[0] =  cv * this.centerVel[0] + sv * this.centerVel[1] ;
vO_in_veh[1] = -sv * this.centerVel[0] + cv * this.centerVel[1] ;

double[] v_i = new double[2] ;
double speed ;

for (i=0 ; i<4 ; ++i)
{
    v_i[0] = vO_in_veh[0] - this.angularVel*Units.degreesToRadians( this.origin_pt[i][1]) ;
    v_i[1] = vO_in_veh[1] + this.angularVel*Units.degreesToRadians( this.origin_pt[i][0]) ;

    speed = Math.sqrt(v_i[0]*v_i[0] + v_i[1]*v_i[1]) ;
    this.lastSteeringAngle[i] = Units.degreesToRadians(Math.atan2(v_i[1], v_i[0]))  ;
    this.lastWheelAngularVel[i] = Units.degreesToRadians( speed) / ( DrivetrainConstants.WHEEL_RADIUS_METERS) ;       
}    
}
}
