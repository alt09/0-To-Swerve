// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
  
    public static double[][] originPoint = new double[4][2]; // x,y coords of the four modules, from the center point of the robot
    
    // Vehicle states values
    public static double angleRadians; // Distance between desired angle and inital angle
    public static double[] velOriginMetersPerSec = new double[2]; // Desired linear velocity of the vehicle origin point
    public static double angleVelDegPerSec; // Desired angular velocity of the chassis

    public static double[] currentSteeringAngleDeg = new double[4];
    public static double[] currentWheelAngularVelDegPerSec = new double [4];

    public static double[] calculatedSteeringAnglesDeg = new double[4];
    public static double[] calculatedWheelAngularVelDegPerSec = new double[4];

    public static double[] recommendedSteeringAnglesDeg = new double[4];
    public static double[] recommendedWheelAngularVelDegPerSec = new double[4];
  
    /** Creates a new Drive. */
  public Drive() {
  }

  public void assignVehicalValues(double d, double wheel_d) {
    DriveConstants.WHEEL_DIAMETER_METERS = wheel_d;

    // Defome tje module locations
    this.originPoint[0][0] =  d;  // module 0 (+x, +y)
    this.originPoint[0][1] =  d;

    this.originPoint[1][0] = -d;  // module 1 (-x, +y)
    this.originPoint[1][1] =  d;

    this.originPoint[2][0] = -d;  // module 2 (-x, -y)
    this.originPoint[2][1] = -d;

    this.originPoint[3][0] =  d;  // module 3 (+x, -y)
    this.originPoint[3][1] = -d;
  }

public void  initalizeSteeringAnglesAndWheelSpeeds(double[] steeringAngleDegree, double[] wheelSpeedsDegPerSec) {
  int i;
  for (i = 0 ; i < 4 ; ++i) 
    {
    this.currentSteeringAngleDeg[i] = steeringAngleDegree[i];
    this.currentWheelAngularVelDegPerSec[i] = wheelSpeedsDegPerSec[i];
    }
  }

public void calcSwerveSteeringAndSpeed() {
  int i;
  double sv, cv;

  cv = Math.cos(this.angleRadians); // Horizontal (x axis) velocity
  sv = Math.sin(this.angleRadians); // Vertical (y axis) velocity

  // Calculate the vehicle velocity in the vehicle coordinate system
  double[] vO_IN_VEHICLE = new double[2];
  vO_IN_VEHICLE[0] =  cv * this.velOriginMetersPerSec[0] + sv * this.velOriginMetersPerSec[1]; // Inital velocity in the x axis | xcos0 - ysin0
  vO_IN_VEHICLE[1] = -sv * this.velOriginMetersPerSec[0] + cv * this.velOriginMetersPerSec[1]; // Inital velocity in the y axis | xsin0 + ycos0

  double[] v_i = new double[2];
  double speed;
  for (i = 0 ; i < 4 ; ++i) 
    {
    v_i[0] = vO_IN_VEHICLE[0] - Units.degreesToRadians(this.angleVelDegPerSec) * this.originPoint[i][1]; // Translates the new x velocity back to the original x velocity
    v_i[1] = vO_IN_VEHICLE[1] + Units.degreesToRadians(this.angleVelDegPerSec) * this.originPoint[i][0]; // Translates the new y velocity back to the original y velocity

    speed = Math.sqrt(v_i[0] * v_i[0] + v_i[1] * v_i[1]); // Calculates the resultant velocity | Vr = sqrt(Vx^2 + Vy^2)
    this.recommendedSteeringAnglesDeg[i] = Units.radiansToDegrees(Math.atan2(v_i[1], v_i[0])); // Calculates the angle that the robot wants to turn to | 0 = atan2(B, C) * 180/pi
    this.recommendedWheelAngularVelDegPerSec[i] = Units.radiansToDegrees(speed) / DriveConstants.WHEEL_RADIUS_METERS; // Angular velocity = Linear velocity / Radius | w = v/r
    }
  }

  public void recommendedSwerveSteeringAndSpeed() {
    int i;
    double ANG_DIFF_DEG;

    for (i = 0 ; i < 4 ; i++) 
    {
      ANG_DIFF_DEG = this.calculatedSteeringAnglesDeg[i] - this.currentSteeringAngleDeg[i];

      while (ANG_DIFF_DEG < 180.0)
        ANG_DIFF_DEG += 360.0;
      while (ANG_DIFF_DEG > 360.0)
        ANG_DIFF_DEG -= 360.0;

    if (Math.abs(ANG_DIFF_DEG) < 90.0) 
      {
      this.recommendedSteeringAnglesDeg[i] = this.currentSteeringAngleDeg[i] + ANG_DIFF_DEG;
      this.recommendedWheelAngularVelDegPerSec[i] = this.calculatedWheelAngularVelDegPerSec[i];
      } else 
      {
      this.recommendedSteeringAnglesDeg[i] = this.currentSteeringAngleDeg[i] + ANG_DIFF_DEG - 180.0; // Or +180.0
      this.recommendedWheelAngularVelDegPerSec[i] = -this.calculatedWheelAngularVelDegPerSec[i];
      while (this.recommendedSteeringAnglesDeg[i] - this.currentSteeringAngleDeg[i] > 180.0)
        this.currentSteeringAngleDeg[i] -= 360.0;
      while (this.recommendedSteeringAnglesDeg[i] - this.currentSteeringAngleDeg[i] < -180.0)
        this.currentSteeringAngleDeg[i] += 360.0;
      }
    }
  }

  public void printResults()
  {
      int i ;
      System.out.println("------------------") ;
      System.out.println("\nVehicle Velocity Inputs (from joystick input):") ;
      System.out.printf("\tvelocity of point 0 = %.4f, %.4f m/sec\n",
                         this.velOriginMetersPerSec[0], this.velOriginMetersPerSec[1]) ;
      System.out.printf("\tangular velocity = %.2f deg/sec\n", this.angleVelDegPerSec) ;
      System.out.printf("vehicle ang = %.2f degrees (from IMU sensor)\n\n", Units.radiansToDegrees(this.angleRadians));
      
      System.out.println("\t\t\t\tmodule 0\tmodule 1\tmodule 2\tmodule3") ;
      System.out.printf("current steering ang") ;
      for (i=0 ; i<4 ; ++i)
      {
          System.out.printf("\t\t%.2f", this.currentSteeringAngleDeg[i]) ;  
      }
      System.out.printf("\tdeg\n") ;
      
      System.out.printf("calculated steering ang") ;
      for (i=0 ; i<4 ; ++i)
      {
          System.out.printf("\t\t%.2f", this.calculatedSteeringAnglesDeg[i]) ;  
      }
      System.out.printf("\tdeg\n") ;
      
      System.out.printf("recommend steering ang") ;
      for (i=0 ; i<4 ; ++i)
      {
          System.out.printf("\t\t%.2f", this.recommendedSteeringAnglesDeg[i]) ;  
      }
      System.out.printf("\tdeg\n\n") ;
      
      System.out.printf("current wheel speed") ;
      for (i=0 ; i<4 ; ++i)
      {
          System.out.printf("\t\t%.1f", this.currentWheelAngularVelDegPerSec[i]) ;  
      }
      System.out.printf("\tdeg/sec\n") ;
      
      System.out.printf("calculated wheel speed") ;
      for (i=0 ; i<4 ; ++i)
      {
          System.out.printf("\t\t%.1f", this.calculatedWheelAngularVelDegPerSec[i]) ;  
      }
      System.out.printf("\tdeg/sec\n") ;
      
      System.out.printf("recommend wheel speed") ;
      for (i=0 ; i<4 ; ++i)
      {
          System.out.printf("\t\t%.1f", this.recommendedWheelAngularVelDegPerSec[i]) ;  
      }
      System.out.printf("\tdeg/sec\n") ;

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
