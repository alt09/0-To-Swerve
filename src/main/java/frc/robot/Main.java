// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Subsystems.drive.Drive;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    // RobotBase.startRobot(Robot::new);
          
          Drive veh = new Drive();
          veh.assignVehicalValues(0.3302, 0.1016) ;
          
          double[] initial_steering_deg = {42.5, 33.6, 18.1, 21.9} ;
          double[] initial_wheel_speed_deg_per_sec = {1.56, 11.3, 4.2, 10.4} ;
          veh.initalizeSteeringAnglesAndWheelSpeeds(initial_steering_deg, initial_wheel_speed_deg_per_sec) ;
          
          // Example Case 1:
          // set the current vehicle angle
          veh.angleRadians = Units.degreesToRadians(30.0);
          
          // set the desired velocity (linear and angular for the vehicle)
          veh.velOriginMetersPerSec[0] =  0.5 ;
          veh.velOriginMetersPerSec[1] =  1.1 ;
          veh.angleVelDegPerSec  = 20.0 ;
          
          veh.calcSwerveSteeringAndSpeed() ;
          veh.recommendedSwerveSteeringAndSpeed() ;
                  
          veh.printResults() ;  // print results of case 1     
          
          // Example Case 2: (translation case)
          // set the current vehicle angle
          veh.angleRadians = Units.degreesToRadians(30.0);
          
          // set the desired velocity (linear and angular for the vehicle)
          veh.velOriginMetersPerSec[0] = 1.1 * Math.cos(Units.degreesToRadians(60.0));
          veh.velOriginMetersPerSec[1] = 1.1 * Math.sin(Units.degreesToRadians(60.0));
          veh.angleVelDegPerSec  = 0.0 ;
          
          veh.calcSwerveSteeringAndSpeed() ;
          veh.recommendedSwerveSteeringAndSpeed() ;
          
          veh.printResults() ;  // print results of case 2 
          
          // Example Case 3: (recommend reverse wheel rotation based on current steering)
          // set the current vehicle angle
          veh.angleRadians = Units.degreesToRadians(30.0);
          
          // set the desired velocity (linear and angular for the vehicle)
          veh.velOriginMetersPerSec[0] = -1.1 * Math.cos(Units.degreesToRadians(60.0)) ;
          veh.velOriginMetersPerSec[1] = -1.1 * Math.sin(Units.degreesToRadians(60.0)) ;
          veh.angleVelDegPerSec  = -25.0 ;
          
          veh.calcSwerveSteeringAndSpeed() ;
          veh.recommendedSwerveSteeringAndSpeed() ;
          
          veh.printResults() ;  // print results of case 3 
      }
  }


