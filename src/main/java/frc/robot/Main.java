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
import frc.robot.Subsystems.Drive;

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
   veh.ASSIGN_VEHICLE_VALUES_FOR_SQUARE_VEHICLE(0.3302, 0.1016) ;
   
   double[] initial_steering_deg = {42.5, 33.6, 18.1, 21.9} ;
   double[] initial_wheel_speed_deg_per_sec = {1.56, 11.3, 4.2, 10.4} ;
   veh.initializeSteeringAnglesAndWheelSpeeds(initial_steering_deg,
                                       initial_wheel_speed_deg_per_sec) ;
   
   // Example Case 1:
   // set the current vehicle angle
   veh.ANGLE_RADIANS = Units.degreesToRadians(30.0) ;
   
   // set the desired velocity (linear and angular for the vehicle)
   veh.VELOCITY_M_PER_SEC[0] =  0.5 ;
   veh.VELOCITY_M_PER_SEC[1] =  1.1 ;
   veh.ANGLE_VELOCITY_DEGREES_PER_SEC  = 20.0 ;
   
   veh.CALCULATION_SWERVE_STEERING_AND_SPEED() ;
   veh.RECOMMENDED_SWERVE_STEERING_AND_SPEED() ;
           
   veh.printResult();  // print results of case 1     
   
   // Example Case 2: (translation case)
   // set the current vehicle angle
   veh.ANGLE_RADIANS = Units.degreesToRadians(30.0) ;
   
   // set the desired velocity (linear and angular for the vehicle)
   veh.VELOCITY_M_PER_SEC[0] = 1.1 * Math.cos(Units.degreesToRadians(60.0)) ;
   veh.VELOCITY_M_PER_SEC[1] = 1.1 * Math.sin(Units.degreesToRadians(60.0)) ;
   veh.ANGLE_VELOCITY_DEGREES_PER_SEC  = 0.0 ;
   
   veh.CALCULATION_SWERVE_STEERING_AND_SPEED() ;
   veh.RECOMMENDED_SWERVE_STEERING_AND_SPEED() ;
   
   veh.printResult() ;  // print results of case 2 
   
   // Example Case 3: (recommend reverse wheel rotation based on current steering)
   // set the current vehicle angle
   veh.ANGLE_RADIANS = Units.degreesToRadians(30.0) ;
   
   // set the desired velocity (linear and angular for the vehicle)
   veh.VELOCITY_M_PER_SEC[0] = -1.1 * Math.cos(Units.degreesToRadians(60.0)) ;
   veh.VELOCITY_M_PER_SEC[1] = -1.1 * Math.sin(Units.degreesToRadians(60.0)) ;
   veh.ANGLE_VELOCITY_DEGREES_PER_SEC  = -25.0 ;
   
   veh.CALCULATION_SWERVE_STEERING_AND_SPEED() ;
   veh.RECOMMENDED_SWERVE_STEERING_AND_SPEED() ;
   
   veh.printResult() ;  // print results of case 3 
  }
}

