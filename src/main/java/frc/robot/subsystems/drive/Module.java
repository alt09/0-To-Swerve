// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;

class main {
    public static void notmain( String args[] ) {
        
        Drive veh = new Drive();
        veh.assign_veh_values_for_square_vehicle(0.3302, 0.1016) ;
        
        double[] initial_steering_deg = {42.5, 33.6, 18.1, 21.9} ;
        double[] initial_wheel_speed_deg_per_sec = {1.56, 11.3, 4.2, 10.4} ;
        veh.initialize_steering_angles_and_wheel_speeds(initial_steering_deg,
                                            initial_wheel_speed_deg_per_sec) ;
        
        // Example Case 1:
        // set the current vehicle angle
        veh.angle = Units.degreesToRadians( 30.0)  ;
        
        // set the desired velocity (linear and angular for the vehicle)
        veh.centerVel[0] =  0.5 ;
        veh.centerVel[1] =  1.1 ;
        veh.angularVel  = 20.0 ;
        
        veh.calc_swerve_steering_and_speed() ;
        veh.recommended_swerve_stering_and_speed() ;
                
        veh.print_results() ;  // print results of case 1     
        
        // Example Case 2: (translation case)
        // set the current vehicle angle
        veh.angle = Units.degreesToRadians(30.0 ) ;
        
        // set the desired velocity (linear and angular for the vehicle)
        veh.centerVel[0] = 1.1 * Math.cos(Units.degreesToRadians(60.0)) ;
        veh.centerVel[1] = 1.1 * Math.sin(Units.degreesToRadians(60.0)) ;
        veh.angularVel  = 0.0 ;
        
        veh.calc_swerve_steering_and_speed() ;
        veh.recommended_swerve_stering_and_speed() ;
        
        veh.print_results() ;  // print results of case 2 
        
        // Example Case 3: (recommend reverse wheel rotation based on current steering)
        // set the current vehicle angle
        veh.angle = Units.degreesToRadians(30.0 );
        
        // set the desired velocity (linear and angular for the vehicle)
        veh.centerVel[0] = -1.1 * Math.cos(Units.degreesToRadians(60.0)) ;
        veh.centerVel[1] = -1.1 * Math.sin(Units.degreesToRadians(60.0)) ;
        veh.angularVel  = -25.0 ;
        
        veh.calc_swerve_steering_and_speed() ;
        veh.recommended_swerve_stering_and_speed() ;
        
        veh.print_results() ;  // print results of case 3 
    }

}


