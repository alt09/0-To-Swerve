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

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class RobotConstants{
    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }


    
    public static Mode getMode() {
      if (RobotBase.isReal()) {
        return Mode.REAL;
      } else if (RobotBase.isSimulation()) {
        return Mode.SIM;
      } else {
        return Mode.REPLAY;
      }
    }
    
    public static Optional<Alliance> getAlliance() {
      return DriverStation.getAlliance();
    }
    
    public static final int CAN_CONFIG_TIMEOUT = 500;
    public static final double LOOP_PERIOD_SECS = 0.02;
  }

  public static class DriveConstants{

    // public static final double WHEEL_DIAMETER_METERS = 0.0762; //3 inches

    private static final double[][] ORIGIN_POINT = new double[4][2];  // four modules and each one has a x y coordinate so an array withing an array 

    //robot state values
    // private static final double ANGLE_RADIANS = 0.0;   //angle from xF to xV (comes from gyro sensor)
    // private static final double[] VELOCITY_POINT0_METERS_PER_SEC = new double[2];  //desired linear velocity of the robot
    // private static final double ANGLE_VELOCITY_DEGREES_PER_SEC = 0.0;  // desired angular velocity of robot chassis
    
    private static final double[] STEERING_ANGLES_DEGREES = new double[4];
    private static final double[] WHEEL_ANGULAR_VELOCITY_DEGREES_PER_SEC = new double[4];

    private static final double[] LAST_STEERING_ANGLES_DEGREES = new double[4];
    private static final double[] LAST_WHEEL_ANGULAR_VELOCITY_DEGREESPER_SEC = new double[4];

    // private static final double[] DESIRED_STEERING_ANGLES_DEGREES = new double[4];
    private static final double[] DESIRED_WHEEL_ANGULAR_VELOCITY_DEGREES_PER_SEC = new double[4];
  }

  public static class OperatorConstants {
    public static final int DRIVE_CONTROLLER = 0;
  }
} 
