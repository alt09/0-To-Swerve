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

import edu.wpi.first.math.util.Units;
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
    public static final double LOOP_PERIOD_SECONDS = 0.02;

    public static class DrivetrainConstants {

      public static enum AbsoluteEncoder{
        FRONT_LEFT(8),
        FRONT_RIGTH(11),
        BACK_LEFT(5),
        BACK_RIGTH(2);
  
        public final int EncoderID;
        AbsoluteEncoder(int ID){
         EncoderID = ID;
        }
      }
  
      public static enum AbsoluteEncoderOffset{
        FRONT_LEFT(-87.6269),
        FRONT_RIGTH( -89.7363),
        BACK_LEFT(-356.0449),
        BACK_RIGTH(-177.6269);
       
        public final double offset;
        AbsoluteEncoderOffset(double value){
          offset = value;
        }
      }
  
      public static final double DRIVE_AFTER_ENCODER_REDUCTION = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  
      public enum DriveMotor {
        FRONT_LEFT(10),
        FRONT_RIGTH(13),
        BACK_LEFT(7),
        BACK_RIGTH(4);
  
        public final int CAN_ID;
  
        
  
        DriveMotor(int value) {
          CAN_ID = value;
        }
  
      }
  
      public enum TurnMotor {
        FRONT_LEFT(9),
        FRONT_RIGTH(12),
        BACK_LEFT(6),
        BACK_RIGTH(3);
  
        public final int CAN_ID;
  
        TurnMotor(int value) {
          CAN_ID = value;
        }
      }
  
      // Wheel Facts
      public static double WHEEL_DIAMETER_METERS = Units.inchesToMeters(2);
  
      // Under this to switch to coast when disabling
      public static final double COAST_THERESHOULD_METERS_PER_SEC = 0.05;
      // Under this speed for this length of time to switch to coast
      public static final double  COAST_THERESHOULD_SEC = 6.0;
      // Threshold to detect falls
      public static final double LEDS_FALLEN_ANGLE_DEGREES = 60.0;
  
      public static final double MAX_LINEAR_SPEED = 4.5; //meters/sec 
  
      public static final double TRACK_WIDTH_X = Units.inchesToMeters(25); //21 from center of wheel to center of other wheel but 22 from encoder to encoder 
      public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25); //21 from center of wheel to center of other wheel but 22 from encoder to encoder
  
      private static final boolean IS_BRAKING_MODE = false;
  
      public static boolean IS_CHARACTERAZING = false;
  
    }
    }
    public static class OperatorConstants {
      public static final int DRIVE_CONTROLLER = 0;
    }
} 
