// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

public class Drive extends SubsystemBase {
  public static double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);
  public static double[] [] ORIGIN_POINT = new double[4] [2]; //x,y coordinate of module point
  public static double ANGLE_RADIANS = 0;
  public static double[] VELOCITY_M_PER_SEC = new double [2];
  public static double ANGLE_VELOCITY_DEGREES_PER_SEC = 0;

  public static double[] STEERING_ANGLE_DEGREES = new double[4];
  public static double[] WHEEL_ANGULAR_VELOCITY_DEGREES_PER_SEC = new double[4];

  public static double[] LAST_STEERING_ANGLES_DEG = new double[4];
  public static double[] LAST_WHEEL_ANGULAR_VELOCITY_DEG_PER_SEC = new double[4];

  public static double[] DESIRED_STEERING_ANGLES_DEG = new double[4];
  public static double[] DESIRED_WHEEL_ANGULAR_VELOCITY_DEG_PER_SEC = new double[4];
  /** Creates a new Drive. */
  public Drive() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
