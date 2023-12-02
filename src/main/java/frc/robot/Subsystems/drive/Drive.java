// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  
    public static double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);
    public static double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);
    public static double[][] ORIGIN_POINT = new double[4][2]; // x,y coords of the four modules
    // Vehicle states values
    public static double ANGLE_RADIANS = 0;
    public static double[] VEL_ORIGIN_METERS_PER_SEC = new double[2];
    public static double ANGLE_VEL_DEG_PER_SEC = 0; 

    public static double[] STEERING_ANGLE_DEG = new double[4];
    public static double[] WHEEL_ANGULAR_VEL_DEG_PER_SEC = new double [4];

    public static double[] LAST_STEEROMG_ANGLES_DEG = new double[4];
    public static double[] LAST_WHEEL_ANGULAR_VEL_DEG_PER_SEC = new double[4];

    public static double[] DESIRED_STEERING_ANGLE_DEG = new double[4];
    public static double[] DESIRED_WHEEL_ANGULAR_VEL_DEG_PER_SEC = new double[4];
  
    /** Creates a new Drive. */
  public Drive() {
  }

  public void assignVehicalValues(double d, double wheel_d) {
    this.WHEEL_DIAMETER_METERS = wheel_d;

    // Defome tje module locations
    this.ORIGIN_POINT[0][0] =  d;  // module 0 (+x, +y)
    this.ORIGIN_POINT[0][1] =  d;

    this.ORIGIN_POINT[1][0] = -d;  // module 1 (-x, +y)
    this.ORIGIN_POINT[1][1] =  d;

    this.ORIGIN_POINT[2][0] = -d;  // module 2 (-x, -y)
    this.ORIGIN_POINT[2][1] = -d;

    this.ORIGIN_POINT[3][0] =  d;  // module 3 (+x, -y)
    this.ORIGIN_POINT[3][1] = -d;
  }

public void calcSwerveSteeringAndSpeed() {
  int i;
  double sv, cv;

  cv = Math.cos(this.ANGLE_RADIANS);
  sv = Math.sin(this.ANGLE_RADIANS);

  // Calculate the vehicle velocity in the vehicle coordinate system
  double[] vO_IN_VEHICLE = new double[2];
  vO_IN_VEHICLE[0] =  cv * this.VEL_ORIGIN_METERS_PER_SEC[0] + sv * this.VEL_ORIGIN_METERS_PER_SEC[1];
  vO_IN_VEHICLE[1] = -sv * this.VEL_ORIGIN_METERS_PER_SEC[0] + cv * this.VEL_ORIGIN_METERS_PER_SEC[1];

  double[] v_i = new double[2];
  double speed;
  for (i = 0 ; i < 4 ; ++i) {
    
    v_i[0] = vO_IN_VEHICLE[0] -  * Units.degreesToRadians(this.ANGLE_VEL_DEG_PER_SEC);
  }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
