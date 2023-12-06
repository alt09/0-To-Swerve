// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
  /** Creates a new Drivetrain. */
  //construtor; builds instance of class
  public Drive() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //constants that are in constants but aren't constants
  public static final double WHEEL_DIAMETER_M = Units.inchesToMeters(1.5); //"final" means u can't update it later even with a "this."
  public static final double CHASSIS_WIDTH_M = Units.inchesToMeters(21.5);
  public static final double[][] ORIGIN_POINT = { //array of xy coordinate systems
    {CHASSIS_WIDTH_M, CHASSIS_WIDTH_M},
    {-CHASSIS_WIDTH_M, CHASSIS_WIDTH_M},
    {-CHASSIS_WIDTH_M, -CHASSIS_WIDTH_M},
    {CHASSIS_WIDTH_M, -CHASSIS_WIDTH_M}};

  double chassisAngleRad;
  double[] chassisVelMPerSec = new double[2]; //joystick input; desired v_x and v_y
  double chassisAngVelDegPerSec; //

  double[] currentSteeringDeg = new double[4];
  double[] currentWheelAngVelDegPerSec = new double[4];

  double[] calculatedSteeringDeg = new double[4];
  double[] calculatedWheelAngVelDegPerSec = new double[4];

  double[] recSteeringDeg = new double[4];
  double[] recWheelAngVelDegPerSec = new double[4];

  void initialize(double[] steering_ang_deg, double[] wheel_speeds) {
    for(int i = 0; i < 4; i++) {
      currentSteeringDeg[i] = steering_ang_deg[i];
      currentWheelAngVelDegPerSec[i] = wheel_speeds[i];
    }
  }

  void calc_swerve() {
    double c_v = Math.cos(this.chassisAngleRad); //just used for prettier calcs
    double s_v = Math.sin(this.chassisAngleRad);
    double[] v = new double [2]; //v_x and v_y of chassis (axis of grid)

    //matrices calculations
    v[0] = c_v * this.chassisVelMPerSec[0] + s_v * this.chassisVelMPerSec[1];
    v[1] = -s_v * this.chassisVelMPerSec[0] + c_v * this.chassisVelMPerSec[1];

    double[] v_i = new double[2]; //change in v
    double speed;

    for(int i = 0; i < 4; i++) {
      v_i[0] = v[0] - Math.toRadians(this.chassisAngVelDegPerSec) * this.ORIGIN_POINT[i][1];
      v_i[0] = v[1] + Math.toRadians(this.chassisAngVelDegPerSec) * this.ORIGIN_POINT[i][0];

      speed = Math.sqrt(v_i[0] * v_i[0] + v_i[1] * v_i[1]);
      this.calculatedSteeringDeg[i] = Math.toDegrees(Math.atan2(v_i[1], v_i[0]));
      this.calculatedWheelAngVelDegPerSec[i] = Math.toDegrees(speed / (WHEEL_DIAMETER_M / 2.0));
    }
  }

  void rec_swerve() {
    double angDiffDeg;
    for(int i = 0; i < 4; i++) {
      angDiffDeg = this.calculatedSteeringDeg[i] - this.currentSteeringDeg[i];

      while(angDiffDeg < 180.0)
        angDiffDeg += 360.0;
      
      while(angDiffDeg > 180.0)
        angDiffDeg += 360.0;

      if(Math.abs(angDiffDeg) < 90.0) {
        this.recSteeringDeg[i] = this.currentSteeringDeg[i] + angDiffDeg;
        this.recWheelAngVelDegPerSec[i] = this.calculatedWheelAngVelDegPerSec[i];
      } else {
        // this.recSteeringDeg[i] = this.currentSteeringDeg[i] + angDiffDeg;
        // this.recWheelAngVelDegPerSec[i] = this.calculatedWheelAngVelDegPerSec[i];
      }

    }
  }
}