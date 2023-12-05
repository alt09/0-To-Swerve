// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import java.lang.Math;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private  double WHEEL_DIAMETER_METERS = 0.0762; //3 inches
  private  double ANGLE_RADIANS;
  private  double[][] ORIGIN_POINT = new double[4][2];  // four modules and each one has a x y coordinate so an array withing an array 
  private  double[] VELOCITY_POINT0_METERS_PER_SEC = new double[2];  //desired linear velocity of the robot
  private  double ANGLE_VELOCITY_DEGREES_PER_SEC;  // desired angular velocity of robot chassis 
  private double[] DESIRED_STEERING_ANGLES_DEGREES = new double[4];
  public Drivetrain() {
  }


  //track width 26 
  //wheel diameter 4 inc 
  public void assignVehValuesForSquareRobot(double d, double wheelD){ 
    //sets the wheel diameter 
    this.WHEEL_DIAMETER_METERS = wheelD;

    //define the locations of the four swerve modules in the xv, yv coord sys
    this.ORIGIN_POINT[0][0] = d; // module 0 is in the +x. +y quadrant
    this.ORIGIN_POINT[0][1] = d;

    this.ORIGIN_POINT[1][0] = -d; // module 1 is in the -x. +y quadrant
    this. ORIGIN_POINT[1][1] = d; 

    this.ORIGIN_POINT[2][0] = -d; // module 2 is in the -x. -y quadrant
    this.ORIGIN_POINT[2][1] = -d;

    this.ORIGIN_POINT[3][0] = d; // module 0 is in the +x. -y quadrant
    this.ORIGIN_POINT[3][1] = -d;
  }

  public void calcSwerveSteeringAndSpeed(){
  int i;
  double sineValue, cosineValue;

  cosineValue = Math.cos(this.ANGLE_RADIANS);
  sineValue = Math.sin(this.ANGLE_RADIANS);

  // calculate the vehicle velocity in the which coord sys
  double[] v0InRobot = new double[2];
  v0InRobot[0] = cosineValue * this.VELOCITY_POINT0_METERS_PER_SEC[0] + sineValue * this.VELOCITY_POINT0_METERS_PER_SEC[1];  // this is your original pythagorem theorm 
  v0InRobot[1] = -sineValue * this.VELOCITY_POINT0_METERS_PER_SEC[0] + cosineValue * this.VELOCITY_POINT0_METERS_PER_SEC[1];

  double[] v_i = new double[2];
  double speed;

  for (i=0; i<4; ++i) {
    v_i[0] = v0InRobot[0] - this.ANGLE_VELOCITY_DEGREES_PER_SEC*Units.degreesToRadians(ORIGIN_POINT[i][1]);  // v_i is the new pythagorem theorm value 
    v_i[1] = v0InRobot[1] + this.ANGLE_VELOCITY_DEGREES_PER_SEC*Units.degreesToRadians(ORIGIN_POINT[i][0]);

    speed = Math.sqrt(v_i[0]*v_i[0] + v_i[1]*v_i[1]);
    this.DESIRED_STEERING_ANGLES_DEGREES[i] = Units.degreesToRadians(Math.atan2(v_i[1], v_i[0]));
    this.
  }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
