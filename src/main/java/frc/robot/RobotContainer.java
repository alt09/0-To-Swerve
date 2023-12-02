// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;

public class RobotContainer {

  private final CommandXboxController driveController;

  public RobotContainer() {
    switch (RobotConstants.getMode()) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
      System.out.println("Robot Current Mode; REAL");
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
      System.out.println("Robot Current Mode; SIM");
        break;
        
      // Replayed robot, disable IO implementations
      default:
      System.out.println("Robot Current Mode; default");
        break;
      }
    
      driveController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

      configureBindings();
  }


  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
