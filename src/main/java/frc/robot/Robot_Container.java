// Copyright 2021-2025 FRC 6328
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

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Drive_Constants;
import frc.robot.subsystems.swerve.Module_IO;
import frc.robot.subsystems.swerve.Module_IO_Real;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class Robot_Container {
  // Subsystems
    @SuppressWarnings ("unused")
    private final Swerve swerve;
    
    
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController adjController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public Robot_Container() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        swerve = new Swerve(
            new Module_IO_Real
            (Drive_Constants.frontLeftDriveCanId, Drive_Constants.frontLeftTurnCanId),
            new Module_IO_Real
            (Drive_Constants.frontRightDriveCanId, Drive_Constants.frontRightTurnCanId),
            new Module_IO_Real
            (Drive_Constants.backLeftDriveCanId, Drive_Constants.backLeftTurnCanId),
            new Module_IO_Real
            (Drive_Constants.backRightDriveCanId, Drive_Constants.backRightTurnCanId)
            
        );
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        swerve = new Swerve(
            new Module_IO() {},
            new Module_IO() {},
            new Module_IO() {},
            new Module_IO() {}   );
        break;

      default:
        // Replayed robot, disable IO implementations
        swerve = new Swerve(
            new Module_IO() {},
            new Module_IO() {},
            new Module_IO() {},
            new Module_IO() {}   );
        break;
    }


    //Add Named Comands here
    

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
   

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive      
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
