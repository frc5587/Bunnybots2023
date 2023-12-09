// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DualStickSwerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer { 

  private final Elevator elevator = new Elevator();
  private final Swerve swerve = new Swerve();
  private final Wrist wrist = new Wrist();
  private final Intake intake = new Intake();

  private final CommandXboxController xbox = new CommandXboxController(0);
  private final CommandXboxController xbox2 = new CommandXboxController(1);
  private final DualStickSwerve driveCommand = new DualStickSwerve(swerve, xbox::getRightY, xbox::getRightX, xbox::getLeftX, () -> false);
  public RobotContainer() {
    swerve.setDefaultCommand(driveCommand);
    configureBindings();
  }
  /* Controller 2 Bindings
  DPad Up = elevatorUp
  DPad Down = elevatorDown
  Right Bumper = wristUp
  Left Bumper = wristDown
  A = Forward Intake
  B = Backward Intake
  */
  private void configureBindings() {
    xbox2.povUp().onTrue(new InstantCommand(elevator::elevatorUp));
    xbox2.povDown().onTrue(new InstantCommand(elevator::elevatorDown));
    xbox2.rightBumper().onTrue(new InstantCommand(wrist::wristUp));
    xbox2.leftBumper().onTrue(new InstantCommand(wrist::wristDown));
    xbox2.a().whileTrue(new InstantCommand(intake::forward));
    xbox2.b().whileTrue(new InstantCommand(intake::backward));
    xbox.x().whileTrue(new InstantCommand(elevator::elevatorUpSlow)).onFalse(new InstantCommand(elevator::stop));
    xbox.y().whileTrue(new InstantCommand(elevator::elevatorDownSlow)).onFalse(new InstantCommand(elevator::stop));
  }

 
  public Command getAutonomousCommand() {
    return null;
    // An example command will be run in autonomous
  }
}
