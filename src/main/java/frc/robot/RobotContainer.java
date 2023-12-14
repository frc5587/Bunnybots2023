// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandCommandXboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DualStickSwerve;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.TriggerWrist;
import frc.robot.commands.WristDown;
import frc.robot.commands.WristUp;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // SUBSYSTEMS:
    private final Elevator elevator = new Elevator();
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Wrist wrist = new Wrist();

    // CONTROLLERS:
    private final DeadbandCommandXboxController xbox = new DeadbandCommandXboxController(0, 0.4);
    private final DeadbandCommandXboxController xbox2 = new DeadbandCommandXboxController(1, 0.4);

    // COMMANDS:
    private final DualStickSwerve driveCommand = new DualStickSwerve(swerve, xbox::getRightY, xbox::getRightX,
            xbox::getLeftX, () -> false);
    // private final TriggerWrist triggerwWrist = new TriggerWrist(wrist, xbox::getLeftTriggerAxis, xbox::getRightTriggerAxis);
            
  public RobotContainer() {
    swerve.setDefaultCommand(driveCommand);
    // wrist.setDefaultCommand(triggerwWrist);
    configureBindings();
  }
  /* Controller 2 Bindings
  DPad Up = elevatorTop
  DPad Down = elevatorBottom
  Right Bumper = wristTop
  Left Bumper = wristBottom
  A = Forward Intake
  B = Backward Intake
  X = Manual ElevatorUp
  Y = Manual ElevatorDown
  Right Trigger = Manual WristUp
  Left Trigger = Manual WristDown
  */
  private void configureBindings() {
    xbox2.povUp().onTrue(new InstantCommand(elevator::elevatorTop));
    xbox2.povDown().onTrue(new InstantCommand(elevator::elevatorBottom));
    xbox2.y().onTrue(new InstantCommand(wrist::wristTop));
    xbox2.b().onTrue(new InstantCommand(wrist::wristMid));
    xbox2.a().onTrue(new InstantCommand(wrist::wristBottom));
    xbox2.rightBumper().whileTrue(new InstantCommand(intake::forward)).onFalse(new InstantCommand(intake::stop));
    xbox2.leftBumper().whileTrue(new InstantCommand(intake::backward)).onFalse(new InstantCommand(intake::stop));
  }

 
  public Command getAutonomousCommand() {
    return null;
    // An example command will be run in autonomous
  }
}
