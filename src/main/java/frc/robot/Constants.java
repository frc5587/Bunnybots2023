// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.pid.FPID;
import org.frc5587.lib.subsystems.ElevatorBase.ElevatorConstants;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.util.swervelib.util.COTSFalconSwerveConstants;
import frc.robot.util.swervelib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class SwerveConstants {
    public static final boolean INVERT_GYRO = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants CHOSEN_MODULE = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(19.51); // distance from left wheel to right wheel
        public static final double WHEEL_BASE = Units.inchesToMeters(21.25); // distance from front wheel to back wheel
        public static final double WHEEL_CIRCUMFERENCE_METERS = CHOSEN_MODULE.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));


        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERTED = CHOSEN_MODULE.driveMotorInvert;
        public static final boolean ANGLE_MOTOR_INVERTED = CHOSEN_MODULE.angleMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERTED = CHOSEN_MODULE.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int DRIVE_CONT_LIMIT = 35;
        public static final int DRIVE_PEAK_LIMIT = 40;
        public static final double DRIVE_PEAK_DURATION = 0.1;
        public static final boolean DRIVE_LIMIT_ENABLED = true;
        public static final int RUMBLE_THRESHOLD = 35;
        
        public static final double SLEW_RATE = 3; // m/s^2 // 

        public static final int ANGLE_CONT_LIMIT = 25;
        public static final int ANGLE_PEAK_LIMIT = 40;
        public static final double ANGLE_PEAK_DURATION = 0.1;
        public static final boolean ANGLE_LIMIT_ENABLED = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Drive Motor PID Values */
        public static final FPID DRIVE_FPID = new FPID(
            0.02, 0.1, 0, 0);//0.05, 0.03, 0., 0.); // //2.8884 for P

        /* Angle Motor PID Values */
        public static final FPID ANGLE_FPID = new FPID(
                CHOSEN_MODULE.angleKF, CHOSEN_MODULE.angleKP, CHOSEN_MODULE.angleKI, CHOSEN_MODULE.angleKD); // 0.05
        

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        //COMMENTED VALS IN () ARE FROM BASEFALCONSWERVE, OTHER COMMENTED VALS ARE SYSID, USED VALS ARE FROM FRESTA
        public static final double DRIVE_KS = (0.32 / 12);//.18576/12; // 0.23034/12; ;
        public static final double DRIVE_KV = (1.51 / 12);//2.3317/12; // 2.6998/12; // ;
        public static final double DRIVE_KA = (0.27 / 12);//0.25916/12; // 0.29868/12; // 
        public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 2.5;//5.;
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = Math.PI;//6.;

        /* Idle Modes */
        public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;
        public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_ID = 10;
            public static final int ANGLE_ID = 15;
            public static final int CANCODER_ID = 50;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(213.75);
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int DRIVE_ID = 11;
            public static final int ANGLE_ID = 16;
            public static final int CANCODER_ID = 51;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(192.129);
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_ID = 12;
            public static final int ANGLE_ID = 17;
            public static final int CANCODER_ID = 52;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(256.008); // 260.332
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int DRIVE_ID = 13;
            public static final int ANGLE_ID = 18;
            public static final int CANCODER_ID = 53;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(306.474); // 194.169;
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }
    }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /* Intake Constants */
  public static final class IntakeConstants {
    // motor ports
    public static final int RIGHT_MOTOR = 40;
    public static final int LEFT_MOTOR = 41;
    
    public static final boolean RIGHT_MOTOR_INVERTED = false;
    public static final boolean LEFT_MOTOR_INVERTED = true;
    // motor limits
    public static final int STALL_LIMIT = 20;
    public static final int FREE_LIMIT = 25;
    // motor speeds
    public static final double THROTTLE_FORWARD = 0.75;
    public static final double THROTTLE_REVERSE = 0.25;
    public static final double LEFT_VELOCITY_THRESHOLD = 10;  // rps
    public static final double EJECT_CRATE_RUNTIME = 0.5; // seconds
    public static final double RIGHT_VELOCITY_THRESHOLD = 500;  // rps
  
    public static final double HOLD = 0.05;
  
    public static final int SOLENOID_ID = 0;
    public static final int SOLENOID_ID2 = 1;
  }

  public static final class ElevConstants {
    public static final boolean MOTOR_INVERTED = false;
    public static final int SUPPLY_LIMIT = 40;
    public static final int STATOR_LIMIT = 40;

    public static final double TOP_POSITION = 1.3;
    public static final double BOTTOM_POSITION = 0.25;

    public static final int LEFT_MOTOR = 20;
    public static final int RIGHT_MOTOR = 21;
    public static final double GEARING = 0.1; 
    public static final double RTM = 10;
    public static final double[] SOFT_LIMITS = {0, 1};
    public static final int ZERO_OFFSET = 10;
    public static final int CPR = 2048;
    public static final int[] SWITCH_PORT = {0, 1};
    public static final boolean[] SWITCHES_INVERTED = {false, false};
    public static final ProfiledPIDController PID = new ProfiledPIDController(1, 0, 0, 
        new Constraints(1, 1)
    );
    public static final ElevatorFeedforward FF = new ElevatorFeedforward(1, 0, 0, 0);
    public static final ElevatorConstants constants = new ElevatorConstants(GEARING, RTM, SOFT_LIMITS, ZERO_OFFSET, CPR,
            SWITCH_PORT, SWITCHES_INVERTED, PID, FF);
  }

  public static final class WristConstants {
    public static final int MOTOR_ID = 30;
    public static final boolean MOTOR_INVERTED = false;

    public static final double TOP_SETPOINT = Math.toRadians(30);
    public static final double BOTTOM_SETPOINT = Math.toRadians(0);
    
    public static final double GEARING = 64;
    public static final double[] SOFT_LIMITS = {Math.toRadians(0), Math.toRadians(180)};
    public static final int ZERO_OFFSET = 0;
    public static final int ENCODER_CPR = 42;
    public static final int[] SWITCH_PORTS = {1};
    public static final boolean[] SWITCH_INVERTIONS = {false, false};
    public static final ProfiledPIDController PID = new ProfiledPIDController(6.9012, 0, 1.5006, new Constraints(5, 50));
    public static final ArmFeedforward FF = new ArmFeedforward(0.34905, -0.032397, 1.23, 0.083365);
  }
}
