package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frc5587.lib.subsystems.SimpleMotorBase;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SimpleMotorBase {
    private static final CANSparkMax rightIntake = new CANSparkMax(IntakeConstants.RIGHT_MOTOR, MotorType.kBrushless);
    private static final CANSparkMax leftIntake = new CANSparkMax(IntakeConstants.LEFT_MOTOR, MotorType.kBrushless);

    public Intake() {
        super(new MotorControllerGroup(rightIntake, leftIntake), IntakeConstants.THROTTLE_FORWARD, IntakeConstants.THROTTLE_REVERSE);
        configureMotors();
    }

    /**
     * Configures the motors, this includes inversions, current limits, and idle modes.
     */
    @Override
    public void configureMotors() {
        rightIntake.restoreFactoryDefaults();
        leftIntake.restoreFactoryDefaults();

        rightIntake.setInverted(IntakeConstants.RIGHT_MOTOR_INVERTED);
        leftIntake.setInverted(IntakeConstants.LEFT_MOTOR_INVERTED);

        rightIntake.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);
        leftIntake.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);

        rightIntake.setIdleMode(IdleMode.kBrake);
        leftIntake.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("left v", leftVelocity());
        SmartDashboard.putNumber("right v", rightVelocity());
    }

    private double leftVelocity() {
        return leftIntake.getEncoder().getVelocity();
    }

    private double rightVelocity() {
        return rightIntake.getEncoder().getVelocity();
    }
}