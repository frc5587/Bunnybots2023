package frc.robot.subsystems;

import org.frc5587.lib.subsystems.ElevatorBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.ElevConstants;

public class Elevator extends ElevatorBase {
    private static ElevatorConstants constants = ElevConstants.CONSTANTS;
    private static CANSparkMax leftMotor = new CANSparkMax(ElevConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private static CANSparkMax rightMotor = new CANSparkMax(ElevConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private static MotorControllerGroup motors = new MotorControllerGroup(leftMotor, rightMotor);

    public Elevator() {
        super(constants, motors);
    }

    public void elevatorUp() {
        setGoal(ElevConstants.TOP_POSITION);
    }

    public void elevatorDown() {
        setGoal(ElevConstants.BOTTOM_POSITION);
    }

    @Override
    public double getEncoderPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return leftMotor.getEncoder().getVelocity();
    }

    @Override
    public void setEncoderPosition(double position) {
        leftMotor.getEncoder().setPosition(position);
    }

    @Override
    public void configureMotors() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftMotor.setInverted(ElevConstants.MOTOR_INVERTED);
        rightMotor.setInverted(ElevConstants.MOTOR_INVERTED);
        leftMotor.setSmartCurrentLimit(ElevConstants.STALL_LIMIT, ElevConstants.FREE_LIMIT);
        rightMotor.setSmartCurrentLimit(ElevConstants.STALL_LIMIT, ElevConstants.FREE_LIMIT);
    }
}
