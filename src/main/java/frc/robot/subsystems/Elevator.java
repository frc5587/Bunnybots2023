package frc.robot.subsystems;

import org.frc5587.lib.subsystems.ElevatorBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevConstants;

public class Elevator extends ElevatorBase {
    private static CANSparkMax leftMotor = new CANSparkMax(ElevConstants.LEFT_MOTOR, MotorType.kBrushless);
    private static CANSparkMax rightMotor = new CANSparkMax(ElevConstants.RIGHT_MOTOR, MotorType.kBrushless);
    private static MotorControllerGroup motors = new MotorControllerGroup(leftMotor, rightMotor);

    public Elevator() {
        super(ElevConstants.constants, motors);
        getController().disableContinuousInput();
    }

    public void elevatorUpSlow() {
        setVoltage(2);
    }

    public void elevatorDownSlow() {
        setVoltage(-2);
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

        leftMotor.setInverted(ElevConstants.LEFT_MOTOR_INVERTED);
        rightMotor.setInverted(ElevConstants.RIGHT_MOTOR_INVERTED);

        // leftMotor.setSmartCurrentLimit(ElevConstants.SUPPLY_LIMIT, ElevConstants.STATOR_LIMIT);
        // rightMotor.setSmartCurrentLimit(ElevConstants.SUPPLY_LIMIT, ElevConstants.STATOR_LIMIT);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LMOTOR SET TO", leftMotor.get());
        SmartDashboard.putNumber("RMOTOR SET TO", rightMotor.get());
        SmartDashboard.putNumber("LVOLTAGE", leftMotor.getBusVoltage());
        SmartDashboard.putNumber("RVOLTAGE", rightMotor.getBusVoltage());
        SmartDashboard.putNumber("SETPOINT", getController().getSetpoint().position);
    }
    public void elevatorTop() {
        setGoal(ElevConstants.TOP_POSITION);
    }

    public void elevatorBottom() {
        setGoal(ElevConstants.BOTTOM_POSITION);
    }
}

