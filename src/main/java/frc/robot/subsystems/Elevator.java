package frc.robot.subsystems;

import org.frc5587.lib.subsystems.ElevatorBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevConstants;

public class Elevator extends ElevatorBase {
    private static CANSparkMax leftMotor = new CANSparkMax(ElevConstants.LEFT_MOTOR, MotorType.kBrushless);
    private static CANSparkMax rightMotor = new CANSparkMax(ElevConstants.RIGHT_MOTOR, MotorType.kBrushless);
    private static MotorControllerGroup motors = new MotorControllerGroup(leftMotor, rightMotor);

    public Elevator() {
        super(ElevConstants.constants, motors);
        SmartDashboard.putBoolean("ELEVATOR OUTPUTENABLED", true);
        super.pidController = getController();
        configureMotors();
        enable();
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

        leftMotor.setSmartCurrentLimit(ElevConstants.SUPPLY_LIMIT, ElevConstants.STATOR_LIMIT);
        rightMotor.setSmartCurrentLimit(ElevConstants.SUPPLY_LIMIT, ElevConstants.STATOR_LIMIT);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kCoast);

        leftMotor.burnFlash();
        rightMotor.burnFlash();

        resetEncoders();
    }

    @Override
    public void periodic() {
        super.periodic();
        // SmartDashboard.putNumber("Elev Output", get)
        if(!SmartDashboard.getBoolean("ELEVATOR OUTPUTENABLED", true)) {
            disable();
        }
        else if(SmartDashboard.getBoolean("ELEVATOR OUTPUTENABLED", true) && !isEnabled()) {
            enable();
        }
        // SmartDashboard.putNumber("SETPOINT", getController().getGoal().position);
        // SmartDashboard.putNumber("POSITION", getMeasurement());
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // double ff = ffController.calculate(setpoint.position, setpoint.velocity, setpoint.a);
        double ff = 0;
        //TODO: remove debug prints once we know this code works
        SmartDashboard.putNumber("ELEVATOR FEEDFORWARD", ff);
        SmartDashboard.putNumber("ELEVATOR OUTPUT USED", output);
        SmartDashboard.putNumber("ELEVATOR SETPOINT USED", setpoint.position);
        SmartDashboard.putNumber("ELEVATOR GOAL USED", pidController.getGoal().position);
        SmartDashboard.putBoolean("ELEVATOR AT SETPOINT", pidController.atGoal());
        SmartDashboard.putNumber("ELEVATOR POSITION", getMeasurement());

        /** if the driver has set output on, useOutput. */
        if(SmartDashboard.getBoolean("ELEVATOR OUTPUT ON?", true)) {
            /** output should be feedforward + calculated PID. */
            /** if the limit switch is pressed and the elevator is powered to move downward, set the voltage to 0 */
            // if(getLimitSwitchValue(constants.switchPorts[0]) && output < 0) {
            //     setVoltage(0);
            // }

            // /** if the elevator is above the limit and is powered to move upward, set the voltage to 0 */
            // else if(!getLimitSwitchValue(constants.switchPorts[0]) && getMeasurement() > constants.softLimits[1] && output > 0) {
            //     setVoltage(0);
            // }

            // else {
                setVoltage(output + ff);
            // }
        }
        else {
            setVoltage(0);
        }
    }

    public void elevatorTop() {
        setGoal(ElevConstants.TOP_POSITION);
    }

    public void elevatorBottom() {
        setGoal(ElevConstants.BOTTOM_POSITION);
    }

    public void elevatorMid() {
        setGoal(ElevConstants.MIDDLE_POSITION);
    }
}

