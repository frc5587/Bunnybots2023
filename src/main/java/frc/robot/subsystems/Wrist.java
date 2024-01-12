package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.WristConstants;

public class Wrist extends PivotingArmBase {
    public static CANSparkMax motor = new CANSparkMax(WristConstants.MOTOR_ID, MotorType.kBrushless); 

    private final DigitalInput frontLimitSwitch = new DigitalInput(WristConstants.SWITCH_PORTS[0]);
    private final DigitalInput rearLimitSwitch = new DigitalInput(WristConstants.SWITCH_PORTS[1]);
    // private final DutyCycleEncoder throughBore = new DutyCycleEncoder(1);

    public static PivotingArmConstants constants = new PivotingArmConstants(
        WristConstants.GEARING,
        1,
        0,
        WristConstants.SOFT_LIMITS,
        WristConstants.ZERO_OFFSET,
        WristConstants.ENCODER_CPR,
        WristConstants.PID,
        WristConstants.FF
    );

    public Wrist() {
        this(motor);
    }
    
    public Wrist(CANSparkMax motor) {
        super("wrist", constants, motor);
        enable();
        // resetEncoders();
        setGoal(0);
        // throughBore.setDutyCycleRange(1./1024., 1023./1024.); // change depending on us range
    }

    public DigitalInput getFrontLimitSwitch() {
        return frontLimitSwitch;
    }

    public DigitalInput getRearLimitSwitch() {
        return rearLimitSwitch;
    }

    @Override
    public double getEncoderPosition() {
        // return -(throughBore.getAbsolutePosition() - throughBore.getPositionOffset());
        return motor.getEncoder().getPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public void setEncoderPosition(double position) {
        motor.getEncoder().setPosition(position);
    }

    @Override
    public void configureMotors() {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(WristConstants.MOTOR_INVERTED);
        motor.setSmartCurrentLimit(WristConstants.STALL_LIMIT, WristConstants.FREE_LIMIT);
        // resetEncoders();
    }
    
    public void forward() {
        motor.set(0.3);
    }

    public void back() {
        motor.set(-0.3);
    }

    public void wristTop() {
        setGoal(WristConstants.TOP_SETPOINT);
    }

    public void wristMid() {
        setGoal(WristConstants.RESTING_SETPOINT);
    }

    public void wristBottom() {
        setGoal(WristConstants.BOTTOM_SETPOINT);
    }

    @Override
    public void periodic() {
        super.periodic();
        if(SmartDashboard.getBoolean("Reset Encoders", false)) {
            resetEncoders();
        }
        SmartDashboard.putBoolean("Reset Encoders", false);

        // if(!throughBore.isConnected()) {
        //     this.disable();
        //     this.stop();
        // }
    }
}
