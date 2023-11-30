package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.swervelib.math.Conversions;
import frc.robot.util.swervelib.util.CTREConfigs;
import frc.robot.util.swervelib.util.CTREModuleState;
import frc.robot.util.swervelib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    public Rotation2d angleOffset;
    private Rotation2d lastAngle;
    public static CTREConfigs ctreConfigs;

    public CANSparkMax mAngleMotor;
    public CANSparkMax mDriveMotor;
    public CANCoder angleEncoder;

    SimpleMotorFeedforward feedforward = SwerveConstants.DRIVE_FF;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        ctreConfigs = new CTREConfigs();
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID, "canivore");
        // angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        mAngleMotor.getPIDController().setP(SwerveConstants.ANGLE_FPID.kP);
        mAngleMotor.getPIDController().setI(SwerveConstants.ANGLE_FPID.kI);
        mAngleMotor.getPIDController().setD(SwerveConstants.ANGLE_FPID.kD);
        mAngleMotor.getPIDController().setFF(SwerveConstants.ANGLE_FPID.kF);

        
        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        // mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SwerveConstants.WHEEL_CIRCUMFERENCE_METERS, SwerveConstants.DRIVE_GEAR_RATIO);
            mDriveMotor.set(velocity);
        }
    }

    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.MAX_SPEED * 0.05)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 5%. Prevents Jittering.
        
        mAngleMotor.getPIDController().setReference(angle.getRotations(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getEncoder().getPosition(), SwerveConstants.ANGLE_GEAR_RATIO));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), SwerveConstants.ANGLE_GEAR_RATIO);
        mAngleMotor.getEncoder().setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.restoreFactoryDefaults();
        // mAngleMotor.configAllSettings(ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(SwerveConstants.ANGLE_MOTOR_INVERTED);
        mAngleMotor.setIdleMode(IdleMode.kBrake);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.restoreFactoryDefaults();
        // mDriveMotor.configAllSettings(ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(SwerveConstants.DRIVE_MOTOR_INVERTED);
        mDriveMotor.setIdleMode(IdleMode.kBrake);
        mDriveMotor.getEncoder().setPosition(0);

    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            -Conversions.falconToMPS(mDriveMotor.getEncoder().getVelocity(), SwerveConstants.WHEEL_CIRCUMFERENCE_METERS, SwerveConstants.DRIVE_GEAR_RATIO),
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            -Conversions.falconToMeters(mDriveMotor.getEncoder().getPosition(), SwerveConstants.WHEEL_CIRCUMFERENCE_METERS, SwerveConstants.DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }
}