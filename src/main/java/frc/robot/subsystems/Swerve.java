package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SwerveBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.DrivetrainConstants;

public class Swerve extends SwerveBase {
    private static SwerveModule[] swerveModules = {
            new SwerveModule(DrivetrainConstants.Mod0.MODULE_CONSTANTS, new CANSparkMax(10, MotorType.kBrushless),
                    new CANSparkMax(15, MotorType.kBrushless), new CANcoder(50)),
            new SwerveModule(DrivetrainConstants.Mod1.MODULE_CONSTANTS, new CANSparkMax(11, MotorType.kBrushless),
                    new CANSparkMax(16, MotorType.kBrushless), new CANcoder(51)),
            new SwerveModule(DrivetrainConstants.Mod2.MODULE_CONSTANTS, new CANSparkMax(12, MotorType.kBrushless),
                    new CANSparkMax(17, MotorType.kBrushless), new CANcoder(52)),
            new SwerveModule(DrivetrainConstants.Mod3.MODULE_CONSTANTS, new CANSparkMax(13, MotorType.kBrushless),
                    new CANSparkMax(18, MotorType.kBrushless), new CANcoder(53))
    };

    public Swerve() {
        super(DrivetrainConstants.SWERVE_CONSTANTS, swerveModules);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("CANcoder val", swerveModules[0].getAbsoluteEncoderValue().getDegrees());
        SmartDashboard.putNumber("Non-Converted Val", swerveModules[0].getAngleMotorEncoderPosition().getDegrees());
        SmartDashboard.putNumber("Converted Val", swerveModules[0].getAngle().getDegrees());
        SmartDashboard.putNumber("Gyro yaw", gyro.getYaw().getDegrees());
        SmartDashboard.putNumber("Yaw offset", gyro.getYawZeroOffset().getDegrees());
        if(SmartDashboard.getBoolean("Zero Yaw", true)) {
                gyro.zeroYaw();
        }
        SmartDashboard.putBoolean("Zero Yaw", false);

        for (int i = 0; i < swerveModules.length; i++) {
                SmartDashboard.putNumber("mod "+i+"degrees", swerveModules[i].getRawAbsoluteEncoderValue().getDegrees());
                SmartDashboard.putNumber("Adjusted absolute "+i, swerveModules[i].getAbsoluteEncoderValue().getDegrees());
                SmartDashboard.putNumber("Motor " +i, swerveModules[i].getAngle().getDegrees());
            }
    }
}
