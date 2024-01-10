package frc.robot.subsystems;

import com.revrobotics.REVLibError;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    // public AHRS gyro;
    
    public SwerveDriveOdometry odometry;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveDriveKinematics kinematics;
    public TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(1.5); 
    
    public Field2d field = new Field2d();
    public Double lockedHeading = null;
   // public boolean hasResetToAbsolute = false;
    // private SlewRateLimiter slew = new SlewRateLimiter(SwerveConstants.SLEW_RATE);

    public Swerve() {
        // this.gyro = new AHRS();
        this.kinematics = SwerveConstants.SWERVE_KINEMATICS;

        this.mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Mod0.MODULECONSTANTS),
            new SwerveModule(1, SwerveConstants.Mod1.MODULECONSTANTS),
            new SwerveModule(2, SwerveConstants.Mod2.MODULECONSTANTS),
            new SwerveModule(3, SwerveConstants.Mod3.MODULECONSTANTS)
        };
        zeroGyro();

        this.odometry = new SwerveDriveOdometry(SwerveConstants.SWERVE_KINEMATICS, getYaw(), getModulePositions());
        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), getOdometryPose()); // Vision standard deviations.

        SmartDashboard.putData("Swerve Pose Field", field);
        SmartDashboard.putBoolean("Swerve Brake Mode", true);
        
        Timer.delay(3.0);
        resetModulesToAbsolute();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), 
                        translation.getY(), 
                        rotation, 
                        getYaw())
                    : new ChassisSpeeds(
                        translation.getX(), // -translation.getX(), 
                        translation.getY(), 
                        rotation));
        SmartDashboard.putNumber("Translation X", translation.getX());
        SmartDashboard.putNumber("Translation Y", translation.getY());
        SmartDashboard.putNumber("Rotation", rotation);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("DesiredAngle " + mod.moduleNumber, swerveModuleStates[mod.moduleNumber].angle.getDegrees());
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public Pose2d getPose() {
        return getEstimatedPose(); 
    }

    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        // gyro.zeroYaw();
        // gyro.setAngleAdjustment(pose.getRotation().getDegrees());

        odometry.resetPosition(getYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void zeroOdometry() {
        resetOdometry(new Pose2d());
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        lockedHeading = null;
        // gyro.zeroYaw();
    }

    public double getRoll() {
        // return gyro.getRoll();
        return 0;
    }

    public double getPitch() {
        // return gyro.getPitch();
        return 0;
    }

    public Rotation2d getYaw() {
        // return (SwerveConstants.INVERT_GYRO) ? Rotation2d.fromDegrees(-gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
        // return odometry.getPoseMeters().getRotation();
        return new Rotation2d();
        // return getYawFromOdom();
    }

    public Rotation2d getYawFromOdom() {
        return odometry.getPoseMeters().getRotation();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void stop() {
        setChassisSpeeds(new ChassisSpeeds());
    }

    /**
     * Set the robot's X speed in m/s.
     * @param speedMetersPerSecond speed to crawl at in m/s. Set to 0 to use speed from constants.
     */

    // private boolean modsStopped() {
    //     int stoppedModules = 0;
    //     for(SwerveModuleState state : getModuleStates()) {
    //         if(state.speedMetersPerSecond < 0.1) {
    //             stoppedModules ++;
    //         }
    //     }
    //     return stoppedModules == 4;
    // }

    @Override
    public void periodic() {
        //  if(!hasResetToAbsolute) {
        //     for (SwerveModule swerveModule : mSwerveMods) {
        //     if(swerveModule.mAngleMotor.getEncoder().setPosition(0).equals(REVLibError.kOk)) {
        //             swerveModule.resetToAbsolute();
        //         }
        //     }
        //     hasResetToAbsolute = true; 
        // }
        
        odometry.update(getYaw(), getModulePositions());
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions()); // ! If this is wrong, its probably a problem with getYaw()
        poseHistory.addSample(Timer.getFPGATimestamp(), getPose());
        field.setRobotPose(getPose());
        
        // DEBUGGING VALUES
        for (int i = 0; i < mSwerveMods.length; i++) {
            SmartDashboard.putNumber("mod " + i + "degrees", mSwerveMods[i].getCanCoder().getDegrees());
            SmartDashboard.putNumber("Motor " + i, mSwerveMods[i].getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Adjusted absolute " + i, mSwerveMods[i].getCanCoder().getDegrees() - mSwerveMods[i].angleOffset.getDegrees());
        }
        getModuleStates();
    }
}