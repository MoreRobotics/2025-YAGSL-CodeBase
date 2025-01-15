/*
 * This subsystem manages the 
 */

package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Swerve extends SubsystemBase {

    // subsystems

    // WPILib class objects
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SwerveDriveKinematics swerveKinematics = Constants.Swerve.swerveKinematics;
    public Translation2d frontLeftModule;
    public Translation2d frontRightModule;
    public Translation2d backLeftModule;
    public Translation2d backRightModule;
    public SwerveModuleState[] swerveModuleStates;
    private Field2d m_field;
    private DoubleArrayPublisher moduleStatePublisher;
    private StructPublisher<Pose2d> posePublisher;
    public StructArrayPublisher<SwerveModuleState> swerveKinematicsPublisher;
    public StructPublisher<Pose2d> estimatedRobotPosePublisher;
    public SwerveDrivePoseEstimator m_poseEstimator;
    

    // constructor
    public Swerve() {

        // instantiate objects 
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        m_field = new Field2d();

        // set gyro
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(Constants.Swerve.gyroOffset);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        // delay reseting modules utill robRIO finishes startup
        Timer.delay(1.0);
        resetModulesToAbsolute();
        swerveOdometry = new SwerveDriveOdometry(swerveKinematics, getGyroYaw(), getModulePositions());
      
        // create loggers
        moduleStatePublisher = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("/ModuleStates").publish();
        posePublisher = NetworkTableInstance.getDefault().getStructTopic("/MyPose", Pose2d.struct).publish();
        swerveKinematicsPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveModuleStates", SwerveModuleState.struct).publish();
        estimatedRobotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("/EstimatedRobotPose", Pose2d.struct).publish();
        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("RobotPose", Pose2d.struct).publish();
      
                // create pose estimator
        m_poseEstimator =
                new SwerveDrivePoseEstimator(
                   Constants.Swerve.swerveKinematics,
                   gyro.getRotation2d(),
                   getModulePositions(),
                   new Pose2d(),
                   VecBuilder.fill(0.1, 0.1, 0.1),
                   VecBuilder.fill(1.5, 1.5, 1.5)
                );

    }


    /*
     * This method will drive the swerve drive using translation and rotation vectors
     * 
     * Parameters:
     * translation X           (double)
     * translation Y           (double)
     * Rotation                (double)
     * is field relative       (boolean)
     * is open loop            (boolean)
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    } 
    
    /*
     * This method will get the modules states and convert
     * them into a ChassisSpeeds object
     * 
     * parameters:
     * none
     * 
     * returns:
     * swerve ChassiSpeeds object
     */
    public ChassisSpeeds getChassisSpeed() {

        return swerveKinematics.toChassisSpeeds(getModuleStates());

    }

    /*
     * This method will set the swerve modules to match a
     * ChassisSpeed parameter.
     * 
     * parameters:
     * ChassisSpeeds input         (ChassisSpeeds)
     * 
     * returns:
     * none
     */
    public void setChassisSpeed(ChassisSpeeds chassisSpeed) {

        SwerveModuleState[] desiredStates = swerveKinematics.toSwerveModuleStates(chassisSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        moduleStatePublisher.set(new double[] {
            desiredStates[0].angle.getDegrees(), desiredStates[0].speedMetersPerSecond,
            desiredStates[1].angle.getDegrees(), desiredStates[1].speedMetersPerSecond,
            desiredStates[2].angle.getDegrees(), desiredStates[2].speedMetersPerSecond,
            desiredStates[3].angle.getDegrees(), desiredStates[3].speedMetersPerSecond,
        });

        setModuleStates(desiredStates);
        
    }
    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
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

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }


    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){

        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
        } else {
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d(Math.PI)));
        }
        
    }

    public Rotation2d getGyroYaw(){
        return gyro.getRotation2d();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }




    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        SmartDashboard.putNumber("ChassisSpeedX", getChassisSpeed().vxMetersPerSecond);
        SmartDashboard.putNumber("ChassisSpeedY", getChassisSpeed().vyMetersPerSecond);
        SmartDashboard.putNumber("ChassisSpeedOmega", getChassisSpeed().omegaRadiansPerSecond);



        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("Robot X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Robot Y", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("gyro angle", getGyroYaw().getDegrees());



        posePublisher.set(getPose());
        swerveKinematicsPublisher.set(getModuleStates());
        estimatedRobotPosePublisher.set(m_poseEstimator.getEstimatedPosition());
    }
}