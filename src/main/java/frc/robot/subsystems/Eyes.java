/*
 * This subsytem controls robot vision. It uses the LimelightHelpers class
 * and contains methods that are used to track april tags and gather
 * positional data from them. This data is then used in the swerve
 * subsystem to update the robot's odometry using the pose estimator.
 * 
 * parameters:
 * none
 */


package frc.robot.subsystems;


import frc.robot.Constants;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Swerve;



public class Eyes extends SubsystemBase {

    // Swerve subsystem for pose estimator
    Swerve s_Swerve;
    LimelightHelpers r_LimelightHelpers;

    // create objects and variables
    public LimelightHelpers limelight;
    public double tx;
    public double ty;
    public double ta;
    public double tID;
    public double txnc;
    public double tync;

   
    public boolean controllerRumble = false;
    public PathPlannerPath rReefPath;
    public PathPlannerPath lReefPath;
    // constuctor
    public Eyes(Swerve swerve) {

        s_Swerve = swerve;
        
    }

 
    /*
     * This method will gather all of the positional data of the limelight target.
     * 
     * parameters:
     * none
     * 
     * returns;
     * none
     */
    public void updateData() {

        /* 
         * get data from limelight target
         * tx = x position in degrees in limelight FOV
         * ty = y position in degrees in limelight FOV
         * ta = pitch in degrees in limelight FOV
         * tID = target ID number
         */
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        System.out.println(table.getEntry("tx").getDouble(0.22));
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        //read values periodically
        double x = tx.getDouble(tID);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        // tx = LimelightHelpers.getTX("limelight");
        // ty = LimelightHelpers.getTY("limelight");
        // ta = LimelightHelpers.getTA("limelight");
        // tID = LimelightHelpers.getFiducialID("limelight");

        // txnc = LimelightHelpers.getTXNC("limelight");  // Horizontal offset from principal pixel/point to target in degrees
        // tync = LimelightHelpers.getTYNC("limelight");  // Vertical  offset from principal pixel/point to target in degrees

        LimelightHelpers.setPipelineIndex("limelight", 0);

        // log target data
        SmartDashboard.putNumber("AprilTagID", tID);

    }

    /*
     * This method will wrap all target data into an array for easy access.
     * 
     * Array indexes:
     * [0] = x
     * [1] = y
     * [2] = a (pitch)
     * [3] = ID
     */
    public double[] getDataPackage() {

        double[] data = {
            tx,
            ty,
            ta,
            tID
        };

        return data;
    }

    /*
     * This method will return the pose of the robot based upon the pose of a detected apriltag
     * 
     * parameters:
     * none
     * 
     * returns:
     * robot pose      (Pose2d)
     */
    public Pose2d getRobotPose() {

        Pose2d pose;

        pose = LimelightHelpers.getBotPose2d("limelight");

        return pose;

    }
    

    /*
     * This method will return the known pose of the desired target.
     * 
     * parameters:
     * none
     * 
     * returns:
     * target pose      (Pose2d)
     */
    public Pose3d getTargetPose() {

        Pose3d pose;

        // if robot is on blue alliance
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {

            // get pose of blue speaker
            pose = new Pose3d(Constants.Positions.speakerBlueX, Constants.Positions.speakerBlueY, 0, new Rotation3d(0,0,Constants.Positions.speakerBlueR));

        // if robot is on red alliance
        } else {

            // get pose of red speaker
            pose = new Pose3d(Constants.Positions.speakerRedX, Constants.Positions.speakerRedY, 0, new Rotation3d(0,0,Constants.Positions.speakerRedR));

        }
        
        return pose;

    }

    public double getTargetRotation() {

        Pose2d robotPose = s_Swerve.m_poseEstimator.getEstimatedPosition();
        Pose3d targetPose = getTargetPose();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        double targetX = targetPose.getX();
        double targetY = targetPose.getY();

        double angle =  (Math.atan((targetY - robotY) / (targetX - robotX)) * (180 / Math.PI));

        if (robotX > targetX) {

            angle = angle + 180;

        }

        SmartDashboard.putNumber("angle", angle);
        SmartDashboard.putNumber(" inverted angle", -angle);

        return -angle + 180;
    }

    public boolean swerveAtPosition() {



        double error = Math.abs(getTargetRotation() + s_Swerve.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees() % 360);

        SmartDashboard.putNumber("getTargetRotation", getTargetRotation());
        SmartDashboard.putNumber("estimated rotation", s_Swerve.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees() % 360);
        SmartDashboard.putNumber("rotationError", error);

        if (error <= Constants.Swerve.atPositionTolerance) {
            return true;
        } else {
            return false;
        }
    }

    public double getDistanceFromTarget() {

        double distance;

        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {

            double xDistanceToSpeaker = Constants.Positions.speakerBlueX - s_Swerve.m_poseEstimator.getEstimatedPosition().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerBlueY - s_Swerve.m_poseEstimator.getEstimatedPosition().getY();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        } else {

            double xDistanceToSpeaker = Constants.Positions.speakerRedX - s_Swerve.m_poseEstimator.getEstimatedPosition().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerRedY - s_Swerve.m_poseEstimator.getEstimatedPosition().getY();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        }

        return distance;

    }

    public double getDistanceFromTargetBlind() {

        double distance;

        if(DriverStation.getAlliance().get() == Alliance.Blue) {

            double xDistanceToSpeaker = Constants.Positions.speakerBlueX - s_Swerve.getPose().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerBlueY - s_Swerve.getPose().getX();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        } else {

            double xDistanceToSpeaker = Constants.Positions.speakerRedX - s_Swerve.getPose().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerRedY - s_Swerve.getPose().getX();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        }

        return distance;

    }

    public double getDistanceFromTargetAuto() {

        double distance;

        if(DriverStation.getAlliance().get() == Alliance.Blue) {

            double xDistanceToSpeaker = Constants.Positions.speakerBlueX - s_Swerve.m_poseEstimator.getEstimatedPosition().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerBlueY - s_Swerve.m_poseEstimator.getEstimatedPosition().getY();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        } else {

            double xDistanceToSpeaker = Constants.Positions.speakerRedX - s_Swerve.m_poseEstimator.getEstimatedPosition().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerRedY - s_Swerve.m_poseEstimator.getEstimatedPosition().getY();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        }

        return distance;

    }

    /*public void updatePoseEstimatorWithVisionBotPose() {
        //invalid limelight
        if (LimelightHelpers.getTV("") == false) {
          return;
        }
        
        // distance from current pose to vision estimated pose
        double poseDifference = s_Swerve.m_poseEstimator.getEstimatedPosition().getTranslation()
            .getDistance(getRobotPose().getTranslation());
    
          double xyStds;
          double degStds;
          // multiple targets detected
          if (limelight.getNumberOfTargetsVisible() >= 2) {
            xyStds = 0.5;
            degStds = 6;
          }
          // 1 target with large area and close to estimated pose
          else if (LimelightHelpers.getTA() > 0.8 && poseDifference < 0.5) {
            xyStds = 1.0;
            degStds = 12;
          }
          // 1 target farther away and estimated pose is close
          else if (limelight.getBestTargetArea() > 0.1 && poseDifference < 0.3) {
            xyStds = 2.0;
            degStds = 30;
          }
          // conditions don't match to add a vision measurement
          else {
            return;
          }
    
          s_Swerve.m_poseEstimator.setVisionMeasurementStdDevs(
              VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
          s_Swerve.m_poseEstimator.addVisionMeasurement(getRobotPose(),
              Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("")/1000.0) - (LimelightHelpers.getLatency_Capture("")/1000.0));
        }
      }
/* */

    @Override
    public void periodic() {
        s_Swerve.m_poseEstimator.update(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions());

        if (LimelightHelpers.getTV("limelight") == true) {
            s_Swerve.m_poseEstimator.addVisionMeasurement(
                getRobotPose(), 
                Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("limelight")/1000.0) - (LimelightHelpers.getLatency_Capture("limelight")/1000.0)
            );
        }

        updateData();

        SmartDashboard.putNumber("Pose estimator rotations", s_Swerve.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("Pose Estimator X", s_Swerve.m_poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Pose Estimator Y", s_Swerve.m_poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("target X", getTargetPose().getX());
        SmartDashboard.putNumber("target Y", getTargetPose().getY());
        SmartDashboard.putNumber("Distance to Target", getDistanceFromTarget());

    }
}