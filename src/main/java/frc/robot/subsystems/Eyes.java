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

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.commands.AlignToReefCommands;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
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

    public int closestReefSide;
    private StructPublisher<Pose2d> lReefPose = NetworkTableInstance.getDefault()
        .getStructTopic("Left Reef Pose", Pose2d.struct).publish();
    private StructPublisher <Pose2d> rReefPose = NetworkTableInstance.getDefault()
        .getStructTopic("Right Reef Pose", Pose2d.struct).publish();


   
    public boolean controllerRumble = false;
    public PathPlannerPath reefPath;
    public boolean closeToReef = false;
    // constuctor
    public Eyes(Swerve swerve) {

        s_Swerve = swerve;
        // reefPath = closestReefpath(-1);
        
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
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
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

        LimelightHelpers.setPipelineIndex("limelight-front", 0);

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

        
        pose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-front");
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

    public double getDistanceFromTargetReef(Pose2d reefPose) {
        double xDistance = reefPose.getX() - s_Swerve.m_poseEstimator.getEstimatedPosition().getX();
        double yDistance = reefPose.getY() - s_Swerve.m_poseEstimator.getEstimatedPosition().getY();
        double distance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

        return distance;
    }

    

    public int getClosestReefSide() {
        double reef0Distance;
        double reef1Distance;
        double reef2Distance;
        double reef3Distance;
        double reef4Distance;
        double reef5Distance;

        double closestDistance;

        closestReefSide = -1;

        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            reef0Distance = getDistanceFromTargetReef(Constants.AprilTagConstants.APRIL_TAGS.getTagPose(18).get().toPose2d());
            reef1Distance = getDistanceFromTargetReef(Constants.AprilTagConstants.APRIL_TAGS.getTagPose(19).get().toPose2d());
            reef2Distance = getDistanceFromTargetReef(Constants.AprilTagConstants.APRIL_TAGS.getTagPose(20).get().toPose2d());
            reef3Distance = getDistanceFromTargetReef(Constants.AprilTagConstants.APRIL_TAGS.getTagPose(21).get().toPose2d());
            reef4Distance = getDistanceFromTargetReef(Constants.AprilTagConstants.APRIL_TAGS.getTagPose(22).get().toPose2d());
            reef5Distance = getDistanceFromTargetReef(Constants.AprilTagConstants.APRIL_TAGS.getTagPose(17).get().toPose2d());

            closestDistance = Math.min(reef0Distance, Math.min(reef1Distance, Math.min(reef2Distance, Math.min(reef3Distance, Math.min(reef4Distance, reef5Distance)))));

            if (closestDistance == reef0Distance) {
                closestReefSide = 0;
            } else if (closestDistance == reef1Distance) {
                closestReefSide = 1;
            } else if (closestDistance == reef2Distance) {
                closestReefSide = 2;
            } else if (closestDistance == reef3Distance) {
                closestReefSide = 3;
            } else if (closestDistance == reef4Distance) {
                closestReefSide = 4;
            } else if (closestDistance == reef5Distance) {
                closestReefSide = 5;
            }
        } else {
            reef0Distance = getDistanceFromTargetReef(Constants.AprilTagConstants.APRIL_TAGS.getTagPose(7).get().toPose2d());
            reef1Distance = getDistanceFromTargetReef(Constants.AprilTagConstants.APRIL_TAGS.getTagPose(6).get().toPose2d());
            reef2Distance = getDistanceFromTargetReef(Constants.AprilTagConstants.APRIL_TAGS.getTagPose(11).get().toPose2d());
            reef3Distance = getDistanceFromTargetReef(Constants.AprilTagConstants.APRIL_TAGS.getTagPose(10).get().toPose2d());
            reef4Distance = getDistanceFromTargetReef(Constants.AprilTagConstants.APRIL_TAGS.getTagPose(9).get().toPose2d());
            reef5Distance = getDistanceFromTargetReef(Constants.AprilTagConstants.APRIL_TAGS.getTagPose(8).get().toPose2d());

            closestDistance = Math.min(reef0Distance, Math.min(reef1Distance, Math.min(reef2Distance, Math.min(reef3Distance, Math.min(reef4Distance, reef5Distance)))));

            if (closestDistance == reef0Distance) {
                closestReefSide = 0;
            } else if (closestDistance == reef1Distance) {
                closestReefSide = 1;
            } else if (closestDistance == reef2Distance) {
                closestReefSide = 2;
            } else if (closestDistance == reef3Distance) {
                closestReefSide = 3;
            } else if (closestDistance == reef4Distance) {
                closestReefSide = 4;
            } else if (closestDistance == reef5Distance) {
                closestReefSide = 5;
            }
        }

        if (closestDistance < Constants.Positions.distanceLimit) {
            closeToReef = true;
        } else {
            closeToReef = false;
        }
        return closestReefSide;
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

    

    public PathPlannerPath closestReefpath(int leftRight) {
        int closestSide = getClosestReefSide();
        Pose2d currentPose = s_Swerve.m_poseEstimator.getEstimatedPosition();
        Pose2d closestReef = AlignToReefCommands.getReefPose(closestSide, leftRight);

        List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(
            
            new Pose2d(currentPose.getX(), currentPose.getY(), closestReef.minus(currentPose).getRotation()),
            new Pose2d(closestReef.getX(), closestReef.getY(), closestReef.minus(currentPose).getRotation())
        );

        PathPlannerPath path = new PathPlannerPath(
            wayPoints,
             new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), 
            new IdealStartingState(0.0, currentPose.getRotation()),
             new GoalEndState(0.0, closestReef.getRotation())
            );

            // path.preventFlipping = true;

            return path;


    }

    public PathPlannerPath closestLReefpath() {
        int closestSide = getClosestReefSide();
        Pose2d currentPose = s_Swerve.m_poseEstimator.getEstimatedPosition();
        Pose2d closestReef = AlignToReefCommands.getReefPoseL(closestSide);

        List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(
            
            new Pose2d(currentPose.getX(), currentPose.getY(), closestReef.minus(currentPose).getRotation()),
            new Pose2d(closestReef.getX(), closestReef.getY(), closestReef.minus(currentPose).getRotation())
        );

        PathPlannerPath path = new PathPlannerPath(
            wayPoints,
             new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), 
            new IdealStartingState(0.0, currentPose.getRotation()),
             new GoalEndState(0.0, closestReef.getRotation())
            );

            // path.preventFlipping = true;

            return path;


    }

    public PathPlannerPath closestRReefpath() {
        int closestSide = getClosestReefSide();
        Pose2d currentPose = s_Swerve.m_poseEstimator.getEstimatedPosition();
        Pose2d closestReef = AlignToReefCommands.getReefPoseR(closestSide);

        List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(
            
            new Pose2d(currentPose.getX(), currentPose.getY(), closestReef.minus(currentPose).getRotation()),
            new Pose2d(closestReef.getX(), closestReef.getY(), closestReef.minus(currentPose).getRotation())
        );

        PathPlannerPath path = new PathPlannerPath(
            wayPoints,
             new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), 
            new IdealStartingState(0.0, currentPose.getRotation()),
             new GoalEndState(0.0, closestReef.getRotation())
            );

            // path.preventFlipping = true;

            return path;


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

    @Override
    public void periodic() {
        s_Swerve.m_poseEstimator.update(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions());

        if (LimelightHelpers.getTV("limelight-front") == true) {
            s_Swerve.m_poseEstimator.addVisionMeasurement(
                getRobotPose(), 
                Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("limelight-front")/1000.0) - (LimelightHelpers.getLatency_Capture("limelight")/1000.0)
            );
        }

        updateData();

        SmartDashboard.putNumber("Pose estimator rotations", s_Swerve.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("Pose Estimator X", s_Swerve.m_poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Pose Estimator Y", s_Swerve.m_poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("target X", getTargetPose().getX());
        SmartDashboard.putNumber("target Y", getTargetPose().getY());
        SmartDashboard.putNumber("Distance to Target", getDistanceFromTarget());
        SmartDashboard.putNumber("Closest Reef Side", getClosestReefSide());
        SmartDashboard.putNumber("Left Reef X", AlignToReefCommands.getReefPoseL(getClosestReefSide()).getX());
        SmartDashboard.putNumber("Left Reef Y", AlignToReefCommands.getReefPoseL(getClosestReefSide()).getY());
        SmartDashboard.putNumber("Right Reef X", AlignToReefCommands.getReefPoseR(getClosestReefSide()).getX());
        SmartDashboard.putNumber("Right Reef Y", AlignToReefCommands.getReefPoseR(getClosestReefSide()).getY());


         lReefPose.set(AlignToReefCommands.getReefPoseL(getClosestReefSide()));
         rReefPose.set(AlignToReefCommands.getReefPoseR(getClosestReefSide()));

    }
}