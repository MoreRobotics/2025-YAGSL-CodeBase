// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;

/** Add your docs here. */
public class AlignToReefCommands {
    // Pose at midpoint between tags 18 and 21 (which are opposite on blue reef)
    private static final Translation2d REEF_CENTER_BLUE = Constants.AprilTagConstants.APRIL_TAGS.getTagPose(18).get().toPose2d().getTranslation()
        .plus(Constants.AprilTagConstants.APRIL_TAGS.getTagPose(21).get().toPose2d().getTranslation()).div(2);

    // Pose at midpoint between tags 10 and 7 (which are opposite on red reef)
    private static final Translation2d REEF_CENTER_RED = Constants.AprilTagConstants.APRIL_TAGS.getTagPose(10).get().toPose2d().getTranslation()
        .plus(Constants.AprilTagConstants.APRIL_TAGS.getTagPose(7).get().toPose2d().getTranslation()).div(2);

    private static boolean flipToRed; // whether to use red reef (otherwise blue)

    // Distance from center of robot to center of reef
    // Found by taking distance from tag 18 to center and adding offset from reef
    private static final Distance REEF_APOTHEM = Units.Meters.of(
            Constants.AprilTagConstants.APRIL_TAGS.getTagPose(18).get().toPose2d().getTranslation().getDistance(REEF_CENTER_BLUE))
            .plus(Constants.AutoConstants.DISTANCE_TO_REEF);//.plus(Distance.ofBaseUnits(2, Meter));

    private static final double mailboxOffset = 2.0;
    // translation to move from centered on a side to scoring position for the left branch
    private static final Translation2d CENTERED_TO_LEFT_BRANCH = new Translation2d(Units.Meters.of(0),
            Units.Inches.of(13.94 / 2));
    private static final Translation2d BRANCH_MAILBOX_OFFSEST = new Translation2d(Units.Meters.of(0),
        Units.Inches.of(mailboxOffset));

    /**
     * Calculates the pose of the robot for scoring on a branch or trough.
     *
     * @param side The side of the reef (0 for left, increases clockwise).
     * @param relativePos The relative position on the reef (-1 for right branch, 0 for center, 1 for left branch).
     * @return The calculated Pose2d for scoring.
     */
    public static Pose2d getReefPose(int side, int relativePos) {
        // determine whether to use red or blue reef position
        boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

        return getReefPose(side, relativePos, isRed);
    }

    /**
     * Calculates the pose of the robot for scoring on a branch or trough.
     *
     * @param side The side of the reef (0 for left, increases clockwise).
     * @param relativePos The relative position on the reef (-1 for right branch, 0 for center, 1 for left branch).
     * @return The calculated Pose2d for scoring.
     */
    public static Pose2d getReefPose(int side, int relativePos, boolean isRed) {
        // determine whether to use red or blue reef position
        flipToRed = isRed;

        // initially do all calculations from blue, then flip later
        Translation2d reefCenter = REEF_CENTER_BLUE;

        // robot position centered on close reef side
        Translation2d translation = reefCenter.plus(new Translation2d(REEF_APOTHEM.unaryMinus(), Units.Meters.zero()));
        // translate to correct branch (left, right, center)
        translation = translation.plus(CENTERED_TO_LEFT_BRANCH.times(relativePos)).minus(BRANCH_MAILBOX_OFFSEST);
        // rotate to correct side
        translation = translation.rotateAround(reefCenter, Rotation2d.fromDegrees(-60 * side));

        // make pose from translation and correct rotation
        Pose2d reefPose = new Pose2d(translation,
                Rotation2d.fromDegrees(-60 * side));

        if (flipToRed) {
            reefPose = flipPose(reefPose);
        }

        return reefPose;
    }

    public static Pose2d getReefPoseL(int side) {
        Pose2d reefPose2d = new Pose2d();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            if (side == 1) {
                reefPose2d = new Pose2d(Constants.Positions.lblue1ReefX, Constants.Positions.lblue1ReefY, Rotation2d.fromDegrees(Constants.Positions.lblue1ReefR));
            } else if (side == 2) {
                reefPose2d = new Pose2d(Constants.Positions.lblue2ReefX, Constants.Positions.lblue2ReefY, Rotation2d.fromDegrees(Constants.Positions.lblue2ReefR));
            } else if (side == 3) {
                reefPose2d = new Pose2d(Constants.Positions.lblue3ReefX, Constants.Positions.lblue3ReefY, Rotation2d.fromDegrees(Constants.Positions.lblue3ReefR));
            } else if (side == 4) {
                reefPose2d = new Pose2d(Constants.Positions.lblue4ReefX, Constants.Positions.lblue4ReefY, Rotation2d.fromDegrees(Constants.Positions.lblue4ReefR));
            } else if (side == 5) {
                reefPose2d = new Pose2d(Constants.Positions.lblue5ReefX, Constants.Positions.lblue5ReefY, Rotation2d.fromDegrees(Constants.Positions.lblue5ReefR));
            } else if (side == 0){
                reefPose2d = new Pose2d(Constants.Positions.lblue0ReefX, Constants.Positions.lblue0ReefY, Rotation2d.fromDegrees(Constants.Positions.lblue0ReefR));
            }
            return reefPose2d;
        } else {
            if (side == 1) {
                reefPose2d = new Pose2d(Constants.Positions.lred1ReefX, Constants.Positions.lred1ReefY, Rotation2d.fromDegrees(Constants.Positions.lred1ReefR));
            } else if (side == 2) {
                reefPose2d = new Pose2d(Constants.Positions.lred2ReefX, Constants.Positions.lred2ReefY, Rotation2d.fromDegrees(Constants.Positions.lred2ReefR));
            } else if (side == 3) {
                reefPose2d = new Pose2d(Constants.Positions.lred3ReefX, Constants.Positions.lred3ReefY, Rotation2d.fromDegrees(Constants.Positions.lred3ReefR));
            } else if (side == 4) {
                reefPose2d = new Pose2d(Constants.Positions.lred4ReefX, Constants.Positions.lred4ReefY, Rotation2d.fromDegrees(Constants.Positions.lred4ReefR));
            } else if (side == 5) {
                reefPose2d = new Pose2d(Constants.Positions.lred5ReefX, Constants.Positions.lred5ReefY, Rotation2d.fromDegrees(Constants.Positions.lred5ReefR));
            } else if (side == 0){
                reefPose2d = new Pose2d(Constants.Positions.lred0ReefX, Constants.Positions.lred0ReefY, Rotation2d.fromDegrees(Constants.Positions.lred0ReefR));
            }
            return reefPose2d;
        }
    }

    public static Pose2d getReefPoseR(int side) {
        Pose2d reefPose2d = new Pose2d();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            if (side == 1) {
                reefPose2d = new Pose2d(Constants.Positions.rblue1ReefX, Constants.Positions.rblue1ReefY, Rotation2d.fromDegrees(Constants.Positions.lblue1ReefR));
            } else if (side == 2) {
                reefPose2d = new Pose2d(Constants.Positions.rblue2ReefX, Constants.Positions.rblue2ReefY, Rotation2d.fromDegrees(Constants.Positions.lblue2ReefR));
            } else if (side == 3) {
                reefPose2d = new Pose2d(Constants.Positions.rblue3ReefX, Constants.Positions.rblue3ReefY, Rotation2d.fromDegrees(Constants.Positions.lblue3ReefR));
            } else if (side == 4) {
                reefPose2d = new Pose2d(Constants.Positions.rblue4ReefX, Constants.Positions.rblue4ReefY, Rotation2d.fromDegrees(Constants.Positions.lblue4ReefR));
            } else if (side == 5) {
                reefPose2d = new Pose2d(Constants.Positions.rblue5ReefX, Constants.Positions.rblue5ReefY, Rotation2d.fromDegrees(Constants.Positions.lblue5ReefR));
            } else if (side == 0){
                reefPose2d = new Pose2d(Constants.Positions.rblue0ReefX, Constants.Positions.rblue0ReefY, Rotation2d.fromDegrees(Constants.Positions.lblue0ReefR));
            }
            return reefPose2d;
        } else { 
            if (side == 1) {
                reefPose2d = new Pose2d(Constants.Positions.rRed1ReefX, Constants.Positions.rRed1ReefY, Rotation2d.fromDegrees(Constants.Positions.rRed1ReefR));
            } else if (side == 2) {
                reefPose2d = new Pose2d(Constants.Positions.rRed2ReefX, Constants.Positions.rRed2ReefY, Rotation2d.fromDegrees(Constants.Positions.rRed2ReefR));
            } else if (side == 3) {
                reefPose2d = new Pose2d(Constants.Positions.rRed3ReefX, Constants.Positions.rRed3ReefY, Rotation2d.fromDegrees(Constants.Positions.rRed3ReefR));
            } else if (side == 4) {
                reefPose2d = new Pose2d(Constants.Positions.rRed4ReefX, Constants.Positions.rRed4ReefY, Rotation2d.fromDegrees(Constants.Positions.rRed4ReefR));
            } else if (side == 5) {
                reefPose2d = new Pose2d(Constants.Positions.rRed5ReefX, Constants.Positions.rRed5ReefY, Rotation2d.fromDegrees(Constants.Positions.rRed5ReefR));
            } else if (side == 0){
                reefPose2d = new Pose2d(Constants.Positions.rRed0ReefX, Constants.Positions.rRed0ReefY, Rotation2d.fromDegrees(Constants.Positions.rRed0ReefR));
            }
            return reefPose2d;
        }
    }



    private static Pose2d flipPose(Pose2d pose) {
        Translation2d center = REEF_CENTER_BLUE.interpolate(REEF_CENTER_RED, 0.5);
        Translation2d poseTranslation = pose.getTranslation();
        poseTranslation = poseTranslation.rotateAround(center, Rotation2d.k180deg);
        return new Pose2d(poseTranslation, pose.getRotation().rotateBy(Rotation2d.k180deg));
    }

    // public static AlignToPoseCommand alignToReef(int side, int relativePos, Swerve swerve) {
    //     return new AlignToPoseCommand(getReefPose(side, relativePos), SCORING_PID_X, SCORING_PID_Y, SCORING_PID_ANGLE, swerve);
    // }

    public static void testReefPoses() {
        testReefPoses(false, -1);
        testReefPoses(false, 0);
        testReefPoses(false, 1);
        testReefPoses(true, -1);
        testReefPoses(true, 0);
        testReefPoses(true, 1);
    }

    public static void testReefPoses(boolean isRed, int relativePos) {
        String topicName = "Reef Alignment Poses/";
        topicName += isRed ? "Red " : "Blue ";
        if (relativePos == -1) {
            topicName += "Right ";
        }
        else if (relativePos == 1) {
            topicName += "Left ";
        }
        else {
            topicName += "Center ";
        }
        
        StructArrayPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructArrayTopic(topicName, Pose2d.struct).publish();
        
        Pose2d[] poses = new Pose2d[6];
        for (int side = 0; side < 6; side++) {
            poses[side] = getReefPose(side, relativePos, isRed);
        }

        publisher.set(poses);
    }
}
