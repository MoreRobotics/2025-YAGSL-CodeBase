package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.ejml.simple.AutomaticSimpleMatrixConvert;
 import edu.wpi.first.units.Units;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final double ELEVATOR_SPROCKET_DIAMETER = 1.75;
    public static final double ELEVATOR_GEAR_RATIO = 5.0;
    public static final double ELEVATOR_CHAIN_RATIO = 2.5;
    public static final double ELEVATOR_ROTATIONS_TO_IN = (1.0 / ELEVATOR_GEAR_RATIO) * ELEVATOR_SPROCKET_DIAMETER * Math.PI / ELEVATOR_CHAIN_RATIO;
    public static final double ELEVATOR_TOLERANCE = 1.0;
    public static final double ELEVATOR_HIGH_LEVEL = 16.0;
    public static final double ELEVATOR_SAFE_LEVEL = 7.0;


    /* PID Rotation */ 
    public static final double ROTATE_KP = 0.02; //0.0222
    public static final double ROTATE_KI = 0.0;
    public static final double ROTATE_KD = 0.001;
    public static final double ROTATE_VELOCITY = 200.0;
    public static final double ROTATE_ACCELERATION = 400.0;



    /* slow mode */
    public static final double SLOW_MODE_PERCENT_TRANSLATION = 0.5;
    public static final double SLOW_MODE_PERCENT_STRAFE = 0.5;
    public static final double SLOW_MODE_PERCENT_ROTATION = 0.5;

    public static final double AUTO_ROTATE_DEADBAND = 1.0;

    public static final class Swerve {

        public static final double maxEncoderVoltage = 4.917;
        public static final int pigeonID = 8;
        public static final double gyroOffset = 0;

        public static final double rotateToAmpTargetAngle = 90;

        public static final double atPositionTolerance = 3.0;
        
        public static RobotConfig robotConfig;


        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2_16T);

        /* Drivetrain Constants */
        public static final double trackWidth = edu.wpi.first.math.util.Units.inchesToMeters(18.75); //TODO: This must be tuned to specific robot
        public static final double wheelBase = edu.wpi.first.math.util.Units.inchesToMeters(18.75); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.0;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.75; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;


        // output: m/s, measure: m
        // public static final ControlConstants SCORING_PID_X = new ControlConstants()
        //         .withPID(2, 0.2, 0.0).withTolerance(Inches.of(2).in(Meters));
        // public static final ControlConstants SCORING_PID_Y = new ControlConstants()
        //         .withPID(2, 0.2, 0.0).withTolerance(Units.inchesToMeters(2));

        // // output: deg/s, measure: deg
        // public static final ControlConstants SCORING_PID_ANGLE = new ControlConstants()
        //         .withPID(5, 0.4, 0.0).withTolerance(1);

       /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int cancoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(200.466); //
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, cancoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int cancoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(96.524); //203.08
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, cancoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int cancoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(21.986);//243.05
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, cancoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int cancoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(334.170); //183.44
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, cancoderID, angleOffset);
        }

    }


    public static final class AprilTagConstants {
        public static final AprilTagFieldLayout APRIL_TAGS = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);

    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
                public static final Distance BUMPER_THICKNESS = Units.Inches.of(3.75);//TODO: get bumper thickness


        public static final Distance DISTANCE_TO_REEF = Units.Inches.of(26.5 / 2).plus(BUMPER_THICKNESS);
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class Positions {

        public static final double speakerBlueX = 0;
        public static final double speakerBlueY = 5.5;
        public static final double speakerBlueR = 0;

        public static final double speakerRedX = 16.5;
        public static final double speakerRedY = 5.5;
        public static final double speakerRedR = 180;

        public static final double ampBlueX = 1.9;
        public static final double ampBlueY = 8.0;
        public static final double ampBlueR = 270;

        public static final double ampRedX = 16.45;
        public static final double ampRedY = 8.0;
        public static final double ampRedR = 270;

        public static final double distanceLimit = 0.1;

        public static final double blue1ReefX = 0.0;
        public static final double blue1ReefY = 0.0;
        public static final double blue1ReefR = 0.0;

        public static final double blue2ReefX = 0.0;
        public static final double blue2ReefY = 0.0;
        public static final double blue2ReefR = 0.0;

        public static final double blue3ReefX = 0.0;
        public static final double blue3ReefY = 0.0;
        public static final double blue3ReefR = 0.0;

        public static final double blue4ReefX = 0.0;
        public static final double blue4ReefY = 0.0;
        public static final double blue4ReefR = 0.0;

        public static final double blue5ReefX = 0.0;
        public static final double blue5ReefY = 0.0;
        public static final double blue5ReefR = 0.0;

        public static final double blue6ReefX = 0.0;
        public static final double blue6ReefY = 0.0;
        public static final double blue6ReefR = 0.0;

        public static final double red1ReefX = 0.0;
        public static final double red1ReefY = 0.0;
        public static final double red1ReefR = 0.0;

        public static final double red2ReefX = 0.0;
        public static final double red2ReefY = 0.0;
        public static final double red2ReefR = 0.0;

        public static final double red3ReefX = 0.0;
        public static final double red3ReefY = 0.0;
        public static final double red3ReefR = 0.0;

        public static final double red4ReefX = 0.0;
        public static final double red4ReefY = 0.0;
        public static final double red4ReefR = 0.0;

        public static final double red5ReefX = 0.0;
        public static final double red5ReefY = 0.0;
        public static final double red5ReefR = 0.0;

        public static final double red6ReefX = 0.0;
        public static final double red6ReefY = 0.0;
        public static final double red6ReefR = 0.0;

        
        

    }
}