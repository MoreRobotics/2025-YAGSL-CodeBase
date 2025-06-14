package frc.robot;

import java.sql.Driver;
import java.time.Instant;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    double rotationSpeed = 1.0;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    /* Drive Controls */
    private final int leftY = XboxController.Axis.kLeftY.value;
    private final int leftX = XboxController.Axis.kLeftX.value;
    private final int rightX = XboxController.Axis.kRightX.value;
    /*Operator Buttons */
    private final JoystickButton operatorButtonY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton operatorButtonA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton operatorButtonB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton operatorButtonX = new JoystickButton(operator, XboxController.Button.kX.value);
    /* Driver Buttons */
    private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driverLB = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverRB = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton driverLStick = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton driverRStick = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final JoystickButton driverStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton driverSelect = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final POVButton driverDpadUp = new POVButton(driver, 0);
    private final POVButton driverDpadRight = new POVButton(driver, 90);
    private final POVButton driverDpadDown = new POVButton(driver, 180);
    private final POVButton driverDpadLeft = new POVButton(driver, 270);

    /* driver axis */
    private final int driverLeftTriggerAxis = XboxController.Axis.kLeftTrigger.value;
    private final int driverRightTriggerAxis = XboxController.Axis.kRightTrigger.value;
    
    /* driver triggers */
    final Trigger driverLeftTrigger = new Trigger(() -> driver.getRawAxis(driverLeftTriggerAxis) > 0.1);
    final Trigger driverRightTrigger = new Trigger(() -> driver.getRawAxis(driverRightTriggerAxis) > 0.1);
 
    /* Operator Buttons */
    private final JoystickButton operatorA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton operatorB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton operatorX = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton operatorY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton operatorLB = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton operatorRB = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton operatorLStick = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton operatorRStick = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    private final JoystickButton operatorUpStick = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton operatorDownStick = new JoystickButton(operator, XboxController.Button.kRightStick.value);

    private final JoystickButton operatorStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton operatorBack = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final POVButton operatorDpadUp = new POVButton(operator, 0);
    private final POVButton operatorDpadRight = new POVButton(operator, 90);
    private final POVButton operatorDpadDown = new POVButton(operator, 180);
    private final POVButton operatorDpadLeft = new POVButton(operator, 270);

    /* operator axis */
    private final int operatorLeftYAxis = XboxController.Axis.kLeftY.value;
    private final int operatorRightYAxis = XboxController.Axis.kRightY.value;
    private final int operatorLeftTriggerAxis = XboxController.Axis.kLeftTrigger.value;
    private final int operatorRightTriggerAxis = XboxController.Axis.kRightTrigger.value;

    /* operator triggers */
    final Trigger operatorLeftTrigger = new Trigger(() -> operator.getRawAxis(operatorLeftTriggerAxis) > 0.1);
    final Trigger operatorRightTrigger = new Trigger(() -> operator.getRawAxis(operatorRightTriggerAxis) > 0.1);

    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Eyes s_Eyes = new Eyes(s_Swerve);
    private final Mailbox s_Mailbox = new Mailbox();
    private final Climber s_Climber = new Climber();
    private final Elevator s_Elevator = new Elevator();
    private final Funnel s_Funnel = new Funnel(s_Climber); 
    private final AlgaePivot s_AlgaePivot = new AlgaePivot();
    private final AlgaeIntake s_AlgaeIntake = new AlgaeIntake();



    private final SendableChooser<Command> autoChooser;



    /** The container for the robot. Contains subsystems, OI devices, and commands. */

    public RobotContainer() {

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(leftY), 
                () -> -driver.getRawAxis(leftX), 
                () -> driver.getRawAxis(rightX),
                () -> driverDpadUp.getAsBoolean(),
                () -> s_Swerve.getGyroYaw().getDegrees(),
                () -> false,
                rotationSpeed,
                false
            )
        );
        
        } else {
        s_Swerve.setPose(new Pose2d(0, 0, new Rotation2d(Math.PI)));
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(leftY), 
                () -> driver.getRawAxis(leftX), 
                () -> driver.getRawAxis(rightX),
                () -> driverDpadUp.getAsBoolean(),
                () -> s_Swerve.getGyroYaw().getDegrees(),
                () -> false,
                rotationSpeed,
                false
            )
        );

        }

        NamedCommands.registerCommand("Outake", 
            new ConditionalCommand(
                new InstantCommand(),
                new OutakeCoral(s_Mailbox),
                () -> s_Mailbox.getSensorInput()
            ));
        NamedCommands.registerCommand("Intake", new IntakeCoral(s_Mailbox, s_Funnel).until(() -> s_Mailbox.getSensorInput() == false));
        NamedCommands.registerCommand("Elevator Lvl 1", new MoveElevator(s_Elevator, s_Elevator.lvl1Position).until(() -> s_Elevator.atPosition()));
        NamedCommands.registerCommand("Elevator Lvl 2", new MoveElevator(s_Elevator, s_Elevator.lvl2Position));
        NamedCommands.registerCommand("Elevator Lvl 3", new MoveElevator(s_Elevator, s_Elevator.lvl3Position));
        NamedCommands.registerCommand("Elevator Lvl 4",
         new ConditionalCommand(
            new InstantCommand(),
            new MoveElevator(s_Elevator, s_Elevator.lvl4Position).until(() -> s_Elevator.atPosition()),
            () -> s_Mailbox.getSensorInput()
        ));
        NamedCommands.registerCommand("Elevator Safe", new MoveElevator(s_Elevator, s_Elevator.restingposition));
        NamedCommands.registerCommand("Elevator Algae Lvl 2", new MoveElevator(s_Elevator,s_Elevator.algaeLvl2Position).until(() -> s_Elevator.atPosition()));
        NamedCommands.registerCommand("Elevator Algae Lvl 3", new MoveElevator(s_Elevator,s_Elevator.algaeLvl3Position).until(() -> s_Elevator.atPosition()));
        NamedCommands.registerCommand("Auto Align Right", new InstantCommand(() -> s_Swerve.followPathCommand(() -> s_Eyes.closestReefpath(-1))).until(() -> s_Eyes.closeToReef));
        NamedCommands.registerCommand("Algae Lvl 2", new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.reefLvl2).until(() -> s_AlgaePivot.atPosition()));
        NamedCommands.registerCommand("Algae Lvl 3", new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.reefLvl3));
        NamedCommands.registerCommand("Algae Safe", new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.safePose));
        NamedCommands.registerCommand("Algae Intake", new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIntakeSpeed)));
        NamedCommands.registerCommand("Algae Idle", new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIdleSpeed)));
        NamedCommands.registerCommand("Algae Outake", new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeOutakeSpeed)));


            NamedCommands.registerCommand("Auto Align Left", (new ConditionalCommand(new InstantCommand(() -> {
                s_Swerve.followPathCommand(() -> s_Eyes.closestReefpath(1)).schedule();
    
                }),

                new InstantCommand(),
    
                () -> !s_Eyes.closeToReef)
    
                ) );

        // Configure the button bindings
        configureButtonBindings();


        
        autoChooser = AutoBuilder.buildAutoChooser();

        
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putBoolean("Left Stick", driverLStick.getAsBoolean());
        SmartDashboard.putBoolean("Dpad Left", driverDpadLeft.getAsBoolean());
        
    }    
    


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // zero gyro
        driverSelect.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        //climb
        driverStart.onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> s_Funnel.setServo(s_Funnel.funnelDown)),
                new ClimberSafe(s_Climber),
                new InstantCommand(() -> s_Climber.changeTarget()),
                new Climb(s_Climber),
                // new InstantCommand(() -> s_Climber.setServo()),
                new InstantCommand(() -> s_Climber.checkClimb())
            )
        );

        // Outake
        driverRightTrigger.whileTrue(new OutakeCoral(s_Mailbox));
        // Intake
        driverLeftTrigger.whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Elevator.setElevatorPosition(0)),
                new IntakeCoral(s_Mailbox, s_Funnel)
                .until(() -> s_Mailbox.getSensorInput() == false))
            
        );
    // Auto Align Right
    driverRStick.whileTrue(
        new SequentialCommandGroup(
            new MoveElevator(s_Elevator, s_Elevator.algaeLvl2Position),
            new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.reefLvl2),
            new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIntakeSpeed))
        )
    ).onFalse(new SequentialCommandGroup(
        new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.safePose),   
        new MoveElevator(s_Elevator, s_Elevator.restingposition),//6.0
        new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIdleSpeed))
    )
    
);
    
    driverRB.whileTrue(new ParallelCommandGroup(new InstantCommand(() -> {
            s_Swerve.followPathCommand(() -> s_Eyes.closestRReefpath()).schedule();

        }),

        new MoveElevator(s_Elevator, s_Elevator.restingposition)

            //  .andThen(new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 1))) //TODO Test this, was only running on init earlier, may need to be run command
    )
    ).onFalse(s_Swerve.getDefaultCommand()); //TODO let driver know we are in position to trap via rumble


    // driverDpadLeft.onTrue(s_Swerve.pathfindiCommand);
    // Auto Align Left
    driverLStick.whileTrue(
        new SequentialCommandGroup(
            new MoveElevator(s_Elevator, s_Elevator.algaeLvl3Position),
            new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.reefLvl3),
            new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIntakeSpeed))
        )
    ).onFalse(new SequentialCommandGroup(
        new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.safePose),
        new MoveElevator(s_Elevator, s_Elevator.restingposition),
        new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIdleSpeed))

        
        )
    );
    
    driverLB.whileTrue(new ParallelCommandGroup(new InstantCommand(() -> {
        s_Swerve.followPathCommand(() -> s_Eyes.closestLReefpath()).schedule();

    }),

    new MoveElevator(s_Elevator, s_Elevator.restingposition)

        //  .andThen(new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 1))) //TODO Test this, was only running on init earlier, may need to be run command
    )
).onFalse( s_Swerve.getDefaultCommand()); 

        // Elevator
        driverA.whileTrue(
        new SequentialCommandGroup(
            new MoveElevator(s_Elevator,s_Elevator.algaeGroundPosition),
            new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.groundPose),
            new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIntakeSpeed))))
        .onFalse(new SequentialCommandGroup(
            new InstantCommand(() -> s_Elevator.setElevatorPosition(s_Elevator.restingposition)),
            new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.safePose)));

        driverB.whileTrue(
            new SequentialCommandGroup(
                new MoveElevator(s_Elevator, s_Elevator.lvl2Position),
                new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.processorPose),
                new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeOutakeSpeed))
            ))
        
        .onFalse(
            new SequentialCommandGroup(
                new MoveElevator(s_Elevator, s_Elevator.restingposition),
                new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.safePose),
                new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIdleSpeed))
                )
            );

        driverX.whileTrue(
            
            new MoveElevator(s_Elevator, s_Elevator.lvl3Position)
            
        )
            .onFalse(new SequentialCommandGroup(
                new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.safePose),   
                new MoveElevator(s_Elevator, s_Elevator.restingposition),//6.0
                new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIdleSpeed))
            )
            
        );
        
        driverY.whileTrue(

            new MoveElevator(s_Elevator, s_Elevator.lvl4Position)//49.45
        )
        .onFalse(new SequentialCommandGroup(
            new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.safePose),
            new MoveElevator(s_Elevator, s_Elevator.restingposition),
            new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIdleSpeed))

            
            )
        );

        driverDpadDown.whileTrue(new InstantCommand(() -> s_Elevator.elevatorDown()).until(() -> s_Elevator.getSensor() == false));
            

    //     driverX.and(driverLStick).whileTrue(
    //         new SequentialCommandGroup(
    //             new MoveElevator(s_Elevator, s_Elevator.algaeLvl2Position),
    //             new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.reefLvl2),
    //             new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIntakeSpeed))
    //         )
    //     ).onFalse(new SequentialCommandGroup(
    //         new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.safePose),   
    //         new MoveElevator(s_Elevator, s_Elevator.restingposition),//6.0
    //         new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIdleSpeed))
    //     )
        
    // );

        // driverY.and(driverLStick).whileTrue(
        //     new SequentialCommandGroup(
        //         new MoveElevator(s_Elevator, s_Elevator.algaeLvl3Position),
        //         new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.reefLvl3),
        //         new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIntakeSpeed))
        //     )
        // ).onFalse(new SequentialCommandGroup(
        //     new MoveAlgaePivot(s_AlgaePivot, s_AlgaePivot.safePose),
        //     new MoveElevator(s_Elevator, s_Elevator.restingposition),
        //     new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIdleSpeed))

            
        //     )
        // );

        // driverRStick.whileTrue(new ReverseOutakeCoral(s_Mailbox, s_Funnel));

        

       

        // Operator
        operatorA.onTrue(new InstantCommand(() -> s_AlgaePivot.moveAlgaePivot(s_AlgaePivot.groundPose)));
        operatorB.onTrue(new InstantCommand(() -> s_AlgaePivot.moveAlgaePivot(s_AlgaePivot.reefLvl2)));
        operatorX.onTrue(new InstantCommand(() -> s_AlgaePivot.moveAlgaePivot(s_AlgaePivot.reefLvl3)));
        operatorY.onTrue(new InstantCommand(() -> s_AlgaePivot.moveAlgaePivot(s_AlgaePivot.safePose)));

        operatorLeftTrigger.whileTrue(new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeIntakeSpeed)))
        .onFalse(new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(5)));

        operatorRightTrigger.whileTrue(new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(s_AlgaeIntake.algaeOutakeSpeed)))
        .onFalse(new InstantCommand(() -> s_AlgaeIntake.runAlgaeIntake(0)));

        // operatorStart.onTrue(new SequentialCommandGroup(
        //     new ClimberSafe(s_Climber),
        //     new Climb(s_Climber, s_Climber.climberReadyPose),
        //     new InstantCommand(() -> s_Funnel.setServo(s_Funnel.funnelDown))
        // ));

        operatorDpadDown.whileTrue(new ReverseOutakeCoral(s_Mailbox, s_Funnel));


    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return autoChooser.getSelected();
        
    }


}
