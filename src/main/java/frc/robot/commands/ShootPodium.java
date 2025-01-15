
package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Eyes;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class ShootPodium extends Command {

    // required subystems
    private ShooterPivot shooterPivot;  
    private Shooter shooter;
    private Elevator elevator;

    private double distance;

    // required WPILib class objects

    // local variables
    private double shooterAngle;
    private double leftShooterSpeed;
    private double rightShooterSpeed;

    // positions 

    //Higher note shot is lower angle!!!

    private final double podiumAngle = 130;
    private final double podiumLeftShooterSpeed = 90.0;
    private final double podiumRightShooterSpeed = podiumLeftShooterSpeed;


    // constructor
    public ShootPodium(ShooterPivot shooterPivot, Shooter shooter) {
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;

        addRequirements(shooterPivot, shooter);

        // instantiate objects
        
    }

    @Override
    public void execute() {

        // move shooter to calculated angle
        shooterPivot.moveShooterPivot(podiumAngle);

        // set shooter speed power to calculated value
        shooter.shootingMotorsSetControl(podiumRightShooterSpeed, podiumLeftShooterSpeed);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setLoaderVoltage(shooter.stopLoaderVoltage);
        shooter.setShooterVoltage(shooter.stopShooterVoltage, shooter.stopShooterVoltage);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; 
    }
}