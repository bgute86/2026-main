// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Mechanism;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    PathPlannerPath testPath;

    public final Mechanism mechanism = new Mechanism();

    public final WaitCommand waitCommand1 = new WaitCommand(0.5);
    public final WaitCommand waitCommand2 = new WaitCommand(0.5);
    

    PathConstraints constraints = new PathConstraints(
    3.0, 3.0,
    Units.degreesToRadians(540), Units.degreesToRadians(720));

    public RobotContainer() {

        NamedCommands.registerCommand("IntakeRun",mechanism.intakeRun());
        NamedCommands.registerCommand("IntakeStop",mechanism.intakeStop());
        NamedCommands.registerCommand("IntakeDown",mechanism.intakeDown());
        NamedCommands.registerCommand("IntakeUp",mechanism.intakeUp());
        NamedCommands.registerCommand("FeederShooters",mechanism.feedShooters());
        NamedCommands.registerCommand("FeedStop",mechanism.feedStop());
        NamedCommands.registerCommand("shoot",mechanism.turretToAngleandShoot());
        NamedCommands.registerCommand("shooterStop",mechanism.shooterStop());
        NamedCommands.registerCommand("turretCenter",mechanism.turretCenter());
        

        autoChooser = AutoBuilder.buildAutoChooser("DoNothing");
        SmartDashboard.putData("Auto Mode", autoChooser);

        //drivetrain.configureFieldPositions();

        configureBindings();
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //joystick.a().onTrue(mechanism.shooterRun(70)).onFalse(mechanism.shooterStop());

        joystick.a().onTrue(mechanism.feedShooters()).onFalse(mechanism.feedStop());
        
        joystick.x().onTrue(mechanism.reverseFeeder()).onFalse(mechanism.feedStop());

        joystick.leftBumper().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed *.4) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed *.4) // Drive left with negative X (left)
                    .withRotationalRate((mechanism.angle/60) * MaxAngularRate *0.6))
                    .alongWith(new WaitUntilCommand(mechanism.pointedAtTarget()).andThen(mechanism.feedShooters())))
                    //.alongWith(mechanism.moveAndShootFeedStart()))// Drive counterclockwise with negative X (left)
                .onFalse (drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate))
                    .alongWith(mechanism.feedStop())
                    //.alongWith(mechanism.moveAndShootFeedStop()) // Drive counterclockwise with negative X (left)
                    ); // Drive counterclockwise with negative X (left)

        //joystick.y().onTrue(mechanism.shooterFeed()).onFalse(mechanism.shooterStop());

        //joystick.leftBumper().onTrue(mechanism.intakeRun()).onFalse(mechanism.intakeStop());
        joystick.rightBumper().onTrue(mechanism.intakeDown().andThen(mechanism.intakeRun())).onFalse(mechanism.intakeUp().andThen(mechanism.intakeStop()));

        //joystick.leftTrigger().onTrue(mechanism.turretLeft()).onFalse(mechanism.turretCenter());
        //joystick.rightTrigger().onTrue(mechanism.turretRight()).onFalse(mechanism.turretCenter());

       joystick.rightTrigger().whileTrue(mechanism.turretToAngleandShoot());
       joystick.leftTrigger().onTrue(mechanism.turretCenter().andThen(mechanism.shooterStop()));
       //joystick.leftTrigger().onTrue(mechanism.turretToGoal()).onFalse(mechanism.turretCenter());

        // Reset the field-centric heading on left bumper press.
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
        //return Commands.print("No autonomous command configured");
    }
}
