// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.BasicVirtualTarget;
import frc.robot.subsystems.TofFunctions;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final boolean kUseLimelight = true;

    private final RobotContainer m_robotContainer;
    
    private Translation2d redGoalLocation = new Translation2d(12.3,4.04);

    private BasicVirtualTarget solver = new BasicVirtualTarget(10);



    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 

        if (kUseLimelight) {
        var driveState = m_robotContainer.drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
        
        LimelightHelpers.SetRobotOrientation("limelight-swamp", headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-swamp");
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
            if(llMeasurement.tagCount > 1)
            {
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds, VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(20)));
            }
            else if(llMeasurement.avgTagDist < 1.5)
            {
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds, VecBuilder.fill(2.0, 2.0, Units.degreesToRadians(60)));
            }
            else
            {
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds, VecBuilder.fill(15.0, 15.0, Units.degreesToRadians(180)));
            }
        }


    }

     m_robotContainer.mechanism.translationToGoal = redGoalLocation.minus(m_robotContainer.drivetrain.getState().Pose.getTranslation());
     m_robotContainer.mechanism.robotRotation = m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
     m_robotContainer.mechanism.robotVelocityX= m_robotContainer.drivetrain.getState().Speeds.vxMetersPerSecond;
     m_robotContainer.mechanism.robotVelocityY= m_robotContainer.drivetrain.getState().Speeds.vyMetersPerSecond;

     //redGoalLocation = new Translation2d(12.3-(1.2*m_robotContainer.drivetrain.getState().Speeds.vxMetersPerSecond), 4.04-(1.2*m_robotContainer.drivetrain.getState().Speeds.vyMetersPerSecond));

     
        double[] pos =  {m_robotContainer.drivetrain.getState().Pose.getX(), m_robotContainer.drivetrain.getState().Pose.getY()};
        double[] vel =  {m_robotContainer.drivetrain.getState().Speeds.vxMetersPerSecond, m_robotContainer.drivetrain.getState().Speeds.vyMetersPerSecond};
        double[] target =  {12.3, 4.04};
        var tof = TofFunctions.ballistic(m_robotContainer.mechanism.getShooterSpeed()*0.25, 15.0);

        double[] virtualTarget = solver.solve(pos, vel, target, tof, 0.02);
        
        redGoalLocation = new Translation2d(virtualTarget[0],virtualTarget[1]);
        
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
