// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.DoubleUnaryOperator;

import com.ctre.phoenix6.HootAutoReplay;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.MoreComplicatedVirtualTarget;
import frc.robot.subsystems.PolarVirtualTarget;
import frc.robot.subsystems.RadialTangential254;
import frc.robot.subsystems.RotatingKalmanNewtonSOTM;
import frc.robot.subsystems.TofFunctions;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final boolean kUseLimelight = true;

    private final RobotContainer m_robotContainer;
    
    private Translation2d redGoalLocation = new Translation2d(12.3,4.04);


    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("goalPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

   // RotatingKalmanNewtonSOTM solver = new RotatingKalmanNewtonSOTM(50.0, 12, 5);
    PolarVirtualTarget solver = new PolarVirtualTarget(10);

    private final double[] m_poseArray = new double[3];

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
           Pathfinding.setPathfinder(new LocalADStar());
        PathfindingCommand.warmupCommand().schedule();
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
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds, VecBuilder.fill(2.0, 2.0, Units.degreesToRadians(20)));
            }
            else if(llMeasurement.avgTagDist < 1.5)
            {
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds, VecBuilder.fill(3.0, 3.0, Units.degreesToRadians(60)));
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
        

        Rotation2d robotHeading = m_robotContainer.drivetrain.getState().Pose.getRotation();

        ChassisSpeeds fieldRelChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_robotContainer.drivetrain.getState().Speeds.vxMetersPerSecond, m_robotContainer.drivetrain.getState().Speeds.vyMetersPerSecond,m_robotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond, robotHeading);

        double heading = m_robotContainer.drivetrain.getPigeon2().getRotation2d().getRadians();           
        double cos = Math.cos(heading);                  
        double sin = Math.sin(heading); 

        double robotAx = m_robotContainer.drivetrain.getPigeon2().getAccelerationX().getValueAsDouble();
        double robotAy = m_robotContainer.drivetrain.getPigeon2().getAccelerationY().getValueAsDouble();

        double fieldAx = robotAx * cos - robotAy * sin;
        double fieldAy = robotAx * sin + robotAy * cos;

        double[] accel = new double[] { fieldAx*0.2, fieldAy*0.2 };

        double[] pos =  {m_robotContainer.drivetrain.getState().Pose.getX(), m_robotContainer.drivetrain.getState().Pose.getY()};
        double[] vel =  {fieldRelChassisSpeeds.vxMetersPerSecond, fieldRelChassisSpeeds.vyMetersPerSecond};
        //double[] accel = {0,0};

        double omega = fieldRelChassisSpeeds.omegaRadiansPerSecond;

         double[] target =  {12.0, 4.04};
        if(m_robotContainer.drivetrain.getState().Pose.getX() < 11 && m_robotContainer.drivetrain.getState().Pose.getY() > 4 )
        {
            target[0] =  14;
            target[1] =  5.5; //red feed
        }
        else if(m_robotContainer.drivetrain.getState().Pose.getX() < 11)
        {
            target[0] =  14;
            target[1] =  2.5; //red feed
        }
        else
        {
            target[0] = 12.0;
            target[1] = 4.04; //reg goal
        }
        
        
        double[] offset =  {0.17, 0};
        //var tof = TofFunctions.ballistic(20, 80.0);

        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);
        double[] fieldOffset = {
            cosH * 0.17,
            sinH * 0.17};

        DoubleUnaryOperator tof = TofFunctions.quadratic_regression(Map.ofEntries(
            Map.entry(2.12, 0.83),
            Map.entry(2.29, 0.91),
            Map.entry(2.5, 0.83),
            Map.entry(2.8, 0.9),
            Map.entry(3.0, 0.89),
            Map.entry(3.2, 0.97),
            Map.entry(3.25, 1.06),
            Map.entry(3.27, 1.16),
            Map.entry(3.77, 1.1),
            Map.entry(4.3, 1.23)
        ));

        solver.setSensitivity(0.8, 1.0, -1.0);
        double[] virtualTarget = solver.solve(pos, vel, accel, target, tof, 0.02, 1.38, omega, fieldOffset);
  
/* 
        solver.setSensitivity(1.0,0.0,0.0);
        solver.predict(0.02);                                
        solver.updateVelocity(vel, 0.01); 
        solver.updateAcceleration(accel, 0.2);

        solver.updatePosition(pos, 0.05);

        double[] virtualTarget = solver.solve(
            target,       
            tof,        
            0.02,         
            1.5,        
            omega,
            offset
        );
*/
        redGoalLocation = new Translation2d(virtualTarget[0],virtualTarget[1]);

        SmartDashboard.putNumber("virtualTarget[0]", virtualTarget[0]);
        SmartDashboard.putNumber("virtualTarget[1]", virtualTarget[1]);
        SmartDashboard.putNumber("tof", tof.applyAsDouble(2.0));

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");

        m_poseArray[0] = virtualTarget[0];
        m_poseArray[1] = virtualTarget[1];
        m_poseArray[2] = 0.0;
        fieldPub.set(m_poseArray);
        
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
