package frc.robot.subsystems;

//import static edu.wpi.first.units.Units.*;

//import java.util.function.BooleanSupplier;

//import edu.wpi.first.wpilibj.SynchronousInterrupt.WaitResult;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;


public class Mechanism extends SubsystemBase {
    private final TalonFX shooterMotorRight;
    private final TalonFX shooterMotorLeft;
    private final TalonFX shooterMotorRightHood;
    private final TalonFX shooterMotorLeftHood;

    private final TalonFX feederMotor;
    private final TalonFX kickerMotor;
    private final TalonFX sorterMotor;
    private final TalonFX floorRollerMotor;

    private final TalonFX intakeRollerMotor;

    private final TalonFX intakeRotation;

    private final TalonFX turretRotation;

    //private final PowerDistribution pdp;


    public int pointDirection;
    
    public Boolean ClimbLock = true;

    public Trigger coralWaitingIntake;
    public Trigger coralAtIntakeEntrance;
    public Trigger coralNotInIntake;

    public Translation2d translationToGoal;
    public double robotRotation;
    public double robotVelocityX;
    public double robotVelocityY;

    private boolean ShooterRun;

public Mechanism()
{
    shooterMotorRight = new TalonFX(18,"rio");
    shooterMotorLeft = new TalonFX(19,"rio");
    shooterMotorRightHood = new TalonFX(16,"rio");
    shooterMotorLeftHood = new TalonFX(20,"rio");

    feederMotor = new TalonFX(13,"rio");
    kickerMotor = new TalonFX(14,"rio");
    sorterMotor = new TalonFX(11,"rio");
    floorRollerMotor = new TalonFX(12,"rio");

    intakeRollerMotor = new TalonFX(9,"rio");

    intakeRotation = new TalonFX(10, "rio");

    turretRotation = new TalonFX(17, "rio");

    intakeRotation.setPosition(0.0);

    turretRotation.setPosition(0.0);

    translationToGoal = new Translation2d(0,0);
    robotRotation = 0;

    ShooterRun = false;

    //pdp = new PowerDistribution(0, ModuleType.kRev);

    // in init function
    var talonFXRotationConfigs = new TalonFXConfiguration();
    // set slot 0 gains
    var slot0Configs = talonFXRotationConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 1.75; // 4.8 A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.15; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicRotationConfigs = talonFXRotationConfigs.MotionMagic;
    motionMagicRotationConfigs.MotionMagicCruiseVelocity = 200; // Target cruise velocity of 80 rps
    motionMagicRotationConfigs.MotionMagicAcceleration = 80; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicRotationConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // in init function
    var talonFXSpeedConfigs = new TalonFXConfiguration();
    // set slot 0 gains
    var slot1Configs = talonFXSpeedConfigs.Slot0;
    slot1Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
    slot1Configs.kV = 0.001; // A velocity target of 1 rps results in 0.12 V output
    slot1Configs.kP = 0.05; // 4.8 A position error of 2.5 rotations results in 12 V output
    slot1Configs.kI = 0; // no output for integrated error
    slot1Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

            // set Motion Magic settings
    var motionMagicSpeedConfigs = talonFXSpeedConfigs.MotionMagic;
    motionMagicSpeedConfigs.MotionMagicCruiseVelocity = 200; // Target cruise velocity of 80 rps
    motionMagicSpeedConfigs.MotionMagicAcceleration = 80; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicSpeedConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    var talonFXHoodSpeedConfigs = new TalonFXConfiguration();
    // set slot 0 gains
    var slot2Configs = talonFXHoodSpeedConfigs.Slot0;
    slot2Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
    slot2Configs.kV = 0.009; // A velocity target of 1 rps results in 0.12 V output
    slot2Configs.kP = 0.03; // 4.8 A position error of 2.5 rotations results in 12 V output
    slot2Configs.kI = 0; // no output for integrated error
    slot2Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

            // set Motion Magic settings
    var motionMagicHoodSpeedConfigs = talonFXHoodSpeedConfigs.MotionMagic;
    motionMagicHoodSpeedConfigs.MotionMagicCruiseVelocity = 200; // Target cruise velocity of 80 rps
    motionMagicHoodSpeedConfigs.MotionMagicAcceleration = 80; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicHoodSpeedConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    
    intakeRotation.getConfigurator().apply(talonFXRotationConfigs);
    var intakeRotationRequest = new MotionMagicVoltage(0).withSlot(0);
    intakeRotation.setControl(intakeRotationRequest.withPosition(0.0));

    turretRotation.getConfigurator().apply(talonFXRotationConfigs);
    var turretRotationRequest = new MotionMagicVoltage(0).withSlot(0);
    turretRotation.setControl(turretRotationRequest.withPosition(0.0));

    shooterMotorLeft.getConfigurator().apply(talonFXSpeedConfigs);
    var shooterSpeedRequest = new MotionMagicVelocityDutyCycle(0).withSlot(0);
    shooterMotorLeft.setControl(shooterSpeedRequest.withVelocity(0));
    
    shooterMotorRight.getConfigurator().apply(talonFXSpeedConfigs);
    shooterMotorRight.setControl(shooterSpeedRequest.withVelocity(0));
    
    shooterMotorLeftHood.getConfigurator().apply(talonFXHoodSpeedConfigs);
    shooterMotorLeftHood.setControl(shooterSpeedRequest.withVelocity(0));
    
    shooterMotorRightHood.getConfigurator().apply(talonFXHoodSpeedConfigs);
    shooterMotorRightHood.setControl(shooterSpeedRequest.withVelocity(0));


    }

public Command intakeUp() {
    return runOnce(
    () -> {
        var intakeRotationRequest = new MotionMagicVoltage(0).withSlot(0);
        intakeRotation.setControl(intakeRotationRequest.withPosition(-5));
        //intakeRoller.set(intakeStowPower); //positive for coral // negitive for balls
    });
} 
public Command intakeDown() {
    return runOnce(
    () -> {
        var intakeRotationRequest = new MotionMagicVoltage(0).withSlot(0);
        intakeRotation.setControl(intakeRotationRequest.withPosition(-1.5));
        //intakeRoller.set(intakeStowPower); //positive for coral // negitive for balls
    });
} 

public Command turretLeft() {
    return runOnce(
    () -> {
        var turretRotationRequest = new MotionMagicVoltage(0).withSlot(0);
        turretRotation.setControl(turretRotationRequest.withPosition(-1.5));
        //intakeRoller.set(intakeStowPower); //positive for coral // negitive for balls
    });
} 

public Command turretRight() {
    return runOnce(
    () -> {
        var turretRotationRequest = new MotionMagicVoltage(0).withSlot(0);
        turretRotation.setControl(turretRotationRequest.withPosition(1.5));
        //intakeRoller.set(intakeStowPower); //positive for coral // negitive for balls
    });
} 

public Command turretCenter() {
    return runOnce(
    () -> {
        var turretRotationRequest = new MotionMagicVoltage(0).withSlot(0);
        turretRotation.setControl(turretRotationRequest.withPosition(0));
        //intakeRoller.set(intakeStowPower); //positive for coral // negitive for balls
    });
} 

public Command turretToAngleandShoot() {
    return runOnce(
    () -> {
        ShooterRun = true;
        //intakeRoller.set(intakeStowPower); //positive for coral // negitive for balls
    });
} 


public Command shooterRun(double speed) {
    return runOnce(
        () -> {

        if(speed > 0 && speed < 100)
        {
                var shooterSpeedRequest = new MotionMagicVelocityDutyCycle(0).withSlot(0);
                shooterMotorLeft.setControl(shooterSpeedRequest.withVelocity(speed));
                shooterMotorRight.setControl(shooterSpeedRequest.withVelocity(-speed));
                shooterMotorLeftHood.setControl(shooterSpeedRequest.withVelocity(-speed));
                shooterMotorRightHood.setControl(shooterSpeedRequest.withVelocity(-speed));
        }
    });
}


public Command shooterStop() {
    return runOnce(
        () -> {
            ShooterRun = false;
        shooterMotorRight.setVoltage(0.0);
        shooterMotorLeft.setVoltage(0.0);
        shooterMotorRightHood.setVoltage(0.0);
        shooterMotorLeftHood.setVoltage(0.0);
    });
}


public Command feedShooters() {
    return runOnce(
        () -> {
        feederMotor.setVoltage(-11.0);
        kickerMotor.setVoltage(10.0);
        sorterMotor.setVoltage(-8.0);
        floorRollerMotor.setVoltage(-6.0);
        //intakeRollerMotor.setVoltage(-4.0);
    });
}

public Command reverseFeeder() {
    return runOnce(
        () -> {
        feederMotor.setVoltage(2.0);
        kickerMotor.setVoltage(11.0);
        sorterMotor.setVoltage(4.0);
        floorRollerMotor.setVoltage(1.0);
    });
}

public Command feedStop() {
    return runOnce(
        () -> {
        feederMotor.setVoltage(0.0);
        kickerMotor.setVoltage(0.0);
        sorterMotor.setVoltage(0.0);
        floorRollerMotor.setVoltage(0.0);
        intakeRollerMotor.setVoltage(0.0);
    });
}

public Command intakeRun() {
    return runOnce(
        () -> {
        intakeRollerMotor.setVoltage(-5.0);
        floorRollerMotor.setVoltage(-6.0);
        feederMotor.setVoltage(2.0);
        sorterMotor.setVoltage(4.0);
    });
}

public Command intakeStop() {
    return runOnce(
        () -> {
        intakeRollerMotor.setVoltage(0.0);
        floorRollerMotor.setVoltage(0.0);
        feederMotor.setVoltage(0.0);
        sorterMotor.setVoltage(0.0);
    });
}

public Double getShooterSpeed()
{
    return shooterMotorLeft.getVelocity().getValueAsDouble();
}


@Override
public void periodic() {

    Rotation2d angleToGoal = new Rotation2d(translationToGoal.getX()*-1, translationToGoal.getY()*-1);
    double angle = angleToGoal.getDegrees() - robotRotation;
    SmartDashboard.putNumber("angle", angle);
    SmartDashboard.putNumber("translation X", translationToGoal.getX()*-1);
    SmartDashboard.putNumber("translation Y", translationToGoal.getY()*-1);
    SmartDashboard.putNumber("Pose Rot", robotRotation);
    SmartDashboard.putNumber("DistanceToGoal", translationToGoal.getNorm());
    SmartDashboard.putNumber("velocityX", robotVelocityX);
    SmartDashboard.putNumber("VelocityY", robotVelocityY);

    if(ShooterRun)
    {
        //Rotation2d angleToGoal = new Rotation2d(translationToGoal.getX()*-1, translationToGoal.getY()*-1);
        //double angle = angleToGoal.getDegrees() - robotRotation;
        if(angle < 85 && angle >-85)
        {
            var turretRotationRequest = new MotionMagicVoltage(0).withSlot(0);
            turretRotation.setControl(turretRotationRequest.withPosition(angle*0.0205));//0.0176
        }
        else
        {
            var turretRotationRequest = new MotionMagicVoltage(0).withSlot(0);
            turretRotation.setControl(turretRotationRequest.withPosition(0));
        }

       double distanceToGoal =  translationToGoal.getNorm();

        double speed = (0.162*(distanceToGoal*distanceToGoal)-(0.0145*distanceToGoal)+3.9357);


        //double speed = (5.2522*(distanceToGoal*distanceToGoal)) - (11.5*distanceToGoal) + 40.153;

        //double speed = (4.6568*(distanceToGoal*distanceToGoal*distanceToGoal)) - (29.068*(distanceToGoal*distanceToGoal)) + (66.758*distanceToGoal) - 10.3422;
        if(speed > 0 && speed < 100)
        {
            shooterMotorRight.setVoltage(-speed);
            shooterMotorLeft.setVoltage(speed);
            shooterMotorRightHood.setVoltage(-speed);
            shooterMotorLeftHood.setVoltage(-speed);
        }

    }
    else
    {
        //doNothing
    }

// This method will be called once per scheduler run
}

}
