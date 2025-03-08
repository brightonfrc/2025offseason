// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.AutonomousNavConstants;
import frc.robot.Constants.ChoreoConstants;
import frc.robot.Constants.CoralStationAlignConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WinchConstants;
import frc.robot.Constants.LiftConstants.Height;
import frc.robot.Constants.AprilTagAlignmentConstants;
import frc.robot.commands.AprilTagAlignment;
import frc.robot.commands.Autos;
import frc.robot.commands.CoralStationAlign;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Lift;
import frc.robot.commands.RunLift;
import frc.robot.commands.SustainLift;
import frc.robot.commands.RunArm;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeTimeLimited;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DatisLift;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.MoveToPoint;
import frc.robot.commands.StopRobot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.HangRobot;
import frc.robot.commands.JankLift;
import frc.robot.commands.JankLiftAutonomous;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Winch;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.AprilTagPoseEstimator;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DatisLift lift= new DatisLift(new SparkMax(LiftConstants.liftNeoCANID, MotorType.kBrushless), new SparkMax(LiftConstants.reversedLiftNeoCANDID, MotorType.kBrushless), new DutyCycleEncoder(LiftConstants.encoderChannel));
  private final Arm arm = new Arm(new DutyCycleEncoder(ArmConstants.armEncoderPort), new SparkMax(ArmConstants.armCANID, MotorType.kBrushless));
  private final Intake intake = new Intake(new SparkMax(IntakeConstants.intakeCanID, MotorType.kBrushless));
  private final DriveSubsystem m_driveSubsystem= new DriveSubsystem();
  private final AprilTagPoseEstimator m_poseEstimator = new AprilTagPoseEstimator();
  private final Winch winch = new Winch(new SparkMax(WinchConstants.winchCANID, MotorType.kBrushless));

  private boolean active = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
  new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandPS4Controller m_manualLiftController= new CommandPS4Controller(OIConstants.kManualLiftControllerPort);
  
  private final FieldOrientedDrive fieldOrientedDrive= new FieldOrientedDrive(m_driveSubsystem, m_driverController);
  // private Lift goToGround=new Lift(lift, Height.Ground);
  private final SustainLift sustainLift = new SustainLift(lift, arm);

  //for Choreo
  private final AutoFactory autoFactory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_driveSubsystem.setDefaultCommand(fieldOrientedDrive);
    lift.setDefaultCommand(sustainLift);
    // m_driveSubsystem.drive(m_driverController.getLeftX(), m_driverController.getLeftY(), 0, false);
    //for Choreo
    autoFactory = new AutoFactory(
            m_driveSubsystem::getPoseChoreo, // A function that returns the current robot pose
            m_driveSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            m_driveSubsystem::followTrajectory, // The drive subsystem trajectory follower 
            false, // If alliance flipping should be enabled 
            m_driveSubsystem // The drive subsystem
        );
    configureBindings();
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

   //public static boolean aliningWithCoral = false;

   public static boolean fieldRelative = true;

  private void configureBindings() {
    //R2 and L2 don't work

    // m_manualLiftController.R1().whileTrue(new RunLift(lift, true));
    // m_manualLiftController.L1().whileTrue(new RunLift(lift, false));

    m_manualLiftController.R1().whileTrue(new RunArm(arm, true, lift));
    m_manualLiftController.L1().whileTrue(new RunArm(arm, false, lift));


    //bind intake and outtake to L3 and R3?

    
    //remember that A on the xbox controllerbutton is bound to "slow Mode"
    
    m_driverController.y().onTrue(new JankLift(lift, arm, Height.CoralStation));
    m_driverController.povUp().onTrue(new JankLift(lift, arm, Height.L1));
    m_driverController.povRight().onTrue(new JankLift(lift, arm, Height.L2));
    m_driverController.povDown().onTrue(new JankLift(lift, arm, Height.L3));
    m_driverController.povLeft().onTrue(new JankLift(lift, arm, Height.L4));
    m_driverController.x().onTrue(new JankLift(lift, arm, Height.StartingConfig));
    
    m_manualLiftController.povUp().onTrue(new JankLift(lift, arm, Height.Algae2));
    m_manualLiftController.povRight().onTrue(new JankLift(lift, arm, Height.Algae3));
    m_manualLiftController.povDown().onTrue(new JankLift(lift, arm, Height.Ground));
    m_manualLiftController.povLeft().onTrue(new JankLift(lift, arm, Height.HangStart));
    m_manualLiftController.cross().onTrue(new HangRobot(lift, winch));
    m_manualLiftController.triangle().onTrue(new JankLift(lift, arm, Height.InternalStow));
    // m_driverController.rightBumper().whileTrue(new RunArm(arm, true, lift));
    // m_driverController.leftBumper().whileTrue(new RunArm(arm, false, lift));

    m_driverController.rightTrigger().whileTrue(new RunIntake(intake, true, lift, arm));
    m_driverController.leftTrigger().whileTrue(new RunIntake(intake, false, lift, arm));
    
    //m_manualLiftController.triangle().onTrue(new CoralStationAlign(m_driveSubsystem, m_driverController, m_poseEstimator));
    // m_manualLiftController.circle().onTrue(new MoveToPointTeleop(m_driveSubsystem, Math.toRadians(225))); // Right
    // m_manualLiftController.square().onTrue(new MoveToPointTeleop(m_driveSubsystem, Math.toRadians(135))); // Left
    //m_manualLiftController.triangle().onTrue();
    //m_manualLiftController.triangle().onTrue()
    //m_manualLiftController.triangle().onTrue(fieldOrientedDrive)


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    //align right
    m_driverController.leftBumper().onTrue(new AprilTagAlignment(m_driveSubsystem, m_poseEstimator, AprilTagAlignmentConstants.stopDisplacementX, AprilTagAlignmentConstants.stopDisplacementY+AprilTagAlignmentConstants.cameraDisplacement));
    //align left
    m_driverController.rightBumper().onTrue(new AprilTagAlignment(m_driveSubsystem, m_poseEstimator, AprilTagAlignmentConstants.stopDisplacementX, -AprilTagAlignmentConstants.stopDisplacementY+AprilTagAlignmentConstants.cameraDisplacement));


    //m_manualLiftController.square().onTrue(); // Reset gyro whenever necessary
    m_manualLiftController.square().onTrue(
      new InstantCommand(() -> m_driveSubsystem.resetGyro(), m_driveSubsystem)
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    StopRobot stop = new StopRobot(m_driveSubsystem);
    //annoyingly, you can't reuse the same command in a Commands.sequence
    StopRobot stop2 = new StopRobot(m_driveSubsystem);
    StopRobot stop3 = new StopRobot(m_driveSubsystem);
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    // m_driveSubsystem.resetOdometry(new Pose2d(ChoreoConstants.startX,ChoreoConstants.startY,new Rotation2d(ChoreoConstants.startRadians)));
    
    //return Commands.sequence(stop);

    // YOU MUST CHANGE THE STARTPOS CONSTANT IN THE CONSTANTS BASED ON THE FIELD!!!!!!!
    // TODO : Properly discuss with Datis and then setup the commands sequence as such
    
    Height scoreHeight = AutonomousNavConstants.scoreHeight;//Height.L4; // Datis wants L4

    switch (AutonomousNavConstants.startPos){
      case Left:
        return Commands.sequence(
          autoFactory.resetOdometry("Left"),
          autoFactory.trajectoryCmd("Left"),
          stop,
          new MoveToPoint(m_driveSubsystem, Math.toRadians(AutonomousNavConstants.endRotOne)),
          new JankLiftAutonomous(lift, arm, scoreHeight),
          new RunIntakeTimeLimited(intake, false /* We want it to outtake, so in should be false */, lift, arm, 2),
          new JankLiftAutonomous(lift, arm, Height.CoralStation),
          new MoveToPoint(m_driveSubsystem, Math.toRadians(0)),
          autoFactory.resetOdometry("LeftToStation"),
          autoFactory.trajectoryCmd("LeftToStation"),
          stop2,
          // Await for coral deposit...?
          new MoveToPoint(m_driveSubsystem, Math.toRadians(CoralStationAlignConstants.leftCoralStationRot)),
          new RunIntakeTimeLimited(intake, true /* We want it to intake, so in should be true */, lift, arm, 4),
          new MoveToPoint(m_driveSubsystem, Math.toRadians(0)),
          autoFactory.resetOdometry("LeftFromStation"),
          autoFactory.trajectoryCmd("LeftFromStation"),
          stop3,
          //new MoveToPoint(m_driveSubsystem, Math.toRadians(AutonomousNavConstants.endRotTwo)),
          new MoveToPoint(m_driveSubsystem, Math.toRadians(AutonomousNavConstants.endRotOne)),
          new JankLiftAutonomous(lift, arm, scoreHeight),
          new RunIntakeTimeLimited(intake, false /* We want it to outtake, so in should be false */, lift, arm, 2),
          new JankLiftAutonomous(lift, arm, Height.CoralStation),
          new MoveToPoint(m_driveSubsystem, Math.toRadians(0))
        );
      case Right:
        return Commands.sequence(
          autoFactory.resetOdometry("Right"),
          autoFactory.trajectoryCmd("Right"),
          stop,
          new MoveToPoint(m_driveSubsystem, Math.toRadians(AutonomousNavConstants.endRotOne)),
          new JankLiftAutonomous(lift, arm, scoreHeight),
          new RunIntakeTimeLimited(intake, false /* We want it to outtake, so in should be false */, lift, arm, 2),
          new JankLiftAutonomous(lift, arm, Height.CoralStation),
          new MoveToPoint(m_driveSubsystem, Math.toRadians(0)),
          autoFactory.resetOdometry("RightToStation"),
          autoFactory.trajectoryCmd("RightToStation"),
          stop2,
          // Await for coral deposit...?
          new MoveToPoint(m_driveSubsystem, Math.toRadians(CoralStationAlignConstants.leftCoralStationRot)),
          new RunIntakeTimeLimited(intake, true /* We want it to intake, so in should be true */, lift, arm, 4),
          new MoveToPoint(m_driveSubsystem, Math.toRadians(0)),
          autoFactory.resetOdometry("RightFromStation"),
          autoFactory.trajectoryCmd("RightFromStation")//,
          // // Might not work due to time constraints, need to ensure rotation is zero for start of tele
          // stop3,
          // //new MoveToPoint(m_driveSubsystem, Math.toRadians(AutonomousNavConstants.endRotTwo)),
          // // new MoveToPoint(m_driveSubsystem, Math.toRadians(AutonomousNavConstants.endRotOne)),
          // // new JankLiftAutonomous(lift, arm, scoreHeight),
          // // new RunIntakeTimeLimited(intake, false /* We want it to outtake, so in should be false */, lift, arm, 2),
          // // new JankLiftAutonomous(lift, arm, Height.CoralStation),
          // // new MoveToPoint(m_driveSubsystem, Math.toRadians(0))
        );
      case Middle: // Weird case, have to check if collide with other robots
      return Commands.sequence(
        autoFactory.resetOdometry("Middle"),
        autoFactory.trajectoryCmd("Middle"),
        stop,
        new MoveToPoint(m_driveSubsystem, Math.toRadians(AutonomousNavConstants.endRotOne)),
        new JankLiftAutonomous(lift, arm, scoreHeight),
        new RunIntakeTimeLimited(intake, false /* We want it to outtake, so in should be false */, lift, arm, 2),
        new JankLiftAutonomous(lift, arm, Height.CoralStation),
        new MoveToPoint(m_driveSubsystem, Math.toRadians(0)),
        autoFactory.resetOdometry("MiddleToStation"),
        autoFactory.trajectoryCmd("MiddleToStation"),
        stop2,
        // Await for coral deposit...?
        new MoveToPoint(m_driveSubsystem, Math.toRadians(CoralStationAlignConstants.rightCoralStationRot)),
        new RunIntakeTimeLimited(intake, true /* We want it to intake, so in should be true */, lift, arm, 4),
        new MoveToPoint(m_driveSubsystem, Math.toRadians(0)),
        autoFactory.resetOdometry("MiddleFromStation"),
        autoFactory.trajectoryCmd("MiddleFromStation"),
        stop3,
        //new MoveToPoint(m_driveSubsystem, Math.toRadians(AutonomousNavConstants.endRotTwo)),
        new MoveToPoint(m_driveSubsystem, Math.toRadians(AutonomousNavConstants.endRotOne)),
        new JankLiftAutonomous(lift, arm, scoreHeight),
        new RunIntakeTimeLimited(intake, false /* We want it to outtake, so in should be false */, lift, arm, 2),
        new JankLiftAutonomous(lift, arm, Height.CoralStation),
        new MoveToPoint(m_driveSubsystem, Math.toRadians(0))
      );
      case Taxi:
        return Commands.sequence(
          autoFactory.resetOdometry("Taxi"),
          autoFactory.trajectoryCmd("Taxi"),
          stop
        );
      default:
        return Commands.sequence(
          stop
        );
    }

     

    /*
    return Commands.sequence(
        //for some reason, resetOdometry isn't working properly
        autoFactory.resetOdometry("Testing"),  
        autoFactory.trajectoryCmd("Testing"),
        stop,
        new MoveToPoint(m_driveSubsystem, Math.toRadians(AutonomousNavConstants.endRotOne)),
        //deposit coral
        // lift command
        // runintakeshort command
        new MoveToPoint(m_driveSubsystem, Math.toRadians(0)),
        autoFactory.resetOdometry("TestingPartTwo"),
        autoFactory.trajectoryCmd("TestingPartTwo"),
        stopAgain
    );
    */
  }
  public void SetUpDefaultCommand(){
    m_driveSubsystem.setDefaultCommand(fieldOrientedDrive);
  }

  public void resetBearings(){  
    m_driveSubsystem.resetOdometry(m_driveSubsystem.getPose());
    m_driveSubsystem.resetGyro();
  }

  public void resetGyro(){
    m_driveSubsystem.resetGyro();
  }
  public void getPose(){
    m_driveSubsystem.getPose();
    // SmartDashboard.putString("Auto", "AprilTag Alignment");
    // return new AprilTagAlignment(m_driveSubsystem, new AprilTagPoseEstimator(), 3, 0.5);
  }

  // TODO: Delete
  public void printPose() {
    Optional<Transform3d> opt = m_poseEstimator.getRobotToSeenTag();
    if(opt.isPresent()) {
      Transform3d r2t = opt.get();
      // SmartDashboard.putString("robot2tag", r2t.getTranslation().toString() + "Rotation3d(yaw="+r2t.getRotation().getZ()+", pitch="+r2t.getRotation().getY()+", roll="+r2t.getX()+")");
      SmartDashboard.putNumber("robot2tag/t/x", r2t.getTranslation().getX());
      SmartDashboard.putNumber("robot2tag/t/y", r2t.getTranslation().getY());
      SmartDashboard.putNumber("robot2tag/r/yaw", r2t.getRotation().getZ());
    }
  }

  public void StowLift(){
    // goToGround.schedule();
  }
  /**Returns true if robot roll or pitch exceeds the maximum tilt */
  public Boolean checkTilt(){
    if(m_driveSubsystem.getTilt()>LiftConstants.maximumTilt){
      return true;
    }
    return false;
  }
  public void getArmAngle(){
    arm.getArmAngle();
  }
  public void getLiftAngle(){
    lift.getLiftAngle();
  }
  /**Prints out the current height set by the controller onto SmartDashboard */
  public void getHeightSet(){
    lift.getHeight();
  }
}

