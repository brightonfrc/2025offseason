// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AprilTagAlignmentConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AprilTagAlignment;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.subsystems.AprilTagPoseEstimator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final DriveSubsystem m_driveSubsystem= new DriveSubsystem();
  private final AprilTagPoseEstimator m_poseEstimator = new AprilTagPoseEstimator();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
  new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final FieldOrientedDrive fieldOrientedDrive= new FieldOrientedDrive(m_driveSubsystem, m_driverController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_driveSubsystem.setDefaultCommand(fieldOrientedDrive);
    // m_driveSubsystem.drive(m_driverController.getLeftX(), m_driverController.getLeftY(), 0, false);
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
  private void configureBindings() {
    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    //align right
    m_driverController.leftStick().onTrue(new AprilTagAlignment(m_driveSubsystem, m_poseEstimator, 3, AprilTagAlignmentConstants.stopDisplacementX, AprilTagAlignmentConstants.stopDisplacementY));
    //align left
    m_driverController.rightStick().onTrue(new AprilTagAlignment(m_driveSubsystem, m_poseEstimator, 3, AprilTagAlignmentConstants.stopDisplacementX, -AprilTagAlignmentConstants.stopDisplacementY));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // SmartDashboard.putString("Auto", "AprilTag Alignment");
    return null;
    // return new AprilTagAlignment(m_driveSubsystem, new AprilTagPoseEstimator(), 3, 0.5, 0);
  }

  // TODO: Delete
  public void printPose() {
    Optional<Transform3d> opt = m_poseEstimator.getRobotToTag();
    if(opt.isPresent()) {
      Transform3d r2t = opt.get();
      SmartDashboard.putString("robot2tag", r2t.getTranslation().toString() + "Rotation3d(yaw="+r2t.getRotation().getZ()+", pitch="+r2t.getRotation().getY()+", roll="+r2t.getX()+")");
      SmartDashboard.putNumber("robot2tag/t/x", r2t.getTranslation().getX());
      SmartDashboard.putNumber("robot2tag/t/y", r2t.getTranslation().getY());
      SmartDashboard.putNumber("robot2tag/t/z", r2t.getTranslation().getZ());
      SmartDashboard.putNumber("robot2tag/r/y", r2t.getRotation().getZ());
      SmartDashboard.putNumber("robot2tag/r/p", r2t.getRotation().getY());
      SmartDashboard.putNumber("robot2tag/r/r", r2t.getX());
    }
  }
}
