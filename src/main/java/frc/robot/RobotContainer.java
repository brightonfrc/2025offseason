// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.LiftConstants.Height;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Lift;
import frc.robot.commands.RunLift;
import frc.robot.commands.RunArm;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DatisLift;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
  new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandPS4Controller m_manualLiftController= new CommandPS4Controller(OIConstants.kManualLiftControllerPort);
  
  private final FieldOrientedDrive fieldOrientedDrive= new FieldOrientedDrive(m_driveSubsystem, m_driverController);
  // private Lift goToGround=new Lift(lift, Height.Ground);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    
    // m_driveSubsystem.setDefaultCommand(fieldOrientedDrive);

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
    //remember that A on the xbox controllerbutton is bound to "slow Mode"
    
    
    // m_manualLiftController.R1().whileTrue(new RunLift(lift, true));
    // m_manualLiftController.L1().whileTrue(new RunLift(lift, false));

    // m_manualLiftController.R2().whileTrue(new RunArm(arm, true, lift));
    // m_manualLiftController.L2().whileTrue(new RunArm(arm, false, lift));

    //bind intake and outtake to L3 and R3?

    
    m_driverController.y().onTrue(new Lift(lift, arm, Height.Ground));
    m_driverController.povUp().onTrue(new Lift(lift, arm, Height.L1));
    m_driverController.povRight().onTrue(new Lift(lift, arm, Height.L2));
    m_driverController.povDown().onTrue(new Lift(lift, arm, Height.L3));
    m_driverController.povLeft().onTrue(new Lift(lift, arm, Height.L4));
    m_driverController.x().onTrue(new Lift(lift, arm, Height.Stow));

    
    m_driverController.rightBumper().whileTrue(new RunArm(arm, true, lift));
    m_driverController.leftBumper().whileTrue(new RunArm(arm, false, lift));

    m_driverController.rightTrigger().whileTrue(new RunIntake(intake, true));
    m_driverController.leftTrigger().whileTrue(new RunIntake(intake, false));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
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
}

