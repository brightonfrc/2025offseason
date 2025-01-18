// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.FieldOrientedDriveConstants;
import frc.robot.Constants.TestingConstants;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.kauailabs.navx.frc.AHRS;

/** An example command that uses an example subsystem. */
public class FieldOrientedDrive extends Command {
    private DriveSubsystem driveSubsystem;
    private CommandXboxController xboxController;
    private AHRS gyro;
    private PIDController bearingPIDController;

    private double goalBearing;
    private double joystickTurnBearing;
    private double joystickMoveMagnitude;
    private double joystickMoveBearing;
    private double robotBearing;
    private double rotSpeed;
    private double xSpeed;
    private double ySpeed;
    private double maximumRotationSpeed;
    private double maximumSpeed;
    private Boolean slowMode;
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public FieldOrientedDrive(DriveSubsystem driveSubsystem, CommandXboxController xboxController, Boolean slowMode) {
        this.driveSubsystem = driveSubsystem;
        this.xboxController = xboxController;
        this.slowMode = slowMode;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        gyro = new AHRS(SPI.Port.kMXP);
        bearingPIDController = new PIDController(FieldOrientedDriveConstants.kFODP, FieldOrientedDriveConstants.kFODI, FieldOrientedDriveConstants.kFODD);
        //setting a tolerance of 2 degrees
        bearingPIDController.setTolerance(Math.PI/180);
        bearingPIDController.setSetpoint(0);
        bearingPIDController.enableContinuousInput(0, 2*Math.PI);
        if (slowMode == true) {
            maximumRotationSpeed = TestingConstants.maximumRotationSpeedReduced;
            maximumSpeed = TestingConstants.maximumSpeedReduced;
        } else {
            maximumRotationSpeed = TestingConstants.maximumRotationSpeed;
            maximumSpeed = TestingConstants.maximumSpeed;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        SmartDashboard.putNumber("Goal bearing", goalBearing);


        //Both joysticks assumes the right to be bearing 0 and then works clockwise from there. To have bearing 0 be in front, the bearing
        //has to be moved back by 90 degrees/ 1/2 PI

        //If right joystick is not being moved retain previous bearing
        if (Math.hypot(xboxController.getRightY(), xboxController.getRightX())>  0.5) {
            joystickTurnBearing = Math.atan2(xboxController.getRightY(), xboxController.getRightX()) + Math.PI/2;
        }
        SmartDashboard.putNumber("Turn: Right Joystick bearing", joystickTurnBearing);

        //error tolerance of 2 degrees
        if (Math.abs(joystickTurnBearing-goalBearing)>Math.PI/180*FieldOrientedDriveConstants.bearingTolerance){
            goalBearing = -joystickTurnBearing;
            bearingPIDController.reset();
            bearingPIDController.setSetpoint(goalBearing);
        }
        robotBearing = gyro.getAngle() % 360;
        //converting to radians
        robotBearing = robotBearing / 180 * Math.PI;
        SmartDashboard.putNumber("Robot bearing", robotBearing);

        joystickMoveBearing = Math.atan2(xboxController.getLeftY(), xboxController.getLeftX());
        SmartDashboard.putNumber("Drive: Left joystick bearing", joystickMoveBearing);

        joystickMoveBearing = joystickMoveBearing - robotBearing;
        SmartDashboard.putNumber("Drive: Robot Relative bearing", joystickMoveBearing);

        joystickMoveMagnitude = Math.pow(Math.pow(xboxController.getLeftX(), 2) + Math.pow(xboxController.getLeftY(), 2), 0.5);
        SmartDashboard.putNumber("Drive: Left joystick magnitude", joystickMoveMagnitude);

        xSpeed = -joystickMoveMagnitude * Math.cos(joystickMoveBearing) * (xboxController.a().getAsBoolean() ? TestingConstants.maximumSpeedReduced : TestingConstants.maximumSpeed);
        SmartDashboard.putNumber("xSpeed", xSpeed);

        ySpeed = -joystickMoveMagnitude * Math.sin(joystickMoveBearing) * (xboxController.a().getAsBoolean() ? TestingConstants.maximumSpeedReduced : TestingConstants.maximumSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);

        rotSpeed = bearingPIDController.calculate(robotBearing) * (xboxController.a().getAsBoolean() ? TestingConstants.maximumRotationSpeedReduced : TestingConstants.maximumRotationSpeed);
        SmartDashboard.putNumber("rotSpeed", rotSpeed);

        driveSubsystem.drive(ySpeed, xSpeed, rotSpeed, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}