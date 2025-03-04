// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //CAN ID list
  //Swerve Drive 1-8
    // public static final int kFrontLeftDrivingCanId = 1;
    // public static final int kFrontLeftTurningCanId = 2;
    // public static final int kFrontRightDrivingCanId = 7;
    // public static final int kFrontRightTurningCanId = 8;
    // public static final int kRearLeftDrivingCanId = 3;
    // public static final int kRearLeftTurningCanId = 4;
    // public static final int kRearRightDrivingCanId = 5;
    // public static final int kRearRightTurningCanId = 6;
  //Misc Motors 11-14
    // public static final int liftNeoCANID=11;
    // public static final int reversedLiftNeoCANDID=12;
    // public static final int armCANID=13;
    // public static final int intakeCanID=14;

  public static class IntakeConstants{
    public static final int intakeCanID=14;
    public static final double intakePower=-0.4;
    public static final double outtakePower=0.4;
  }
  public static class AngleLimitConstants{
    //all in degrees, remember to convert to radians

    //arm angle below horizontal
    //in actuality, it can reach -154, but I can't see a reason why you would need to go that far
    public static final double minArmAngle=-140;
    //arm angle above horizontal
    //in actuality, it starts at 124
    public static final double maxArmAngle=128;
    public static final double maxLiftAngle=100;
    public static final double minLiftAngle=10;
  }
  public static class ArmConstants{
    public static final double kWeightMomentOffsetFactor=0.03;
    //get a engineer's square the next time I do this
    public static final double angleAtZero=140;
    public static final int decimalPlaces=2;
    public static final int armCANID=13;
    public static final int armEncoderPort=1;
    //its all in degrees, remember to convert
    public static final double[] desiredArmAngle= new double[]{
      //do note that ground height isn't actually at 0, because arm has minimum height
      0,//Ground intake (placeholder)
      -30,//L1
      -55,//L2
      -30,//L3
      60, //L4
      -80, //Algae2 
      -99, //Algae3 (placeholder)
      20, //CoralStation 
      -110 //StartingConfig 
    };
    public static final double maxPower=0.4;
    public static final double kPArm=0.13;
    public static final double kIArm=0.03;
    public static final double kDArm=0;
    //1 degree tolerance
    public static final double angleTolerance=Math.PI/180;

    public static final double armFallingPower=-0.01;
  }
  public static class LiftConstants{
    //for some reason at 0.04 the robot goes insane
    public static final double maximumPowerChange=0.05;
    public static final double maximumTilt=10;
    public static final double angleAtPeakHeight=138.8;
    public static final int encoderChannel=0;
    public static final int liftNeoCANID=11;
    public static final int reversedLiftNeoCANDID=12;
    //to ensure the arm doesn't tear itself apart. 
    public static final double maxPower=0.7;

    //angles required
    public static final double[] desiredLiftAngle= new double[]{
      //do note that ground height isn't actually at 0, because arm has minimum height
      0.22,//ground intake (placeholder) 
      Math.toRadians(30),//L1
      Math.toRadians(61),//L2
      Math.toRadians(86),//L3 
      Math.toRadians(80),//L4
      Math.toRadians(50),//Algae2 
      Math.toRadians(74),//Algae3 (placeholder)
      Math.toRadians(33),//CoralStation 
      Math.toRadians(35)//StartingConfig 
    };
    

    public static enum Height{
      Ground,
      L1,
      L2,
      L3,
      L4,
      StartingConfig, 
      Algae2, //algae between L2 and L3
      Algae3, //algae between L3 and L4
      CoralStation
    }
    public static final double armLength=27.4*0.0254;
    public static final double kPLift=0.6;
    public static final double kILift=0.03;
    public static final double kDLift=0;
    public static final double angleTolerance=Math.PI/180;
    //remember to adjust as more components are added
    public static final double kWeightMomentOffsetFactor=0.02;

    public static final double liftFallingPower=-0.25;
    public static final double L4OuttakeAngle=Math.toRadians(75);
  }
  public static class AccelerationLimiterConstants{
    public static double maximumAcceleration=0.02;
    public static double maximumDeceleration=0.02;
  }
  public static class TestingConstants{
    public static final double maximumSpeed=0.40;
    public static final double maximumRotationSpeed=0.01;
    public static final double maximumSpeedReduced=0.10;
    public static final double maximumRotationSpeedRobotOriented=0.005;
  }
  public static final class FieldOrientedDriveConstants{
    public static final double kFODP = 0.25;
    public static final double kFODI = 0.3;
    public static final double kFODD = 0.0;

    //Maximum rotation speed
    public static final double rotationScalar = Math.PI;

    public static final double bearingTolerance = 0.5;

    public static final double moveJoystickDeadzone=0.02;
  }
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(14) + 90;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(14) + 90;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontLeftTurningCanId = 2;
    public static final int kFrontRightDrivingCanId = 7;
    public static final int kFrontRightTurningCanId = 8;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kRearRightDrivingCanId = 5;
    public static final int kRearRightTurningCanId = 6;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManualLiftControllerPort=1;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final long kShooterDurationMillis = 5000; // 3s without intake then 2 seconds with it
    public static long kIntakeDurationMillis = 2000; // 2 seconds
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  // Don't think we need this, but just in case
  // public static final class LiftConstants{
  //   public static final double kP = 0.1;
  //   public static final double kI = 0;
  //   public static final double kD = 0;
  //   public static final double kTolerance = 0.1;

  //   public static final double kSetpointClimb = 1;
  //   public static final double kSetpointDeclimb = 0;

  // }
}