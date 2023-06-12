// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SwerveModuleConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Driver {
    public static final int kDriverControllerPort = 0;
  }

  public static class Swerve {
    public static final double kMaxSpeedTele = 3.0; //Meters per Second
    public static final double kMaxAngularSpeedFast = Math.PI; //Radians per Second

    public static final double kTrackWidth = Units.inchesToMeters(20);
    public static final double kWheelBase = Units.inchesToMeters(20);

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
      new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
      new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
      new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
    );

  }

  public static class Module {
    public static final double kDriveGearRatio = 1.0f / 8.14f;
    public static final double kAngleGearRatio = 1.0f / 12.8f;

    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kWheelCircumfrence = kWheelDiameter * Math.PI;

    public static final double kDrivePositionConversionFactor = kDriveGearRatio * kWheelCircumfrence;
    public static final double kDriveVelocityConverstionFactor = kDrivePositionConversionFactor / 60.0f;

    public static final double kAnglePositionConversionFactor = kAngleGearRatio * 360.0;
    public static final double kAngleVelocityConverstionFactor = kAnglePositionConversionFactor / 60.0f;

    public static final double kPAngle = 0;
    public static final double kIAngle = 0;
    public static final double kDAngle = 0;

    public static final double kPDrive = 0;
    public static final double kIDrive = 0;
    public static final double kDDrive = 0;

    public static final double kSDrive = 0;
    public static final double kVDrive = 0;
    public static final double kADrive = 0;

    public static final int kDriveCurrentLimit = 40;
    public static final int kAngleCurrentLimit = 30;

    public static final CANSparkMax.IdleMode kDriveIdleMode = CANSparkMax.IdleMode.kBrake;
    public static final CANSparkMax.IdleMode kAngleIdleMode = CANSparkMax.IdleMode.kCoast;

    public static final boolean angleMotorInverted = false;
    public static final boolean driveMotorInverted = false;
    public static final boolean absoluteEncoderInverted = false;
  }

  public static class Modules {
    public static class FrontLeft{
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 0;
      public static final int absoluteEncoderID = 0;

      public static final Rotation2d absoluteEncoderOffset = new Rotation2d(0);

      public static final SwerveModuleConfig FL0 = new SwerveModuleConfig(
        driveMotorID, 
        angleMotorID, 
        absoluteEncoderID, 
        absoluteEncoderOffset);
    }

    public static class FrontRight{
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 0;
      public static final int absoluteEncoderID = 0;

      public static final Rotation2d absoluteEncoderOffset = new Rotation2d(0);

      public static final SwerveModuleConfig FR1 = new SwerveModuleConfig(
        driveMotorID, 
        angleMotorID, 
        absoluteEncoderID, 
        absoluteEncoderOffset);
    }

    public static class BackLeft{
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 0;
      public static final int absoluteEncoderID = 0;

      public static final Rotation2d absoluteEncoderOffset = new Rotation2d(0);

      public static final SwerveModuleConfig BL2 = new SwerveModuleConfig(
        driveMotorID, 
        angleMotorID, 
        absoluteEncoderID, 
        absoluteEncoderOffset);
    }

    public static class BackRight{
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 0;
      public static final int absoluteEncoderID = 0;

      public static final Rotation2d absoluteEncoderOffset = new Rotation2d(0);

      public static final SwerveModuleConfig BR3 = new SwerveModuleConfig(
        driveMotorID, 
        angleMotorID, 
        absoluteEncoderID, 
        absoluteEncoderOffset);
    }
  }
}
