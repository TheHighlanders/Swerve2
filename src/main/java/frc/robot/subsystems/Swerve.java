// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Modules;


public class Swerve extends SubsystemBase {
  /* Array of Modules */
  public SwerveModule[] modules;
  private AHRS gyro;

  public Field2d field;

  public ChassisSpeeds chassisSpeeds;

  public SwerveDrivePoseEstimator swervePoseEstimator;

  public Swerve() {
    /* Initializes modules from Constants */
    modules = new SwerveModule[] {
      new SwerveModule(0, Modules.FrontLeft.FL0),
      new SwerveModule(1, Modules.FrontRight.FR1),
      new SwerveModule(2, Modules.BackLeft.BL2),
      new SwerveModule(3, Modules.BackRight.BR3)
    };
   
    gyro = new AHRS(SPI.Port.kMXP);
    zeroGyro();

    field = new Field2d();

    swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConst.kinematics, getYaw(), getModulePositions(), new Pose2d());

    chassisSpeeds = new ChassisSpeeds();
    // SmartDashboard.putData(field);
  }

  @Override
  public void periodic() {
    swervePoseEstimator.update(getYaw(), getModulePositions());

    //field.setRobotPose(getPose());
    for(SwerveModule m : modules){
      m.runPeriodicLimiting();
    }

  }

  /**
   * Runs all IK and sets modules states
   * @param translate Desired translations speeds m/s
   * @param rotate Desired rotation rate Rotation2d
   * @param fieldRelative Driving mode
   * @param isOpenLoop Drive controller mode
   */
  public void drive(Translation2d translate, Rotation2d rotate, boolean fieldRelative, boolean isOpenLoop){
    chassisSpeeds = fieldRelative ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(translate.getX(), translate.getY(), rotate.getRadians(), getYaw())
      : new ChassisSpeeds(translate.getX(), translate.getY(), rotate.getRadians());

    SwerveModuleState[] swerveModuleStates = Constants.SwerveConst.kinematics.toSwerveModuleStates(chassisSpeeds);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConst.kMaxSpeedTele);

    for(SwerveModule m : modules){
      m.setModuleState(swerveModuleStates[m.moduleNumber], isOpenLoop);
    }
  }

  /**
   * Zeros the NavX
   */
  public void zeroGyro(){
    gyro.zeroYaw();
  }

  /**
   * Returns the gyro's yaw
   * @return Yaw of gyro, includes zeroing
   */
  public Rotation2d getYaw(){
    return Rotation2d.fromDegrees(-1 * gyro.getYaw());
  }

  /**
   * Gets swerve modules positions for all modules
   * @return Array of modules positions, in modules ID order
   */
  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : modules) {
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Pose2d getPose() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  public Field2d getField() {
    return field;
  } 
  
  /**
   * Gets swerve modules states for all modules
   * @return Array of modules states, in modules ID order
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : modules) {
        states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void resetAllModulestoAbsol(){
    for(SwerveModule m : modules){
      m.setIntegratedAngleToAbsolute();
    }
  }

  public void sendAngleDiagnostic(){
    for(SwerveModule m : modules){
      SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Angle Actual", m.angleEncoder.getPosition());
    }
  }

  public void sendAngleTargetDiagnostic(){
    for(SwerveModule m : modules){
      SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Angle Target", m.angleReference);
    }
  }

  public void sendDriveDiagnostic(){
    double[] wheelSpeeds = new double[4];
    for(SwerveModule m : modules){
      SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Velocity Actual", m.driveEncoder.getVelocity());
      wheelSpeeds[m.driveMotor.getDeviceId()/10 - 1] = m.driveEncoder.getVelocity();
    }

    SmartDashboard.putNumber("Velocity Range", Math.abs(Arrays.stream(wheelSpeeds).max().getAsDouble()) - Math.abs(Arrays.stream(wheelSpeeds).min().getAsDouble()));
  }

  public void sendDriveTargetDiagnostic(){
    for(SwerveModule m : modules){
      SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Velocity Target", m.driveReference);
    }
  }

  public void sendAbsoluteDiagnostic(){
    for(SwerveModule m : modules){
      SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId()/10 + " Absolute", m.getAbsolutePosition().getDegrees());
      SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId()/10 + " ERROR", (m.angleEncoder.getPosition() - m.getAbsolutePosition().getDegrees())%360);
    }
  }

  public void sendSmartDashboardDiagnostics(){
    sendAngleDiagnostic();
    sendAngleTargetDiagnostic();

    sendDriveDiagnostic();
    sendDriveTargetDiagnostic();
   
    // sendAbsoluteDiagnostic();

    SmartDashboard.putNumber("NavX Angle", getYaw().getDegrees());
    SmartDashboard.putNumber("Pose X", getPose().getX());
    SmartDashboard.putNumber("Pose Y", getPose().getY());
    SmartDashboard.putNumber("Pose Theta", getPose().getRotation().getDegrees());
    
  }

  public void jogSingleModule(int moduleNumber, double input, boolean drive){
    if(drive){
      modules[moduleNumber].setDriveState(new SwerveModuleState(input, new Rotation2d(0)), false);
      DriverStation.reportWarning(modules[moduleNumber].driveMotor.getDeviceId() +"", false);
    } else{
      modules[moduleNumber].setAngleState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(input))));
      DriverStation.reportWarning(modules[moduleNumber].angleMotor.getDeviceId() +"", false);
    }
  }

  public void jogAllModuleDrive(double v){
    drive(new Translation2d(0, v), new Rotation2d(0), true, false);
  }

}
