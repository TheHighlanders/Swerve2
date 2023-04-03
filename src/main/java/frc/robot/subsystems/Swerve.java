// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Modules;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  public SwerveModule frontLeft;
  public SwerveModule frontRight;
  public SwerveModule backLeft;
  public SwerveModule backRight;

  public Swerve() {
    /*Constructs Modules */
    frontLeft = Modules.FrontLeft.frontLeft;
    frontRight = Modules.FrontRight.frontRight;
    backLeft = Modules.BackLeft.backLeft;
    backRight = Modules.BackRight.backRight;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
