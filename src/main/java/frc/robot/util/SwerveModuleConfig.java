package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConfig{
    public int driveMotorID;
    public int angleMotorID;
    public int absoluteEncoderId;
    
    public Rotation2d absoluteEncoderOffset;

    public SwerveModuleConfig(
        int driveMotorID,
        int angleMotorID,
        int absoluteEncoderID,
        Rotation2d absoluteEncoderOffset
    ) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.absoluteEncoderId = absoluteEncoderID;

        this.absoluteEncoderOffset = absoluteEncoderOffset;
    }
}