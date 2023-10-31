package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Module;
import frc.robot.Constants.SwerveConst;
import frc.robot.util.SwerveModuleConfig;

public class SwerveModule {
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    public int moduleNumber;

    private SparkMaxPIDController driveController;
    private SparkMaxPIDController angleController;

    private SimpleMotorFeedforward driveFeedforward;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder angleEncoder;

    private SparkMaxAbsoluteEncoder absoluteEncoder;

    private Rotation2d moduleAbsoluteOffset;
    private Rotation2d lastAngle;

    public SwerveModule(int moduleNumber, SwerveModuleConfig config){
        
        driveMotor = new CANSparkMax(config.driveMotorID, MotorType.kBrushless);
        angleMotor = new CANSparkMax(config.angleMotorID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();
        
        driveController = driveMotor.getPIDController();
        angleController = angleMotor.getPIDController();
        
        /* Creates an additional FF controller for extra drive motor control */
        driveFeedforward = new SimpleMotorFeedforward(Module.kSDrive, Module.kVDrive, Module.kADrive);

        absoluteEncoder = angleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        this.moduleAbsoluteOffset = config.absoluteEncoderOffset;

        configureDriveMotor();
        configureAngleMotor();
    }

    /**
     * Sets both Angle and Drive to desired states
     * 
     * @param state: Desired module state
     * @param isOpenLoop: Controls if the drive motor use a PID loop
     */
    public void setModuleState(SwerveModuleState state, boolean isOpenLoop){
        state = SwerveModuleState.optimize(state, getAnglePosition());
        
        setAngleState(state);
        setDriveState(state, isOpenLoop);
    }

    /**
     * Sets the Drive Motor to a desired state, 
     * if isOpenLoop is true, it will be set as a percent, if it is false, than it will use a velocity PIDF loop
     * 
     * @param state: Desired module state
     * @param isOpenLoop: Whether or not to use a PID loop
     */
    public void setDriveState(SwerveModuleState state, boolean isOpenLoop){
        if(isOpenLoop){
            double motorPercent = state.speedMetersPerSecond / SwerveConst.kMaxSpeedTele;
            driveMotor.set(motorPercent);
        } else {
            driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, 0, 
                                                        driveFeedforward.calculate(state.speedMetersPerSecond));
        }

    }

    /**
     * Sets the Angle Motor to a desired state, does not set the state if speed is too low, to stop wheel jitter
     * 
     * @param state: Desired module state
     */
    public void setAngleState(SwerveModuleState state){
        //Anti Jitter Code, not sure if it works, need to test and review
        // Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= SwerveConst.kMaxAngularSpeedFast * 0.001)
        // ? lastAngle : state.angle;
        Rotation2d angle = state.angle;

        SmartDashboard.putString("Module " + driveMotor.getDeviceId() / 10 + " Angle Target", state.angle.getDegrees() + ""); // Added Because angle, due to lastAngle was null, due to having no default -AH 2023-10-31
        SmartDashboard.putNumber("Module " + driveMotor.getDeviceId() / 10 + " Angle Actual", angleEncoder.getPosition());
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = state.angle;
    }

    /**
     * Returns the position of the Angle Motor, measured with integrated encoder
     * 
     * @return Angle Motor Position
     */
    public Rotation2d getAnglePosition(){
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    /**
     * Returns the velocity of the Drive Motor, measured with integrated encoder
     * 
     * @return Drive Motor Velocity
     */
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    /**
     * Gets the position of the Drive Motor, measured with integrated encoder
     * 
     * @return Drive Motor Position
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Gets the position of the module using the absolute encoder
     * 
     * @return Position of the module between 0 and 360, as a Rotation2d
     */
    public Rotation2d getAbsolutePosition(){
        /* Gets Position from SparkMAX absol encoder * 360  to degrees */
        double positionDeg = absoluteEncoder.getPosition() * 360.0d;
        
        /*Subtracts magnetic offset to get wheel position */
        positionDeg -= moduleAbsoluteOffset.getDegrees();

        /* Inverts if necesary */
        positionDeg *= (Module.absoluteEncoderInverted ? -1 : 1);

        return new Rotation2d(positionDeg);
    }

    /**
     * 
     * @return Swerve Module Position (Position & Angle)
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), getAnglePosition());
    }

    /**
     * 
     * @return Swerve Module State (Velocity & Angle)
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), getAnglePosition());
    }

    /**
     *  Returns the assigned module number
     */
    public int getModuleNumber(){
        return moduleNumber;
    }

    /**
     * Configures Drive Motor using parameters from Constants
     */
    private void configureDriveMotor(){
        driveMotor.restoreFactoryDefaults();

        driveMotor.setInverted(Module.driveMotorInverted);
        driveMotor.setIdleMode(Module.kDriveIdleMode);
        
        /* Sets encoder ratios to actual module gear ratios */
        driveEncoder.setPositionConversionFactor(Module.kDrivePositionConversionFactor);
        driveEncoder.setVelocityConversionFactor(Module.kDriveVelocityConverstionFactor);

        /* Configures PID loop */
        driveController.setP(Module.kPDrive);
        driveController.setI(Module.kIDrive);
        driveController.setD(Module.kDDrive);

        driveMotor.setSmartCurrentLimit(Module.kDriveCurrentLimit);

        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    /**
     * Configures Angle Motor using parameters from Constants
     */
    private void configureAngleMotor(){
        angleMotor.restoreFactoryDefaults();

        angleMotor.setInverted(Module.angleMotorInverted);
        angleMotor.setIdleMode(Module.kAngleIdleMode);
        
        /* Sets encoder ratios to actual module gear ratios */
        angleEncoder.setPositionConversionFactor(Module.kAnglePositionConversionFactor);
        angleEncoder.setVelocityConversionFactor(Module.kAngleVelocityConverstionFactor);

        /* Configures PID loop */
        angleController.setP(Module.kPAngle);
        angleController.setI(Module.kIAngle);
        angleController.setD(Module.kDAngle);

        /* Defines wheel angles as -pi to pi */
        angleController.setPositionPIDWrappingMaxInput(Math.PI);
        angleController.setPositionPIDWrappingMinInput(-Math.PI);
        angleController.setPositionPIDWrappingEnabled(true);

        angleMotor.setSmartCurrentLimit(Module.kAngleCurrentLimit);

        angleMotor.burnFlash();

        setIntegratedAngleToAbsolute();
    }

    /**
     * Resets the Angle Motor to the position of the absolute position
     */
    public void setIntegratedAngleToAbsolute(){
        // Disabled to allow non absolute motor movement for bench testing 10/27 -AH
        // angleEncoder.setPosition(getAbsolutePosition().getDegrees());
    }
}
