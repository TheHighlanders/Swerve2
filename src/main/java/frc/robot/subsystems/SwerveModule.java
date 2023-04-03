package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private SparkMaxPIDController driveController;
    private SparkMaxPIDController angleController;

    private SimpleMotorFeedforward driveFeedforward;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder angleEncoder;

    private AnalogInput absoluteEncoder;

    private Rotation2d moduleAbsoluteOffset;
    private Rotation2d lastAngle;

    boolean driveMotorInverted;
    boolean angleMotorInverted;
    boolean absoluteEncoderInverted;

    public SwerveModule(int driveMotorID, int angleMotorID, int absoluteEncoderID, Rotation2d moduleAbsoluteOffset, 
            boolean driveMotorInverted, boolean angleMotorInverted, boolean absoluteEncoderInverted){
        
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();

        driveController = driveMotor.getPIDController();
        angleController = angleMotor.getPIDController();

        driveFeedforward = new SimpleMotorFeedforward(ModuleConstants.kSDrive, ModuleConstants.kVDrive, ModuleConstants.kADrive);

        absoluteEncoder = new AnalogInput(absoluteEncoderID);

        this.moduleAbsoluteOffset = moduleAbsoluteOffset;

        this.driveMotorInverted = driveMotorInverted;
        this.angleMotorInverted = angleMotorInverted;
        this.absoluteEncoderInverted = absoluteEncoderInverted;

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
            double motorPercent = state.speedMetersPerSecond / SwerveConstants.kMaxSpeedTele;
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
        Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= SwerveConstants.kMaxAngularSpeedFast * 0.001)
        ? lastAngle : state.angle;


        angleController.setReference(angle.getRadians(), ControlType.kPosition);
        lastAngle = state.angle;
    }

    /**
     * Returns the position of the Angle Motor, measured with integrated encoder
     * 
     * @return Angle Motor Position
     */
    private Rotation2d getAnglePosition(){
        return new Rotation2d(angleEncoder.getPosition());
    }

    /**
     * Returns the velocity of the Drive Motor, measured with integrated encoder
     * 
     * @return Drive Motor Velocity
     */
    private double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    /**
     * Gets the position of the Drive Motor, measured with integrated encoder
     * 
     * @return Drive Motor Position
     */
    private Rotation2d getDrivePosition() {
        return new Rotation2d(driveEncoder.getPosition());
    }

    /**
     * Gets the position of the module using the absolute encoder
     * 
     * @return Position of the module between 0 and 2Pi, as a Rotation2d
     */
    private Rotation2d getAbsolutePosition(){
        /*Gets absolute encoder voltage, and adjusts for fluctuating bus voltage, also converts to radians 0 to 2Pi */
        double positionRadians = ((absoluteEncoder.getVoltage() / RobotController.getVoltage5V()) * Math.PI * 2);
        
        /*Subtracts magnetic offset to get wheel position */
        positionRadians -= moduleAbsoluteOffset.getRadians();

        /* Inverts if necesary */
        positionRadians *= (absoluteEncoderInverted ? -1 : 1);

        return new Rotation2d(positionRadians);
    }

    /**
     * Configures Drive Motor using parameters from Constants
     */
    private void configureDriveMotor(){
        driveMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorInverted);
        driveMotor.setIdleMode(ModuleConstants.kDriveIdleMode);
        
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivePositionConversionFactor);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveVelocityConverstionFactor);

        driveController.setP(ModuleConstants.kPDrive);
        driveController.setI(ModuleConstants.kIDrive);
        driveController.setD(ModuleConstants.kDDrive);

        driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveCurrentLimit);

        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    /**
     * Configures Angle Motor using parameters from Constants
     */
    private void configureAngleMotor(){
        angleMotor.restoreFactoryDefaults();

        angleMotor.setInverted(angleMotorInverted);
        angleMotor.setIdleMode(ModuleConstants.kAngleIdleMode);
        
        angleEncoder.setPositionConversionFactor(ModuleConstants.kAnglePositionConversionFactor);
        angleEncoder.setVelocityConversionFactor(ModuleConstants.kAngleVelocityConverstionFactor);

        angleController.setP(ModuleConstants.kPAngle);
        angleController.setI(ModuleConstants.kIAngle);
        angleController.setD(ModuleConstants.kDAngle);

        angleController.setPositionPIDWrappingMaxInput(Math.PI);
        angleController.setPositionPIDWrappingMinInput(-Math.PI);
        angleController.setPositionPIDWrappingEnabled(true);

        angleMotor.setSmartCurrentLimit(ModuleConstants.kAngleCurrentLimit);

        angleMotor.burnFlash();

        setIntegratedAngleToAbsolute();
    }

    /**
     * Resets the Angle Motor to the position of the absolute position
     */
    private void setIntegratedAngleToAbsolute(){
        angleEncoder.setPosition(getAbsolutePosition().getRadians());
    }
}
