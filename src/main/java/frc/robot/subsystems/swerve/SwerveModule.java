package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.Reportable;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;

/**
 * Swerve module that uses CANCoder for the absolute position
 */
public class SwerveModule implements Reportable {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder canCoder;
    private final TalonFXConfigurator driveConfigurator;
    private final TalonFXConfigurator turnConfigurator;

    private final PositionVoltage turnRequest;
    private final MotionMagicVelocityVoltage driveRequest;
    private final NeutralOut brakeRequest;

    private final int driveMotorID;
    private final int turnMotorID;
    private final int CANCoderID;

    private double currentPercent = 0;
    private double currentTurnPercent = 0;
    private double currentAngle = 0;
    private double desiredAngle = 0;
    private double desiredVelocityRPS = 0;
    private boolean velocityControl = true;

    private SwerveModuleState desiredState = null;
    private SwerveModulePosition currPosition = new SwerveModulePosition();
    private SwerveModuleState currState = new SwerveModuleState();

    /**
     * Construct a new CANCoder Swerve Module.
     * 
     * @param driveMotorId
     * @param turningMotorId
     * @param invertDriveMotor
     * @param invertTurningMotor
     * @param CANCoderId
     * @param CANCoderOffsetDegrees
     * @param CANCoderReversed
     */
    public SwerveModule(int driveMotorId, int turningMotorId, boolean invertDriveMotor, boolean invertTurningMotor, 
        int CANCoderId, boolean CANCoderReversed) {
        this.canCoder = new CANcoder(CANCoderId, ModuleConstants.kCANivoreName);
        this.driveMotor = new TalonFX(driveMotorId, ModuleConstants.kCANivoreName);
        this.turnMotor = new TalonFX(turningMotorId, ModuleConstants.kCANivoreName);
        
        this.driveConfigurator = driveMotor.getConfigurator();
        this.turnConfigurator = turnMotor.getConfigurator();
        
        this.driveRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(false);
        this.turnRequest = new PositionVoltage(0).withEnableFOC(false);
        this.brakeRequest = new NeutralOut();

        this.driveMotorID = driveMotorId;
        this.turnMotorID = turningMotorId;
        this.CANCoderID = CANCoderId;

        this.driveMotor.setInverted(invertDriveMotor);
        this.turnMotor.setInverted(invertTurningMotor);
        
        this.desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

        configureMotors();
        refreshPID();
    }

    public void configureMotors() {
        TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration();
        driveConfigurator.refresh(driveMotorConfigs);
        driveMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveMotorConfigs.Feedback.RotorToSensorRatio = 1;
        driveMotorConfigs.Feedback.SensorToMechanismRatio = ModuleConstants.kDriveMotorGearRatio;
        driveMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        driveMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
        driveMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveMotorConfigs.MotorOutput.DutyCycleNeutralDeadband = ModuleConstants.kDriveMotorDeadband;
        driveMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        driveMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        driveMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        driveMotorConfigs.Audio.AllowMusicDurDisable = true;

        TalonFXConfiguration turnMotorConfigs = new TalonFXConfiguration();
        turnConfigurator.refresh(turnMotorConfigs);
        turnMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turnMotorConfigs.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
        turnMotorConfigs.Feedback.RotorToSensorRatio = ModuleConstants.kTurnMotorGearRatio;
        turnMotorConfigs.Feedback.SensorToMechanismRatio = 1;
        turnMotorConfigs.ClosedLoopGeneral.ContinuousWrap = true;
        turnMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        turnMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
        turnMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnMotorConfigs.MotorOutput.DutyCycleNeutralDeadband = ModuleConstants.kDriveMotorDeadband;
        turnMotorConfigs.CurrentLimits.SupplyCurrentLimit = 30;
        turnMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnMotorConfigs.CurrentLimits.SupplyCurrentThreshold = 20;
        turnMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        turnMotorConfigs.Audio.AllowMusicDurDisable = true;

        StatusCode result = driveConfigurator.apply(driveMotorConfigs);
        if (!result.isOK()) {
            DriverStation.reportError("Could not apply drive configs, error code: "+ result.toString(), true);
        }

        result = turnConfigurator.apply(turnMotorConfigs);
        if (!result.isOK()) {
            DriverStation.reportError("Could not apply turn configs, error code: "+ result.toString(), true);
        }
    }

    public void refreshPID() {
        Slot0Configs turnPIDConfigs = new Slot0Configs();
        turnConfigurator.refresh(turnPIDConfigs);
        ModuleConstants.kPTurning.loadPreferences();
        ModuleConstants.kITurning.loadPreferences();
        ModuleConstants.kDTurning.loadPreferences();
        ModuleConstants.kSTurning.loadPreferences();
        ModuleConstants.kVTurning.loadPreferences();
        turnPIDConfigs.kP = ModuleConstants.kPTurning.get();
        turnPIDConfigs.kI = ModuleConstants.kITurning.get();
        turnPIDConfigs.kD = ModuleConstants.kDTurning.get();
        turnPIDConfigs.kS = ModuleConstants.kSTurning.get();
        turnPIDConfigs.kV = ModuleConstants.kVTurning.get();
        
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigurator.refresh(driveConfigs);
        ModuleConstants.kPDrive.loadPreferences();
        ModuleConstants.kIDrive.loadPreferences();
        ModuleConstants.kDDrive.loadPreferences();
        ModuleConstants.kVDrive.loadPreferences();
        ModuleConstants.kSDrive.loadPreferences();
        ModuleConstants.kSDrive.loadPreferences();
        ModuleConstants.kDriveMotionMagicAcceleration.loadPreferences();
        ModuleConstants.kDriveMotionMagicJerk.loadPreferences();
        driveConfigs.Slot0.kP = ModuleConstants.kPDrive.get();
        driveConfigs.Slot0.kI = ModuleConstants.kIDrive.get();
        driveConfigs.Slot0.kD = ModuleConstants.kDDrive.get();
        driveConfigs.Slot0.kV = ModuleConstants.kVDrive.get();
        driveConfigs.Slot0.kS = ModuleConstants.kSDrive.get();
        driveConfigs.Slot0.kA = ModuleConstants.kADrive.get();
        driveConfigs.MotionMagic.MotionMagicAcceleration = ModuleConstants.kDriveMotionMagicAcceleration.get();
        driveConfigs.MotionMagic.MotionMagicJerk = ModuleConstants.kDriveMotionMagicJerk.get();

        StatusCode result = driveConfigurator.apply(driveConfigs);
        if (!result.isOK()) {
            DriverStation.reportError("Could not apply drive PID, error code: "+ result.toString(), true);
        }
        result = turnConfigurator.apply(turnPIDConfigs);
        if (!result.isOK()) {
            DriverStation.reportError("Could not apply turn PID, error code: "+ result.toString(), true);
        }
    }

    /**
     * Set the percent output of both motors to zero.
     */
    public void stop() {
        driveMotor.setControl(brakeRequest);
        turnMotor.setControl(brakeRequest);

        this.desiredState = new SwerveModuleState(0, Rotation2d.fromRadians(getTurningPosition()));
    }

    public void run() {
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(getTurningPosition()));

        desiredAngle = desiredState.angle.getDegrees();

        this.desiredVelocityRPS = desiredState.speedMetersPerSecond / ModuleConstants.kMetersPerRevolution;
        
        if (Math.abs(desiredVelocityRPS) < 0.001) {
            driveMotor.setControl(brakeRequest);
        }
        
        turnRequest.Slot = 0;
        turnRequest.Position = desiredState.angle.getRotations();
        turnMotor.setControl(this.turnRequest);
    }

    public void resetDesiredAngle() {
        this.desiredAngle = 0;
    }
    
    //****************************** GETTERS ******************************/

    /**
     * Get the distance travelled by the motor in meters
     * @return Distance travelled by motor (in meters)
     */
    public double getDrivePosition() {
        return driveMotor.getRotorPosition().getValue()
            * ModuleConstants.kMetersPerRevolution;
    }

    public double getDrivePositionTicks() {
        return driveMotor.getRotorPosition().getValue() * 2048;
    }


    /**
     * Get the turning motor's CANCoder's angle
     * @return Angle in radians
     */
    public double getTurningPosition() {
        double turningPosition = Math.toRadians(getTurningPositionDegrees());
        return turningPosition;
    }

    /**
     * Get the turning motor's CANCoder's angle
     * @return Angle in degrees
     */
    public double getTurningPositionDegrees() {
        double turningPosition = (360 * canCoder.getAbsolutePosition().getValue()) % 360;
        return turningPosition;
    }

    /**
     * Get the velocity of the drive motor
     * @return Velocity of the drive motor (in meters / sec)
     */
    public double getDriveVelocity() {
        return driveMotor.getRotorVelocity().getValue() 
            * ModuleConstants.kMetersPerRevolution;
    }

    /**
     * Get the velocity of the drive motor
     * @return Velocity of the drive motor (in meters / sec)
     */
    public double getDriveVelocityRPS() {
        return driveMotor.getRotorVelocity().getValue();
    }

    /**
     * Get the velocity of the turning motor
     * @return Velocity of the turning motor (in radians / sec)
     */
    public double getTurningVelocity() {
        double turnVelocity = Math.toRadians(getTurningVelocityDegrees());
        return turnVelocity;
    }

    /**
     * Get the velocity of the turning motor
     * @return Velocity of the turning motor (in degrees / sec)
     */
    public double getTurningVelocityDegrees() {
        double turnVelocity = canCoder.getVelocity().getValue();
        return turnVelocity;
    }

    /**
     * Return the current state (velocity and rotation) of the Swerve Module
     * @return This Swerve Module's State
     */
    public SwerveModuleState getState() {
        currState.speedMetersPerSecond = getDriveVelocity();
        currState.angle = Rotation2d.fromRadians(getTurningPosition());
        return currState;
        // return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));

    }

    public SwerveModulePosition getPosition() {
        currPosition.distanceMeters = getDrivePosition();
        currPosition.angle = Rotation2d.fromRadians(getTurningPosition());
        return currPosition;
        //return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    //****************************** SETTERS ******************************/

    /**
     * Set the desired state of the Swerve Module and move towards it
     * @param state The desired state for this Swerve Module
     */
    public void setDesiredState(SwerveModuleState state, boolean withVelocityControl) {
        this.velocityControl = withVelocityControl;
        setDesiredState(state);
    }

    /**
     * Set the desired state of the Swerve Module and move towards it
     * @param state The desired state for this Swerve Module
     */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            state.speedMetersPerSecond = 0;
        }

        this.desiredState = state;
    }

    public void toggleVelocityControl(boolean velocityControlOn) {
        this.velocityControl = velocityControlOn;
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }
        int moduleId = (driveMotorID / 10);
        ShuffleboardTab tab = Shuffleboard.getTab("Module " + moduleId);

        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addNumber("Turn percent (motor controller)", () -> turnMotor.getDutyCycle().getValue());
                tab.addNumber("Turn percent (current)", () -> this.currentTurnPercent);
            case MEDIUM:
                tab.addNumber("Turn Motor Current", () -> turnMotor.getStatorCurrent().getValue());
                tab.addNumber("Drive Motor Voltage", () -> (driveMotor.getDutyCycle().getValue() * driveMotor.getSupplyVoltage().getValue()));
                tab.addNumber("Turn Motor Voltage", () -> turnMotor.getSupplyVoltage().getValue());// ::getMotorOutputVoltage);
                tab.addNumber("Drive percent (motor controller)", () -> driveMotor.getDutyCycle().getValue());
                tab.addNumber("Drive percent (current)", () -> this.currentPercent);
                
                tab.addNumber("Drive ticks", this::getDrivePositionTicks);
                tab.addNumber("Turn angle percent", () -> turnMotor.getDutyCycle().getValue());
                tab.addNumber("Angle Difference", () -> desiredAngle - currentAngle);
            case MINIMAL:
                tab.addNumber("Turn angle", this::getTurningPositionDegrees);
                tab.addNumber("Desired Angle", () -> desiredAngle);
                tab.addNumber("Drive Motor Current", () -> driveMotor.getSupplyCurrent().getValue());
                // tab.addNumber("Module Velocity", this::getDriveVelocity);
                tab.addNumber("Module Velocity RPS", this::getDriveVelocityRPS);
                tab.addNumber("Desired Velocity", () -> this.desiredVelocityRPS);
                tab.addBoolean("Velocity Control", () -> this.velocityControl);
                tab.addString("Error Status", () -> driveMotor.getFaultField().getName());
                break;
            }
            
    }

     public void reportToSmartDashboard(LOG_LEVEL level) {
    //     currentAngle = Math.toDegrees(getTurningPosition());
    //     switch (level) {
    //         case OFF:
    //             break;
    //         case ALL:
    //             SmartDashboard.putNumber("Drive Motor #" + driveMotorID + " Current", driveMotor.getStatorCurrent().getValue());
    //             SmartDashboard.putNumber("Turn Motor #" + turnMotorID + " Current", turnMotor.getStatorCurrent().getValue());
    //             SmartDashboard.putNumber("Drive Motor #" + driveMotorID + " Voltage", (driveMotor.getDutyCycle().getValue() * driveMotor.getSupplyVoltage().getValue()));
    //             SmartDashboard.putNumber("Turn Motor #" + turnMotorID + " Voltage", (turnMotor.getDutyCycle().getValue() * turnMotor.getSupplyVoltage().getValue()));
    //             SmartDashboard.putNumber("Turn Offset", this.CANCoderOffsetDegrees.get());
    //         case MEDIUM:
    //             SmartDashboard.putNumber("Module velocity #" + driveMotorID, getDriveVelocity());
    //             SmartDashboard.putNumber("Drive percent #" + driveMotorID, driveMotor.getDutyCycle().getValue());
    //             SmartDashboard.putNumber("Turn Angle #" + turnMotorID, currentAngle);
    //             SmartDashboard.putNumber("Turn Error #" + turnMotorID, desiredAngle - currentAngle);
    //         case MINIMAL:
    //             break;
    //     }

     }

    /**
     * Enable or disable the break mode on the motors
     * @param breaking  Whether or not the motor should be on break mode
     */
    public void setBreak(boolean breaking) {
        NeutralModeValue mode = (breaking ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        
        MotorOutputConfigs turnConfigs = new MotorOutputConfigs();
        this.turnConfigurator.refresh(turnConfigs);
        turnConfigs.NeutralMode = mode;
        this.turnConfigurator.apply(turnConfigs);
        
        MotorOutputConfigs driveConfigs = new MotorOutputConfigs();
        this.driveConfigurator.refresh(driveConfigs);
        driveConfigs.NeutralMode = mode;
        this.driveConfigurator.apply(driveConfigs);
    }
}