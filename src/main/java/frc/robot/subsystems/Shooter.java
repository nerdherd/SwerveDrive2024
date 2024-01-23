package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase implements Reportable{

    final TalonFX topShooter;
    final TalonFX bottomShooter;

    final VoltageOut m_topVoltageRequest = new VoltageOut(0);
    final VoltageOut m_bottomVoltageRequest = new VoltageOut(0);
    final DutyCycleOut m_topDutyCycleRequest = new DutyCycleOut(0);
    final DutyCycleOut m_bottomDutyCycleRequest = new DutyCycleOut(0);

    final VelocityVoltage m_topVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    final VelocityVoltage m_bottomVelocity = new VelocityVoltage(0, 0, true, 0,0, false, false, false);

    final NeutralOut m_brake = new NeutralOut();

    public Shooter(){
        topShooter = new TalonFX(ShooterConstants.kTopMotorID, ModuleConstants.kCANivoreName);
        bottomShooter = new TalonFX(ShooterConstants.kBottomMotorID, ModuleConstants.kCANivoreName);
        topShooter.setInverted(false);

        refreshPID();

    }

    public void refreshPID() {
        TalonFXConfiguration topMotorConfigs = new TalonFXConfiguration();
        
        topShooter.getConfigurator().refresh(topMotorConfigs);
        ShooterConstants.kPTopMotor.loadPreferences();
        ShooterConstants.kITopMotor.loadPreferences();
        ShooterConstants.kDTopMotor.loadPreferences();
        ShooterConstants.kVTopMotor.loadPreferences();
        topMotorConfigs.Slot0.kP = ShooterConstants.kPTopMotor.get();
        topMotorConfigs.Slot0.kI = ShooterConstants.kITopMotor.get();
        topMotorConfigs.Slot0.kD = ShooterConstants.kDTopMotor.get();
        topMotorConfigs.Slot0.kV = ShooterConstants.kVTopMotor.get();
        
        TalonFXConfiguration bottomMotorConfigs = new TalonFXConfiguration();
        
        bottomShooter.getConfigurator().refresh(bottomMotorConfigs);
        ShooterConstants.kPBottomMotor.loadPreferences();
        ShooterConstants.kIBottomMotor.loadPreferences();
        ShooterConstants.kDBottomMotor.loadPreferences();
        ShooterConstants.kVBottomMotor.loadPreferences();
        bottomMotorConfigs.Slot0.kP = ShooterConstants.kPBottomMotor.get();
        bottomMotorConfigs.Slot0.kI = ShooterConstants.kIBottomMotor.get();
        bottomMotorConfigs.Slot0.kD = ShooterConstants.kDBottomMotor.get();
        bottomMotorConfigs.Slot0.kV = ShooterConstants.kVBottomMotor.get();

        topMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        topMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
        bottomMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        bottomMotorConfigs.Voltage.PeakReverseVoltage = -11.5;

        StatusCode statusTop = topShooter.getConfigurator().apply(topMotorConfigs);
        StatusCode statusBottom = bottomShooter.getConfigurator().apply(bottomMotorConfigs);

        if(!statusTop.isOK()){
            DriverStation.reportError("Could not apply top shooter configs, error code:"+statusTop.toString(), new Error("Could not apply top shooter configs").getStackTrace());
        }
        if(!statusBottom.isOK()){
            DriverStation.reportError("Could not apply bottom shooter configs, error code:"+statusBottom.toString(),  new Error("Could not apply top shooter configs").getStackTrace());
        }
    }

    public Command setBottomSpeed(double bottomSpeed) {
        return Commands.runOnce(() -> {
            m_bottomVelocity.Slot = 0;
            bottomShooter.setControl(m_bottomVelocity.withVelocity(bottomSpeed));
            SmartDashboard.putBoolean("Pressed", true);
        });
    }


    public Command setTopSpeed(double topSpeed) {
        return Commands.runOnce(() -> {
            m_topVelocity.Slot = 0;
            topShooter.setControl(m_topVelocity.withVelocity(topSpeed));
            SmartDashboard.putBoolean("Pressed", true);
        });
    }

    public Command setSpeed(double topSpeed, double bottomSpeed) {
        return Commands.parallel(
            setTopSpeed(topSpeed),
            setBottomSpeed(bottomSpeed)
        );
    }


    public Command setPowerZeroCommand() {
        return Commands.runOnce(() -> {
            topShooter.setControl(m_brake);
            bottomShooter.setControl(m_brake);
            // bottomShooter.setControl(m_bottomDutyCycleRequest.withOutput(0));
            SmartDashboard.putBoolean("Pressed", false);
        });
    }

    public void setPowerZero() {
        topShooter.setControl(m_brake);
        bottomShooter.setControl(m_brake);
        // bottomShooter.setControl(m_bottomDutyCycleRequest.withOutput(0));
        SmartDashboard.putBoolean("Pressed", false);

    }

    public void reportToSmartDashboard(LOG_LEVEL logLevel) {}

    public void initShuffleboard(LOG_LEVEL logLevel) {
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        tab.addNumber("Top Velocity", ()-> topShooter.getVelocity().getValueAsDouble());
        tab.addNumber("Bottom Velocity", ()-> bottomShooter.getVelocity().getValueAsDouble());

    }

}
