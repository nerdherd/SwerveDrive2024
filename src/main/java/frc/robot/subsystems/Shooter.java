package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter {

    final TalonFX topShooter;
    final TalonFX bottomShooter;

    private double[] topSpeeds = {-0.1, 0.5};
    private double[] bottomSpeeds = {-0.1, 0.5};
    private int index = 0;

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
        // bottomShooter.setControl(new Follower(topShooter.getDeviceID(), false));

        

        // bottomShooter.getConfigurator().apply(slot1Configs, 0.050);
 
        // topShooter.setControl(m_topVoltageRequest.withOutput(11.0));
        // bottomShooter.setControl(m_bottomVoltageRequest.withOutput(11.0));

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
            DriverStation.reportError("Could not apply top shooter configs, error code:"+statusTop.toString(), null);
        }
        if(!statusBottom.isOK()){
            DriverStation.reportError("Could not apply bottom shooter configs, error code:"+statusBottom.toString(), null);
        }
    }

    public void printSpeeds() {
        SmartDashboard.putNumber("Index", index);
        SmartDashboard.putNumber("Intake", topSpeeds[0]);
        SmartDashboard.putNumber("OuttakeTop", topSpeeds[1]);
        SmartDashboard.putNumber("OuttakeBottom", bottomSpeeds[1]);
    }

    public Command setIndex(int index) {
        return Commands.runOnce(() -> {
            this.index = index;
        });
    }

    public Command setSpeed(double topSpeed, double bottomSpeed) {
        return Commands.runOnce(() -> {

            // Percent Ouput
            // topShooter.setControl(m_leftDutyCycleRequest.withOutput(topSpeeds[index]));
            // topShooter.setControl(m_leftDutyCycleRequest.withOutput(bottomSpeeds[index]));

            // Velocity Control
            m_topVelocity.Slot = 0;
            m_bottomVelocity.Slot = 0;
            // m_bottomVelocity.Slot = 1;

            topShooter.setControl(m_topVelocity.withVelocity(topSpeed));
            bottomShooter.setControl(m_bottomVelocity.withVelocity(bottomSpeed));
            SmartDashboard.putBoolean("Pressed", true);
        });
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

    public Command increaseTop() {
        return Commands.runOnce(() -> {
            this.topSpeeds[index] += 0.1;
        });
    }
    public Command increaseBottom() {
        return Commands.runOnce(() -> {
            this.bottomSpeeds[index] += 0.1;
        });
    }

    public Command decreaseTop() {
        return Commands.runOnce(() -> {
            this.topSpeeds[index] -= 0.1;
        });
    }
    public Command decreaseBottom() {
        return Commands.runOnce(() -> {
            this.bottomSpeeds[index] -= 0.1;
        });
    }

    public void reportToSmartDashboard(){
        // SmartDashboard.putNumber("Left RPM", topShooter.getSelectedSensorVelocity(0) * 10 / 2048);
        // SmartDashboard.putNumber("Right RPM", bottomShooter.getSelectedSensorVelocity(0) * 10 / 2048);
        // SmartDashboard.putNumber("Left Current", topShooter.getSupplyCurrent());
        // SmartDashboard.putNumber("Right Current", bottomShooter.getSupplyCurrent());

    }

    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        tab.addNumber("Top Velocity", ()-> topShooter.getVelocity().getValueAsDouble());
        tab.addNumber("Bottom Velocity", ()-> bottomShooter.getVelocity().getValueAsDouble());

    }

}
