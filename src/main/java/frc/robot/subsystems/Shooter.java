package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.Slot1Configs;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter {

    final TalonFX topShooter;
    final TalonFX bottomShooter;

    private double[] topSpeeds = {-0.1, 0.5};
    private double[] bottomSpeeds = {-0.1, 0.5};
    private int index = 0;

    final VoltageOut m_leftVoltageRequest = new VoltageOut(0);
    final VoltageOut m_rightVoltageRequest = new VoltageOut(0);
    final DutyCycleOut m_leftDutyCycleRequest = new DutyCycleOut(0);
    final DutyCycleOut m_rightDutyCyclerequest = new DutyCycleOut(0);

    final VelocityVoltage m_topVelocity = new VelocityVoltage(0);
    final VelocityVoltage m_bottomVelocity = new VelocityVoltage(0);

    public Shooter(){
        topShooter = new TalonFX(ShooterConstants.kTopMotorID, ModuleConstants.kCANivoreName);
        bottomShooter = new TalonFX(ShooterConstants.kBottomMotorID, ModuleConstants.kCANivoreName);
        topShooter.setInverted(false);
        bottomShooter.setControl(new Follower(topShooter.getDeviceID(), false));

        ShooterConstants.kPTopMotor.loadPreferences();
        // ShooterConstants.kITopMotor.loadPreferences();
        // ShooterConstants.kDTopMotor.loadPreferences();
        ShooterConstants.kVTopMotor.loadPreferences();

        ShooterConstants.kPBottomMotor.loadPreferences();
        // ShooterConstants.kIBottomMotor.loadPreferences();
        // ShooterConstants.kDBottomMotor.loadPreferences();
        ShooterConstants.kVBottomMotor.loadPreferences();

        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = ShooterConstants.kVTopMotor.get();
        slot0Configs.kP = ShooterConstants.kPTopMotor.get();
        // slot0Configs.kI = ShooterConstants.kITopMotor.get();
        // slot0Configs.kD = ShooterConstants.kDTopMotor.get();

        topShooter.getConfigurator().apply(slot0Configs, 0.050);

        var slot1Configs = new Slot1Configs();
        slot1Configs.kV = ShooterConstants.kVBottomMotor.get();
        slot1Configs.kP = ShooterConstants.kPBottomMotor.get();
        // slot1Configs.kI = ShooterConstants.kIBottomMotor.get();
        // slot1Configs.kD = ShooterConstants.kDBottomMotor.get();

        bottomShooter.getConfigurator().apply(slot1Configs, 0.050);
 
        topShooter.setControl(m_leftVoltageRequest.withOutput(11.0));
        bottomShooter.setControl(m_rightVoltageRequest.withOutput(11.0));
    }

    public void printSpeeds() {
        SmartDashboard.putNumber("Index", index);
        SmartDashboard.putNumber("Intake", topSpeeds[0]);
        SmartDashboard.putNumber("OuttakeTop", topSpeeds[1]);
        SmartDashboard.putNumber("OuttakeBottom", bottomSpeeds[1]);
    }

    public CommandBase setIndex(int index) {
        return Commands.runOnce(() -> {
            this.index = index;
        });
    }

    public CommandBase setSpeed() {
        return Commands.runOnce(() -> {

            // Percent Ouput
            // topShooter.setControl(m_leftDutyCycleRequest.withOutput(topSpeeds[index]));
            // topShooter.setControl(m_leftDutyCycleRequest.withOutput(bottomSpeeds[index]));

            // Velocity Control
            m_topVelocity.Slot = 0;
            m_bottomVelocity.Slot = 1;

            topShooter.setControl(m_topVelocity.withVelocity(topSpeeds[index]));
            bottomShooter.setControl(m_bottomVelocity.withVelocity(bottomSpeeds[index]));
            SmartDashboard.putBoolean("Pressed", true);
        });
    }

    public CommandBase setPowerZero() {
        return Commands.runOnce(() -> {
            topShooter.setControl(m_leftDutyCycleRequest.withOutput(0));
            topShooter.setControl(m_leftDutyCycleRequest.withOutput(0));
            SmartDashboard.putBoolean("Pressed", false);
        });
    }

    public CommandBase increaseTop() {
        return Commands.runOnce(() -> {
            this.topSpeeds[index] += 0.1;
        });
    }
    public CommandBase increaseBottom() {
        return Commands.runOnce(() -> {
            this.bottomSpeeds[index] += 0.1;
        });
    }

    public CommandBase decreaseTop() {
        return Commands.runOnce(() -> {
            this.topSpeeds[index] -= 0.1;
        });
    }
    public CommandBase decreaseBottom() {
        return Commands.runOnce(() -> {
            this.bottomSpeeds[index] -= 0.1;
        });
    }

    // public void reportToSmartDashboard(){
    //     SmartDashboard.putNumber("Left RPM", topShooter.getSelectedSensorVelocity(0) * 10 / 2048);
    //     SmartDashboard.putNumber("Right RPM", bottomShooter.getSelectedSensorVelocity(0) * 10 / 2048);
    //     SmartDashboard.putNumber("Left Current", topShooter.getSupplyCurrent());
    //     SmartDashboard.putNumber("Right Current", bottomShooter.getSupplyCurrent());
    // }

}
