package frc.robot.subsystems;

import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter {
    final TalonFX leftShooter;
    final TalonFX rightShooter;

    private double[] speeds = {-0.1, 0.5, 0.7};
    private double[] speeds2 = {-0.1, 0.5, 0.7};
    private int index = 0;

    final VoltageOut m_leftVoltageRequest = new VoltageOut(0);
    final VoltageOut m_rightVoltageRequest = new VoltageOut(0);
    final DutyCycleOut m_leftDutyCycleRequest = new DutyCycleOut(0);
    final DutyCycleOut m_rightDutyCyclerequest = new DutyCycleOut(0);


    public Shooter(){
        leftShooter = new TalonFX(ShooterConstants.kTopMotorID, ModuleConstants.kCANivoreName);
        rightShooter = new TalonFX(ShooterConstants.kBottomMotorID, ModuleConstants.kCANivoreName);
        
        // leftShooter.configVoltageCompSaturation(11);
        // rightShooter.configVoltageCompSaturation(11);
        // leftShooter.enableVoltageCompensation(true);
        // rightShooter.enableVoltageCompensation(false);
        
        leftShooter.setControl(m_leftVoltageRequest.withOutput(11.0));
        rightShooter.setControl(m_rightVoltageRequest.withOutput(11.0));

        leftShooter.setInverted(false);
        rightShooter.setControl(new Follower(leftShooter.getDeviceID(), false));

        SmartDashboard.putNumber("Power", 0);
    }

    public void printSpeeds() {
        SmartDashboard.putNumber("Index", index);
        SmartDashboard.putNumber("IntakeTop", speeds[0]);
        SmartDashboard.putNumber("OuttakeTop1", speeds[1]);
        SmartDashboard.putNumber("OuttakeTop2", speeds[2]);
        SmartDashboard.putNumber("IntakeBottom", speeds2[0]);
        SmartDashboard.putNumber("OuttakeBottom1", speeds2[1]);
        SmartDashboard.putNumber("OuttakeBottom2", speeds2[2]);
    }

    public CommandBase setIndex(int index) {
        return Commands.runOnce(() -> {
            this.index = index;
        });
    }

    public CommandBase setSpeed() {
        return Commands.runOnce(() -> {
            leftShooter.setControl(m_leftDutyCycleRequest.withOutput(speeds[index]));
            leftShooter.setControl(m_leftDutyCycleRequest.withOutput(speeds2[index]));
            SmartDashboard.putBoolean("Pressed", true);
        });
    }

    public CommandBase setPowerZero() {
        return Commands.runOnce(() -> {
            leftShooter.setControl(m_leftDutyCycleRequest.withOutput(0));
            leftShooter.setControl(m_leftDutyCycleRequest.withOutput(0));
            SmartDashboard.putBoolean("Pressed", true);
        });
    }

    public CommandBase increaseTop() {
        return Commands.runOnce(() -> {
            this.speeds[index] += 0.1;
        });
    }
    public CommandBase increaseBottom() {
        return Commands.runOnce(() -> {
            this.speeds2[index] += 0.1;
        });
    }

    public CommandBase decreaseTop() {
        return Commands.runOnce(() -> {
            this.speeds[index] -= 0.1;
        });
    }
    public CommandBase decreaseBottom() {
        return Commands.runOnce(() -> {
            this.speeds2[index] -= 0.1;
        });
    }

    // public void reportToSmartDashboard(){
    //     SmartDashboard.putNumber("Left RPM", leftShooter.getSelectedSensorVelocity(0) * 10 / 2048);
    //     SmartDashboard.putNumber("Right RPM", rightShooter.getSelectedSensorVelocity(0) * 10 / 2048);
    //     SmartDashboard.putNumber("Left Current", leftShooter.getSupplyCurrent());
    //     SmartDashboard.putNumber("Right Current", rightShooter.getSupplyCurrent());
    // }

}
