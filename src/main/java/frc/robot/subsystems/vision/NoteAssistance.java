package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Reportable;
import frc.robot.util.NerdyMath;

public class NoteAssistance implements Reportable{
    private Limelight limelight;
    private String name;

    private PIDController areaController;
    private PIDController txController;
    private PIDController skewController;

    private GenericEntry targetFound;
    private GenericEntry currentArea;
    private GenericEntry currentTX;
    private GenericEntry currentSkew;
    private GenericEntry forwardSpeed;
    private GenericEntry sidewaysSpeed;
    private GenericEntry angularSpeed;

    double[] speeds = {0.0, 0.0, 0.0};

    public NoteAssistance(String name) {
        this.name = name;

        //TODO set pid constants to preferences for easy tuning
        areaController = new PIDController(0, 0, 0);
        txController = new PIDController(0, 0, 0);
        skewController = new PIDController(0, 0, 0);

        try {
            limelight = new Limelight(name);
        } catch (Exception e) {
            limelight = null;
        }
    }

    public void pidTuning_test() {
        VisionConstants.kPNoteForward.loadPreferences();
        VisionConstants.kINoteForward.loadPreferences();
        VisionConstants.kDNoteForward.loadPreferences();
        VisionConstants.kPNoteSide.loadPreferences();
        VisionConstants.kINoteSide.loadPreferences();
        VisionConstants.kDNoteSide.loadPreferences();
        VisionConstants.kPNoteAngle.loadPreferences();
        VisionConstants.kINoteAngle.loadPreferences();
        VisionConstants.kDNoteAngle.loadPreferences();
        
        areaController = new PIDController(VisionConstants.kPNoteForward.get(), VisionConstants.kINoteForward.get(), VisionConstants.kDNoteForward.get());
        txController = new PIDController(VisionConstants.kPNoteSide.get(), VisionConstants.kINoteSide.get(), VisionConstants.kDNoteSide.get());
        skewController = new PIDController(VisionConstants.kPNoteAngle.get(), VisionConstants.kINoteAngle.get(), VisionConstants.kDNoteAngle.get());
    }

    public void speedToNote(double targetArea, double targetTX, double targetSkew) {
        if(limelight == null) return;

        limelight.setPipeline(VisionConstants.kNotePipeline);
        boolean hasTarget = limelight.hasValidTarget();
        targetFound.setBoolean(hasTarget);
        if(!hasTarget) return;

        double area = limelight.getArea_avg();
        currentArea.setDouble(area);
        double tx = limelight.getXAngle_avg();
        currentTX.setDouble(tx);
        double skew = limelight.getSkew();
        currentSkew.setDouble(skew);

        speeds[0] = -1 * areaController.calculate(area, targetArea);
        speeds[1]= -1 * txController.calculate(tx, targetTX);
        speeds[2] = 1 * skewController.calculate(skew, targetSkew);

        forwardSpeed.setDouble(speeds[0]);
        sidewaysSpeed.setDouble(speeds[1]);
        angularSpeed.setDouble(speeds[2]);

        speeds[0] = NerdyMath.deadband(speeds[0], -0.5, 0.5);
        speeds[1] = NerdyMath.deadband(speeds[1], -0.5, 0.5);
        speeds[2] = NerdyMath.deadband(speeds[2], -0.5, 0.5);
    }

    public double getForwardSpeed() { return speeds[0]; }
    public double getSidewaysSpeed() { return speeds[1]; }
    public double getAngularSpeed() { return speeds[2]; }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reportToSmartDashboard'");
    }

    public void initShuffleboard(LOG_LEVEL priority) {
        if (priority == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab(name);

        //the lack of "break;"'s is intentional
        switch (priority) {
            case ALL:

            case MEDIUM:
                currentArea = tab.add("Area", 0)
                .withPosition(2, 0)
                .withSize(2, 1)
                .getEntry();
                
                currentTX = tab.add("TX", 0)
                .withPosition(2, 1)
                .withSize(2, 1)
                .getEntry();

                currentSkew = tab.add("Skew", 0)
                .withPosition(2, 2)
                .withSize(2, 1)
                .getEntry();

            case MINIMAL:   
                forwardSpeed = tab.add("Forward Speed", 0)
                .withPosition(0, 0)
                .withSize(2, 1)
                .getEntry();
                
                sidewaysSpeed = tab.add("Sideways Speed", 0)
                .withPosition(0, 1)
                .withSize(2, 1)
                .getEntry();

                angularSpeed = tab.add("Angular Speed", 0)
                .withPosition(0, 2)
                .withSize(2, 1)
                .getEntry();
                
                targetFound = tab.add("Target Found", false)
                .withPosition(0, 3)
                .withSize(2, 1)
                .getEntry();

            case OFF:
                break;
            
        }
    }
    
}
