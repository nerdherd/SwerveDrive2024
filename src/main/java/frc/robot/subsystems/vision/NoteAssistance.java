package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.NerdyMath;

public class NoteAssistance implements Reportable{
    private Limelight limelight;
    private String name;

    private PIDController areaController;
    private PIDController txController;

    private GenericEntry targetFound;
    private GenericEntry currentArea;
    private GenericEntry currentTX;
    private GenericEntry currentTY;

    private GenericEntry forwardSpeed;
    private GenericEntry sidewaysSpeed;

    double[] speeds = {0.0, 0.0};

    public NoteAssistance(String name) {
        this.name = name;
        ShuffleboardTab tab = Shuffleboard.getTab(name);

        //TODO set pid constants to preferences for easy tuning
        areaController = new PIDController(0.34, 0, 0);// todo, tuning pls!!!
        txController = new PIDController(0.08, 0, 0.006);// todo, tuning pls!!!

        try {
            limelight = new Limelight(name);
            tab.add(name + " inited", true);
            limelight.setPipeline(VisionConstants.kNotePipeline);

        } catch (Exception e) {
            limelight = null;
            tab.add(name + " inited", false);
        }
    }

    public void pidTuning_test() {
        VisionConstants.kPNoteForward.loadPreferences();
        VisionConstants.kINoteForward.loadPreferences();
        VisionConstants.kDNoteForward.loadPreferences();
        VisionConstants.kPNoteSide.loadPreferences();
        VisionConstants.kINoteSide.loadPreferences();
        VisionConstants.kDNoteSide.loadPreferences();
        // VisionConstants.kPNoteAngle.loadPreferences();
        // VisionConstants.kINoteAngle.loadPreferences();
        // VisionConstants.kDNoteAngle.loadPreferences();
        
        areaController = new PIDController(VisionConstants.kPNoteForward.get(), VisionConstants.kINoteForward.get(), VisionConstants.kDNoteForward.get());
        txController = new PIDController(VisionConstants.kPNoteSide.get(), VisionConstants.kINoteSide.get(), VisionConstants.kDNoteSide.get());
        //skewController = new PIDController(VisionConstants.kPNoteAngle.get(), VisionConstants.kINoteAngle.get(), VisionConstants.kDNoteAngle.get());
    }

    public void reset() {
        limelight.resetLists();
    }

    public void speedToNote(double targetArea, double targetTX, double targetTY) {
        if(limelight == null) return;
        
        boolean hasTarget = limelight.hasValidTarget();
        if(targetFound != null)
            targetFound.setBoolean(hasTarget);
        if(hasTarget) 
        {
            double area = limelight.getArea_avg();
            if(currentArea != null)
                currentArea.setDouble(area);
                
            double tx = limelight.getXAngle_avg();
            if(currentTX != null)
                currentTX.setDouble(tx);
                
            double ty = limelight.getYAngle_avg();
            if(currentTY != null)
                currentTY.setDouble(ty);

            if( area < 0.5 || area > 5.5 ) // todo, tuning pls!!!
            {
                speeds[0] = speeds[1] = 0; // something is wrong, or filted out by camera dashboard
            } 
            else if( ty < targetTY ) 
            {
                speeds[0] = speeds[1] = 0; // stop it otherwise too close to the notes
            } 
            else if( tx < 6 && tx > -6 && area > 3.7 ) // todo, tuning pls!!!
            {
                speeds[0] = speeds[1] = 0; // good range to get the note. faster than the pid
            } 
            else
            {
                speeds[0] = 1 * areaController.calculate(area, targetArea);
                speeds[1]= 1 * txController.calculate(tx, targetTX);

                speeds[0] = NerdyMath.deadband(speeds[0], -0.2, 0.2); // todo, tuning pls!!!
                speeds[1] = NerdyMath.deadband(speeds[1], -0.35, 0.35);// todo, tuning pls!!!
            }
        }
        else
        {
            speeds[0] = speeds[1] = 0;
        }

        if(forwardSpeed != null)
            forwardSpeed.setDouble(speeds[0]);
        if(sidewaysSpeed != null)
            sidewaysSpeed.setDouble(speeds[1]);
    }

    public void driveToNote(SwerveDrivetrain drivetrain, double targetArea, double targetTX, double targetTY) {
        speedToNote(targetArea, targetTX, targetTY);
        drivetrain.drive(getForwardSpeed(), getSidewaysSpeed(), 0);
    }

    public Command driveToNoteCommand(SwerveDrivetrain drivetrain, double targetArea) {
        return Commands.sequence(
            Commands.runOnce(() -> limelight.resetLists()),
            Commands.run(
                () -> driveToNote(drivetrain, targetArea, 0, 0.1)// todo, tuning pls!!!
            ).until(() -> Math.abs(getForwardSpeed()) <= 0.1 && Math.abs(getSidewaysSpeed()) <= 0.1)// todo, tuning pls!!!
        );
    }

    public double getForwardSpeed() { return speeds[0]; }
    public double getSidewaysSpeed() { return speeds[1]; }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
    }

    public void initShuffleboard(LOG_LEVEL priority) {
        if (priority == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab(name);

        //the lack of "break;"'s is intentional
        switch (priority) {
            case ALL:
       
            try{
                tab.addCamera(name + ": Stream", name, VisionConstants.kLimelightFrontIP);
            }catch(Exception e){};

            case MEDIUM:
                currentArea = tab.add("Area", 0)
                .withPosition(2, 0)
                .withSize(2, 1)
                .getEntry();
                
                currentTX = tab.add("Tx", 0)
                .withPosition(2, 1)
                .withSize(2, 1)
                .getEntry();

                currentTY = tab.add("Ty", 0)
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
                
                targetFound = tab.add("Target Found", false)
                .withPosition(0, 3)
                .withSize(2, 1)
                .getEntry();

            case OFF:
                break;
            
        }
    }
    
}
