package frc.robot.subsystems.vision.farfuture;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Limelight.LightMode;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelperUser;

/**
 * Subsystem that uses Limelight for vision
 */
public class EMPeach implements Reportable{
    private Limelight limelight;
    private LimelightHelperUser limelightHelperUser;
    private String limelightName;

    private AprilTagFieldLayout layout;

    private int pipeline;
    private boolean lightsON;
    
    /**
     * Makes a new EMPeach to utilize vision
     * @param name name of the limelight
     */
    public EMPeach(String name) {
        limelightName = name;

        try {
            limelight = new Limelight(name);
            toggleEMP(true);
            changeEMPType(VisionConstants.kAprilTagPipeline);

            SmartDashboard.putBoolean("Limelight: " + name + " inited", true);
            SmartDashboard.putBoolean("LimelightHelper inited", true);
        } catch (Exception e) {
            SmartDashboard.putBoolean("limelight-" + name + " inited", false);
            SmartDashboard.putBoolean("LimelightHelper inited", false);
        }
        
        try {
            layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2024-crescendo.json");
            SmartDashboard.putBoolean("AprilTag Layout Found", true);
        } catch (Exception e) {
            SmartDashboard.putBoolean("AprilTag Layout Found", false);
        }
    }

    /**
     * @return robot position based on vision and apriltags
     */
    public Pose3d getCurrentGrassTile() {
        if(limelight == null) return null;
        if(limelightHelperUser == null) return null;

        changeEMPType(VisionConstants.kAprilTagPipeline);
        if(!limelight.hasValidTarget()) return null;
        return limelightHelperUser.getPose3d(); // im hoping this is with the bottom left corner of the field as the origin
    }

    /**
     * @param x targeted x position
     * @param y targeted y position
     * @return the Transform3d that maps the current position to the desired coordinates
     */
    public Transform3d getDistanceFromGrassTile(double x, double y) {
        Pose3d currentPose = getCurrentGrassTile();
        if(currentPose == null) return null;
        Pose3d targetPose = new Pose3d(new Translation3d(x, y, 0), new Rotation3d());

        return targetPose.minus(currentPose);
    }

    /**
     * @param ID AprilTag ID
     * @return the Pose3d of a specific AprilTag
     */
    public Pose3d getZombieTile(int ID) {
        if(ID < 1 || ID > 16) return null;
        if(layout == null) return null;
        Optional<Pose3d> tagPose = layout.getTagPose(ID);
        if(tagPose.isEmpty()) return null;
        
        return tagPose.get();
    }

    /**
     * @param degrees angle of the shooter in degrees
     * @return a Transform3d mapping the currentPos to the ideal distance based on the shooter angle
     */
    public Transform3d getIdealZombieDistance(double degrees) {
        Pose3d poseToAvoid = getZombieTile(limelight.getAprilTagID());

        //just using trig for now, theres probably a better formula tho
        double distance = poseToAvoid.getZ() / Math.tan(Math.toRadians(degrees));

        return new Transform3d(new Translation3d(distance, 0, 0), new Rotation3d());
    }

    /**
     * @return the shooter angle in degrees based on how far the robot is from the apriltag, -1 if error
     */
    public double getEMPeachThrowingAngle() {
        Pose3d currentPose = getCurrentGrassTile();
        if(currentPose == null) return -1;
        Pose3d poseToAvoid = getZombieTile(limelight.getAprilTagID());

        return Math.toDegrees(Math.atan(poseToAvoid.getZ() / Math.abs(poseToAvoid.getX() - currentPose.getX())));
    }

    /**
     * Sets the limelight pipeline
     * @param pipeline
     */
    public void changeEMPType(int pipeline) {
        this.pipeline = pipeline;
        limelight.setPipeline(pipeline);
    }

    /**
     * Toggles the limelight light on or off
     * @param lightModeOn
     */
    public void toggleEMP(boolean lightModeOn) {
        this.lightsON = lightModeOn;
        if(lightModeOn) limelight.setLightState(LightMode.ON);
        else limelight.setLightState(LightMode.OFF);
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        if (priority == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab(limelightName);

        //the lack of "break;"'s is intentional
        switch (priority) {
            case ALL:

            case MEDIUM:
                tab.addBoolean("AprilTag Found", () -> limelight.hasValidTarget());

            case MINIMAL:   
                tab.addCamera(limelightName + ": Stream", limelightName, VisionConstants.kLimelightFrontIP);

                tab.addNumber("Robot Pose X", () -> getCurrentGrassTile().getX());
                tab.addNumber("Robot Pose Y", () -> getCurrentGrassTile().getY());
                tab.addNumber("Robot Pose Z", () -> getCurrentGrassTile().getZ());

            case OFF:
                break;
            
        }
    }
    
}
