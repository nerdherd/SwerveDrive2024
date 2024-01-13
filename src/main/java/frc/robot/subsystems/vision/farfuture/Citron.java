package frc.robot.subsystems.vision.farfuture;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Reportable;

/**
 * Subsystem that uses PhotonVision for vision
 */
public class Citron implements Reportable{
    // Library Variables
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private static AprilTagFieldLayout layout;

    // Camera Specific Variables
    public String name;
    private String ip;
    private double lastTimestamp = 0.0;

    // Variables for Logging
    private GenericEntry cameraInited;
    private GenericEntry estimatorInited;
    private GenericEntry robotPose;
    private GenericEntry hasTarget;
    private GenericEntry goodVisionFrequency;

    // Other Variables
    private int visionFrequency;
    private int counter = 0;

    PIDController forwardController = new PIDController(0.3, 0, 0);
    PIDController rotationController = new PIDController(0.02, 0, 0);

    /**
     * Makes a new Citron to utilize vision
     * @param name name of the camera
     * @param ip ip address of the limelight(ex. 10.6.87.99)
     * @param visionFrequency how often the vision pose estimator should be updated(ex. 2 is every 0.5 seconds)
     */
    public Citron(String name, String ip, int visionFrequency) {
        this.name = name;
        this.ip = ip;
        this.visionFrequency = visionFrequency;

        if(layout != null) layout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        
        try {
            camera = new PhotonCamera(name);
            toggleCitronBall(true);
            cameraInited.setBoolean(true);
        } catch (Exception e) {
            camera = null;
            cameraInited.setBoolean(false);
        }

        try {
            poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.kCameraToRobot);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            estimatorInited.setBoolean(true);
        } catch (Exception e) {
            poseEstimator = null;
            estimatorInited.setBoolean(false);
        }
    }

    /**
     * Toggles the camera light on or off
     * @param lightModeOn
     */
    public void toggleCitronBall(boolean lightModeOn) {
        if(lightModeOn) camera.setLED(VisionLEDMode.kOn);
        else camera.setLED(VisionLEDMode.kOff);
    }

    /**
     * Sets the camera pipeline
     * @param pipeline
     */
    public void setCitronVariant(int pipeline) {
        camera.setPipelineIndex(pipeline);
    }

    /**
     * @return the timestamp in seconds of the last time the camera was updated
     */
    public double getPreviousChargeTime() {
        return lastTimestamp;
    }
    
    /**
     * @return robot Pose3d based on PhotonVision Pose Estimator
     */
    public Pose3d getCurrentGrassTile() {
        if(camera == null) return null;
        if(poseEstimator == null) return null;

        setCitronVariant(0);

        PhotonPipelineResult results = camera.getLatestResult();

        if(!results.hasTargets()) {
            hasTarget.setBoolean(false);
            return null;
        }
        hasTarget.setBoolean(true);
        double currentTimestamp = results.getTimestampSeconds();

        if(lastTimestamp > 0 && Math.abs(lastTimestamp - currentTimestamp) < 1e-6) {
            goodVisionFrequency.setBoolean(false);
            return null;
        }
        goodVisionFrequency.setBoolean(true);

        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(results);

        lastTimestamp = currentTimestamp;

        if(estimatedPose.isEmpty()) return null;
        robotPose.setString(estimatedPose.get().estimatedPose.toString());
        return estimatedPose.get().estimatedPose;
    }

    /**
     * @return robot Pose3d based on Camera and AprilTag Pose and transformations
     */
    public Pose3d getCurrentGrassTileAlternative() {
        if(camera == null) return null;

        setCitronVariant(0);
        PhotonPipelineResult results = camera.getLatestResult();
        
        lastTimestamp = results.getTimestampSeconds();

        if(!results.hasTargets()) {
            hasTarget.setBoolean(false);
            return null;
        }
        hasTarget.setBoolean(true);
        PhotonTrackedTarget target = results.getBestTarget();

        Optional<Pose3d> targetPose = layout.getTagPose(target.getFiducialId());
        if(targetPose.isEmpty()) return null;

        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Pose3d cameraPose = targetPose.get().transformBy(cameraToTarget.inverse()).transformBy(VisionConstants.kCameraToRobot);

        robotPose.setString(cameraPose.toString());
        return cameraPose;
    }

    /**
     * Returns non-null every 1/VisionFrequency of a second
     * @return robot Pose3d based on PhotonVision Pose Estimator
     */
    public Pose3d getCitronCurrentPosition() {
        counter++;
        if(counter % visionFrequency != 0) return null;

        Pose3d pose = getCurrentGrassTile();
        if(pose == null) return null;

        return pose;
    }

    /**
     * @param ID AprilTag ID
     * @return the Pose3d of a specific AprilTag
     */
    public Pose3d getZombieTile(int ID) {
        if(ID < 1 || ID > 16) return null;
        Optional<Pose3d> tagPose = layout.getTagPose(ID);
        if(tagPose.isEmpty()) return null;
        
        return tagPose.get();
    }

    /**
     * Goes to the closest AprilTag and stops targetMeters meters away
     * @param targetMeters how far away to stop from the target
     * @return double[] in the form of {forwardSpeed, rotationSpeed}
     */
    public double[] goToZombie(double targetMeters) {
        double[] speeds = {0.0, 0.0};
        if(camera == null) return speeds;

        setCitronVariant(0);
        PhotonPipelineResult results = camera.getLatestResult();

        if(!results.hasTargets()) {
            hasTarget.setBoolean(false);
            return speeds;
        }
        hasTarget.setBoolean(true);

        double range =
            PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.kCameraHeightMeters,
                VisionConstants.kTargetHeightMeters,
                Units.degreesToRadians(VisionConstants.kCameraPitchDegrees),
                Units.degreesToRadians(results.getBestTarget().getPitch()));

        speeds[0] = forwardController.calculate(range, targetMeters);
        speeds[1] = rotationController.calculate(results.getBestTarget().getYaw(), 0);

        return speeds;
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        switch (priority) {
            case ALL:
                
            case MEDIUM:

            case MINIMAL:
                break;
            default:
                break;
        }
    }

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        if (priority == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab(name);

        switch (priority) {
            case ALL:

            case MEDIUM:

            case MINIMAL:   
                cameraInited = tab.add("Camera Inited", false)
                    .withSize(2, 1)
                    .withPosition(6, 0)
                    .getEntry();
                estimatorInited = tab.add("Pose Estimator Inited", false)
                    .withSize(2, 1)
                    .withPosition(6, 1)
                    .getEntry();
                
                hasTarget = tab.add("Has Target", false)
                    .withSize(2, 1)
                    .withPosition(6, 2)
                    .getEntry();
                goodVisionFrequency = tab.add("Good Vision Frequency", false)
                    .withSize(2, 1)
                    .withPosition(6, 3)
                    .getEntry();
                robotPose = tab.add("Robot Pose", "null")
                    .withSize(6, 1)
                    .withPosition(0, 3)
                    .getEntry();

                tab.addCamera(name + ": Stream", name, ip)
                    .withSize(6, 3)
                    .withPosition(0, 0);
                break;
            
            default:
                break;
        }
    }
}
