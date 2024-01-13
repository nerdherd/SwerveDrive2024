package frc.robot.subsystems.vision.farfuture;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Reportable;

public class Citron implements Reportable{
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private AprilTagFieldLayout layout;
    public String name;
    private String ip;
    private double lastTimestamp = 0.0;

    //Takes in photonvision camera name
    public Citron(String cameraName, String ip) {
        name = cameraName;
        this.ip = ip;
        try {
            camera = new PhotonCamera(cameraName);
            SmartDashboard.putBoolean("Limelight inited", true);
        } catch (Exception e) {
            camera = null;
            SmartDashboard.putBoolean("Limelight inited", false);
        }

        try {
            
            layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2024-crescendo.json");
            estimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.kCameraToRobot);
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            SmartDashboard.putBoolean("Layout Found", true);
        } catch (Exception e) {
            layout = null;
            estimator = null;
            SmartDashboard.putBoolean("Layout Found", false);
        }
    }

    //returns the pose2d of 
    public Pose3d usePlasmaBall() {
        if (camera == null) return null;
        
        camera.setPipelineIndex(0);
        PhotonPipelineResult results = camera.getLatestResult();

        //store time for later use by pose estimator
        lastTimestamp = results.getTimestampSeconds();

        if(!results.hasTargets()) {
            SmartDashboard.putBoolean("Has Target", false);
            return null;
        }
        PhotonTrackedTarget target = results.getBestTarget();

        int index = target.getFiducialId() - 1; // subtract 1 because numbered 1-16
        if(index < 0) {
            SmartDashboard.putBoolean("index legit", false);
            return null;
        }
        Pose3d targetPose = layout.getTags().get(index).pose;

        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Pose3d cameraPose = targetPose.transformBy(cameraToTarget.inverse());

        //account for camera not being in the center of the robot
        return cameraPose.transformBy(VisionConstants.kCameraToRobot);
    }

    public Pose3d usePlantFood() {
        if(estimator == null) {
            // SmartDashboard.putBoolean(name + ":Estimator null", true);
            return null;
        }
        if(camera == null) {
            // SmartDashboard.putBoolean(name + ":Camera null", true);
            return null;
        }

        Optional<EstimatedRobotPose> estimatedPose = estimator.update();

        PhotonPipelineResult result = camera.getLatestResult();
        if(!result.hasTargets()) {
            SmartDashboard.putBoolean(name + ":No targets", true);
            return null;
        }
        double latestTimestamp = result.getTimestampSeconds();
        // boolean substantialDifference = Math.abs(latestTimestamp - lastTimestamp) > 1e-5;
        // if(substantialDifference) lastTimestamp = latestTimestamp;

        if(lastTimestamp > 0 && Math.abs(lastTimestamp - latestTimestamp) < 1e-6) {
            SmartDashboard.putBoolean(name + ":Time bad", true);
            return null;
        }

        if(layout.getTagPose(result.getBestTarget().getFiducialId()).isEmpty()) {
            SmartDashboard.putBoolean(name + ":No tag pose", true);
            return null;
        }

        if(result.getBestTarget().getPoseAmbiguity() == -1 && result.getBestTarget().getPoseAmbiguity() < 10) {
            SmartDashboard.putNumber(name + ":Pose ambiguity", result.getBestTarget().getPoseAmbiguity());
            SmartDashboard.putBoolean(name + ":Bad ambiguity", true);
            return null;
        }

        lastTimestamp = latestTimestamp;

        if(estimatedPose.isEmpty()) {
            SmartDashboard.putBoolean(name + ":Has Target", false);
            return null;
        }
        SmartDashboard.putBoolean(name + ":Has Target", true);
        return estimatedPose.get().estimatedPose;
    }

    public double getChargeTime() {
        return lastTimestamp;
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
        ShuffleboardTab tab = Shuffleboard.getTab(name);

        switch (priority) {
            case ALL:

            case MEDIUM:

            case MINIMAL:   
                tab.addCamera(name + ": Stream", name, ip);

                // tab.addString("Robot Pose", currentPose.toString());

            case OFF:
                break;
            
        }
    }
}
