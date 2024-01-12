package frc.robot.subsystems.vision.farfuture;

import java.util.Optional;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

public class Citron {
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private AprilTagFieldLayout layout;
    private double lastTimestamp = 0.0;

    //Takes in photonvision camera name
    public Citron(String cameraName) {
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
        if(estimator == null) SmartDashboard.putBoolean("Estimator null", true);
        if(camera == null) SmartDashboard.putBoolean("Camera null", true);

        Optional<EstimatedRobotPose> estimatedPose = estimator.update();

        PhotonPipelineResult result = camera.getLatestResult();
        if(!result.hasTargets()) SmartDashboard.putBoolean("No targets", true);
        double latestTimestamp = result.getTimestampSeconds();
        // boolean substantialDifference = Math.abs(latestTimestamp - lastTimestamp) > 1e-5;
        // if(substantialDifference) lastTimestamp = latestTimestamp;

        if(lastTimestamp > 0 && Math.abs(lastTimestamp - latestTimestamp) < 1e-6) SmartDashboard.putBoolean("Time bad", true);

        if(result.getBestTarget().getPoseAmbiguity() == -1 && result.getBestTarget().getPoseAmbiguity() < 10) {
            SmartDashboard.putNumber("Pose ambiguity", result.getBestTarget().getPoseAmbiguity());
            SmartDashboard.putBoolean("Bad ambiguity", true);
        }
        if(layout.getTagPose(result.getBestTarget().getFiducialId()).isEmpty()) SmartDashboard.putBoolean("No tag pose", true);

        lastTimestamp = latestTimestamp;

        if(estimatedPose.isEmpty()) {
            SmartDashboard.putBoolean("Has Target", false);
            return null;
        }
        SmartDashboard.putBoolean("Has Target", true);
        return estimatedPose.get().estimatedPose;
    }

    public double getChargeTime() {
        return lastTimestamp;
    }
}
