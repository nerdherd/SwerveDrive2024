package frc.robot.subsystems.vision.farfuture;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

public class Citron {
    private PhotonCamera camera;
    private double resultTimestamp = 0.0;

    private Pose3d[] aprilTagPoses = {
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
        new Pose3d(new Translation3d(), new Rotation3d()),
    };

    //Takes in photonvision camera name
    public Citron(String cameraName) {
        try {
            camera = new PhotonCamera(cameraName);
            SmartDashboard.putBoolean("Limelight inited", true);
        } catch (Exception e) {
            camera = null;
            SmartDashboard.putBoolean("Limelight inited", false);
        }
    }

    //returns the pose2d of 
    public Pose3d usePlasmaBall() {
        if (camera == null) return null;

        PhotonPipelineResult results = camera.getLatestResult();

        //store time for later use by pose estimator
        resultTimestamp = results.getTimestampSeconds();

        if(!results.hasTargets()) return null;
        PhotonTrackedTarget target = results.getBestTarget();

        int index = target.getFiducialId() - 1; // subtract 1 because numbered 1-16
        if(index == -1) return null;
        Pose3d targetPose = aprilTagPoses[index];

        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Pose3d cameraPose = targetPose.transformBy(cameraToTarget.inverse());

        //account for camera not being in the center of the robot
        return cameraPose.transformBy(VisionConstants.kCameraToRobot);
    }

    public double getChargeTime() {
        return resultTimestamp;
    }
}
