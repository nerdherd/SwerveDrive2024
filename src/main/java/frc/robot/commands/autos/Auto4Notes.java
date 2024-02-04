package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
//import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.NoteAssistance;
import frc.robot.subsystems.vision.farfuture.DriverAssist;

public class Auto4Notes extends SequentialCommandGroup {
    public Auto4Notes(SwerveDrivetrain swerve, String autoPath, NoteAssistance notething, DriverAssist tagAssist) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startPose2d = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        // Blue side
        Pose2d firstPickPose = GetEndPointInPath(pathGroup.get(0));//new Pose2d(2.23,6.63, new Rotation2d(Units.degreesToRadians(20))); // todo. testing it
        Pose2d secondPickPose = GetEndPointInPath(pathGroup.get(1));//new Pose2d(2.23,6.63, new Rotation2d(Units.degreesToRadians(20)));

        int aimTargetApriltagID;

        if(RobotContainer.IsRedSide())
        {
            firstPickPose = GeometryUtil.flipFieldPose(firstPickPose);
            secondPickPose = GeometryUtil.flipFieldPose(secondPickPose);
            aimTargetApriltagID = 4;
        }
        else // Blue side
        {
            aimTargetApriltagID = 7;
        }

        addCommands(
            //Commands.none()
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(()->tagAssist.resetInitPoseByVision(swerve, startPose2d, aimTargetApriltagID) ),
            Commands.waitSeconds(2), // debug time

            // Pickup 1
            PathCurrentToDest(firstPickPose, 0.45, 0.45, 360.0, 540.0), 
            //Commands.run(() -> swerve.drive(0,0,0)),
            //AutoBuilder.followPath((pathGroup.get(0))), 
            // notething.driveToNoteCommand(swerve, 4.5, 10, 40, firstPickPose),
            // Commands.waitSeconds(1),
            // tagAssist.aimToApriltagCommand(swerve, aimTargetApriltagID, 4, 20, secondPickPose, true),
            // skip this aim&shoot, do it at next location
            Commands.waitSeconds(4),

            // Pickup 2
            FindPathThenFollowPlanned(pathGroup.get(1), 0.45, 0.45, 360.0, 540.0), // because the pose was changed in the previous step
            //AutoBuilder.followPath((pathGroup.get(1))), 
            // tagAssist.aimToApriltagCommand(swerve, aimTargetApriltagID, 4, 20, secondPickPose, true),
            // notething.driveToNoteCommand(swerve, 4.5, 10, 40, secondPickPose),
            // tagAssist.aimToApriltagCommand(swerve, aimTargetApriltagID, 4, 20, secondPickPose, true),
            // shoot twice 
            // Commands.waitSeconds(4),

            // // Pickup 3
            // AutoBuilder.followPath((pathGroup.get(2))), 
            // //notething.driveToNoteCommand(swerve, 4.5, 10, 40, thirdPickPose),
            // //tagAssist.aimToApriltagCommand(swerve, aimTargetApriltagID, 4, 20, thirdPickPose, true),
            // Commands.waitSeconds(4),

            // AutoBuilder.followPath((pathGroup.get(3))), // Pos Mid
            // Commands.waitSeconds(5),

            // AutoBuilder.followPath((pathGroup.get(4))), // Back Shoot
            // Commands.waitSeconds(5),

            // AutoBuilder.followPath((pathGroup.get(5))), // Pos Mid
            // Commands.waitSeconds(5),

            // AutoBuilder.followPath((pathGroup.get(6))), // Back Shoot
            // Commands.waitSeconds(5),

            // AutoBuilder.followPath((pathGroup.get(7))), // back to init spot, testing code
            // Commands.waitSeconds(3),
            // tagAssist.TagDriving(swerve, 1.6, -2.77, 26, 7),
            Commands.none()
        );
    }

    public Auto4Notes(SwerveDrivetrain swerve, String autoPath, NoteAssistance notething, DriverAssist tagAssist, boolean usingPose) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startPose2d = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d firstNotePose;// = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);
        Pose2d secondNotePose;
        Pose2d thirdNotePose;
        Pose2d fourthNotePose;

        int aimTargetApriltagID;

        if(RobotContainer.IsRedSide())
        {
            firstNotePose = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0)));
            secondNotePose = new Pose2d(); // todo
            thirdNotePose = new Pose2d();
            aimTargetApriltagID = 4;
        }
        else // Blue side
        {
            firstNotePose = new Pose2d(2.23,6.63, new Rotation2d(Units.degreesToRadians(20)));
            secondNotePose = new Pose2d(2.0,5.53, new Rotation2d(Units.degreesToRadians(0)));
            thirdNotePose = new Pose2d(2.14,4.43, new Rotation2d(Units.degreesToRadians(-20)));
            fourthNotePose = new Pose2d(2.14,2.43, new Rotation2d(Units.degreesToRadians(-20))); // testing position. todo
            aimTargetApriltagID = 7;
        }

        addCommands(
            //Commands.none()
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(()->tagAssist.resetInitPoseByVision(swerve, startPose2d, aimTargetApriltagID)),
            Commands.waitSeconds(2), // debug time

            PathCurrentToDest(firstNotePose, 1.5, 1.5, 360.0, 540.0), // Pickup 1
            //AutoBuilder.followPath((pathGroup.get(0))), // Pickup 1
            Commands.waitSeconds(4),

            PathCurrentToDest(secondNotePose, 1.5, 1.5, 360.0, 540.0), // Pickup 2
            //AutoBuilder.followPath((pathGroup.get(1))), // Pickup 2
            //notething.driveToNoteCommand(swerve, 5),
            Commands.waitSeconds(4),

            PathCurrentToDest(thirdNotePose, 1.5, 1.5, 360.0, 540.0), // Pickup 3
            //AutoBuilder.followPath((pathGroup.get())), // Pickup 3
            //notething.driveToNoteCommand(swerve, 5),
            Commands.waitSeconds(4),

            // AutoBuilder.followPath((pathGroup.get(3))), // Pos Mid
            // Commands.waitSeconds(5),

            // AutoBuilder.followPath((pathGroup.get(4))), // Back Shoot
            // Commands.waitSeconds(5),

            // AutoBuilder.followPath((pathGroup.get(5))), // Pos Mid
            // Commands.waitSeconds(5),

            // AutoBuilder.followPath((pathGroup.get(6))), // Back Shoot
            // Commands.waitSeconds(5),

            // AutoBuilder.followPath((pathGroup.get(7))), // back to init spot, testing code
            // Commands.waitSeconds(3)
            // tagAssist.TagDriving(swerve, 1.6, -2.77, 26, 7)
            Commands.none()
        );
    }

    public Command PathCurrentToDest(Pose2d destPose, 
        double maxVelocity, double MaxAcceleration,
        double maxAngleVelocity, double MaxAngleAcceleration)
    {
        return AutoBuilder.pathfindToPose(
            destPose,
            new PathConstraints(
                maxVelocity, MaxAcceleration, 
                Units.degreesToRadians(maxAngleVelocity), Units.degreesToRadians(MaxAngleAcceleration)
            )
        );
    }

    public Command FindPathThenFollowPlanned(PathPlannerPath goalPath, 
        double maxVelocity, double MaxAcceleration,
        double maxAngleVelocity, double MaxAngleAcceleration)
    {
        return AutoBuilder.pathfindThenFollowPath(
            goalPath,
            new PathConstraints(
                maxVelocity, MaxAcceleration, 
                Units.degreesToRadians(maxAngleVelocity), Units.degreesToRadians(MaxAngleAcceleration)
            )
        );
        
    }

    public static Pose2d GetEndPointInPath(PathPlannerPath path)
    {
        PathPoint tail  = path.getPoint(path.numPoints()-1);
        Translation2d pos = tail.position;
        double rad = tail.rotationTarget.getTarget().getRadians();
        return new Pose2d(pos, new Rotation2d(rad));
    } 
}