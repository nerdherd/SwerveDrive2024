package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        Pose2d firstNotePose;// = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);
        Pose2d secondNotePose;
        Pose2d thirdNotePose;
        Pose2d fourthNotePose;

        int aimTargetApriltagID;

        if(RobotContainer.IsRedSide())
        {
            firstNotePose = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0)));
            secondNotePose = new Pose2d(); // todo
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
            Commands.runOnce(()->swerve.resetInitPoseByVision()),
            Commands.waitSeconds(2), // debug time

            PathCurrentToDest(firstNotePose, 1.5, 1.5, 360.0, 540.0, 0.0, 2.0), // Pickup 1
            //AutoBuilder.followPath((pathGroup.get(0))), // Pickup 1
            Commands.waitSeconds(4),

            PathCurrentToDest(secondNotePose, 1.5, 1.5, 360.0, 540.0, 0.0, 2.0), // Pickup 1
            //AutoBuilder.followPath((pathGroup.get(1))), // Pickup 2
            notething.driveToNoteCommand(swerve, 5),
            Commands.waitSeconds(4),

            AutoBuilder.followPath((pathGroup.get(2))), // Pickup 3
            notething.driveToNoteCommand(swerve, 5),
            Commands.waitSeconds(4),

            AutoBuilder.followPath((pathGroup.get(3))), // Pos Mid
            Commands.waitSeconds(5),

            AutoBuilder.followPath((pathGroup.get(4))), // Back Shoot
            Commands.waitSeconds(5),

            AutoBuilder.followPath((pathGroup.get(5))), // Pos Mid
            Commands.waitSeconds(5),

            AutoBuilder.followPath((pathGroup.get(6))), // Back Shoot
            Commands.waitSeconds(5),

            AutoBuilder.followPath((pathGroup.get(7))), // Back Shoot
            Commands.waitSeconds(3)
            // tagAssist.TagDriving(swerve, 1.6, -2.77, 26, 7)
        );
    }

    public Command PathCurrentToDest(Pose2d destPose, 
        double maxVelocity, double MaxAcceleration,
        double maxAngleVelocity, double MaxAngleAcceleration,
        double goalEndVelocity, double rotationDelayDistance)
    {
        return AutoBuilder.pathfindToPose(
            destPose,
            new PathConstraints(
                maxVelocity, MaxAcceleration, 
                Units.degreesToRadians(maxAngleVelocity), Units.degreesToRadians(MaxAngleAcceleration)
            ), 
            goalEndVelocity, 
            rotationDelayDistance
        );
    }
}
