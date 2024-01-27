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

        int aimTargetApriltagID;

        if(RobotContainer.IsRedSide())
        {
            firstNotePose = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0)));
            aimTargetApriltagID = 4;
        }
        else
        {
            firstNotePose = new Pose2d(2.23,6.63, new Rotation2d(Units.degreesToRadians(20)));
            aimTargetApriltagID = 7;
        }

        addCommands(
            //Commands.none()
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(()->swerve.resetInitPoseByVision()),
            Commands.waitSeconds(2), // debug time

            Path1stCom(firstNotePose), // Pickup 1

            //AutoBuilder.followPath((pathGroup.get(0))), // Pickup 1
            Commands.waitSeconds(4),

            AutoBuilder.followPath((pathGroup.get(1))), // Pickup 2
            notething.driveToNoteCommand(swerve, 5, 1.5),
            Commands.waitSeconds(4),

            AutoBuilder.followPath((pathGroup.get(2))), // Pickup 3
            notething.driveToNoteCommand(swerve, 5, 1.5),
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

    public Command Path1stCom(Pose2d destPose)
    {
        return AutoBuilder.pathfindToPose(
            destPose,//new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
            new PathConstraints(
                4.0, 4.0, 
                Units.degreesToRadians(360), Units.degreesToRadians(540)
            ), 
            0, 
            2.0
        );
    }

    public Auto4Notes(SwerveDrivetrain swerve, String autoPath, NoteAssistance noteAssistance, boolean idk) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            //init
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.setPoseMeters(startingPose)),

            // The path of Pickup 1st
            AutoBuilder.followPath((pathGroup.get(0))), 

            // Find the note
            Commands.runOnce(() -> noteAssistance.resetBuffer()),
            Commands.race(
                Commands.run(() -> noteAssistance.driveToNote(swerve, 4.5, 0, 0), swerve), // need to calibrate the ta
                Commands.waitSeconds(1.5) // to be changed to 1sec or less
            ),

            Commands.runOnce(() -> swerve.stopModules()) //faster?
        );
    }
}
