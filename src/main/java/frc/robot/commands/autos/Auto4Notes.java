package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.NoteAssistance;

public class Auto4Notes extends SequentialCommandGroup {
    public Auto4Notes(SwerveDrivetrain swerve, String autoPath) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            //Commands.none()
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),

            Commands.runOnce(()->swerve.setPoseMeters(startingPose)),
            AutoBuilder.followPath((pathGroup.get(0))), // Pickup 1
            Commands.waitSeconds(1),
            AutoBuilder.followPath((pathGroup.get(1))), // Pickup 2
            Commands.waitSeconds(1),
            AutoBuilder.followPath((pathGroup.get(2))), // Pickup 3
            Commands.waitSeconds(1),
            AutoBuilder.followPath((pathGroup.get(3))), // Pos Mid
            Commands.waitSeconds(1),
            AutoBuilder.followPath((pathGroup.get(4))), // Back Shoot
            Commands.waitSeconds(0.5),
            AutoBuilder.followPath((pathGroup.get(5))), // Pos Mid
            Commands.waitSeconds(1),
            AutoBuilder.followPath((pathGroup.get(6))), // Back Shoot
            Commands.waitSeconds(0.5),
            AutoBuilder.followPath((pathGroup.get(7))), // Back Shoot
            Commands.waitSeconds(0.5)
        );
    }

    public Auto4Notes(SwerveDrivetrain swerve, String autoPath, NoteAssistance noteAssistance) {     
        
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
