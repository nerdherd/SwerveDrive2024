package frc.robot.commands.autos;


import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.NoteAssistance;
import frc.robot.subsystems.vision.farfuture.DriverAssist;

public class Auto9Notes extends SequentialCommandGroup {
    public Auto9Notes(SwerveDrivetrain swerve, String autoPath, DriverAssist apriltagVision, NoteAssistance noteVision) {
        // Use the PathPlannerAuto class to get a path group from an auto
        final List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);
        final int aimTargetApriltagID;

        // Add these Poses on the shuffleboard, so we can build the auto path dynamically.
        Pose2d firstNotePose;
        Pose2d secondNotePose;
        Pose2d thirPose2d;
        //...

        if(RobotContainer.IsRedSide())
        {
            firstNotePose = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0)));
            aimTargetApriltagID = 4;
        }
        else
        {
            firstNotePose = new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0)));
            aimTargetApriltagID = 7;
        }

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(()->swerve.resetInitPoseByVision()),
            Commands.waitSeconds(10), // debug time

            Path1stCom(firstNotePose), // Pickup 1
            Commands.race(
                Commands.runOnce(()->noteVision.driveToNote(swerve, 4.5, 0, 0)), // need to use its command format. Todo
                Commands.waitSeconds(10)// debug time, intake
            ),
            Commands.race(
                Commands.runOnce(()->apriltagVision.TagAimingRotation(swerve, 0, 0, 0, aimTargetApriltagID)), // need to use its command format. Todo
                Commands.waitSeconds(10)// debug time, shoot
            ),

            AutoBuilder.followPath((pathGroup.get(1))), // Pickup 2
            Commands.waitSeconds(1)
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
}
