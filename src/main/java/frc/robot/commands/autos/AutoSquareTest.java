package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class AutoSquareTest extends SequentialCommandGroup {
    public AutoSquareTest(SwerveDrivetrain swerve, String autoPath) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            //Commands.none()
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(() -> {
                swerve.setPoseMetersWithAlliance(startingPose);
            }),
            Commands.runOnce(()->swerve.setPoseMeters(startingPose)),
            AutoBuilder.followPath((pathGroup.get(0))),
            Commands.runOnce(() -> swerve.towModules())
        );
    }
}