package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.farfuture.DriverAssist;

public class Autonomous2Source extends Command {

    private SwerveDrivetrain swerveDrive;
    private DriverAssist apriltagVision;

    private final Pose2d SourcePose;

    /**
     * Construct a new FollowVisionPath command
     * 
     * Uses primal sunflower to follow a path using pathplanner trajectory
     * 
     * @param autoBuilder Swerve Auto Builder for Path Planner
     * @param sunflower   Primal Sunflower
     */
    public Autonomous2Source(SwerveDrivetrain swerveDrive, DriverAssist apriltagVision) {
        this.swerveDrive = swerveDrive;
        this.apriltagVision = apriltagVision;
        if(RobotContainer.IsRedSide())
            SourcePose = new Pose2d(0, 0, new Rotation2d()); // todo
        else
            SourcePose = new Pose2d(0, 0, new Rotation2d()); // todo
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
    
}