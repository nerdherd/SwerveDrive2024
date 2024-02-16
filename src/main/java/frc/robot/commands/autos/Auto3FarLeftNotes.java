package frc.robot.commands.autos;

import java.util.List;
import java.util.function.BooleanSupplier;

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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.NoteAssistance;
import frc.robot.subsystems.vision.farfuture.DriverAssist;

public class Auto3FarLeftNotes  {
    public Auto3FarLeftNotes(SwerveDrivetrain swerve, String autoPath, NoteAssistance notething, DriverAssist tagAssist) {     
        
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

        //addCommands
        //(
            //Commands.none()
            // Commands.runOnce(swerve.getImu()::zeroAll),
            // Commands.runOnce(()->tagAssist.resetInitPoseByVision(swerve, startPose2d, aimTargetApriltagID) ),
            // Commands.waitSeconds(2), // debug time

            // // Pickup 1 //AutoBuilder.followPath((pathGroup.get(0))), 
            // PathCurrentToDest(firstPickPose, 0.9, 3, 360.0, 540.0),
            // //notething.driveToNoteCommand(swerve, 7.7, 7, 0.1, 10, 40, firs *tPickPose),
            // Commands.waitSeconds(4),
            // tagAssist.aimToApriltagCommand(swerve, aimTargetApriltagID, 4, 20, firstPickPose, true),
            // // skip this aim&shoot, do it at next location
            // Commands.waitSeconds(4),

            // // Pickup 2 //
            // AutoBuilder.followPath((pathGroup.get(1))), 
            // //PathCurrentToDest(secondPickPose, 0.9, 1.5, 360.0, 540.0),
            
            // //FindPathThenFollowPlanned(pathGroup.get(1), 0.9, 3, 360.0, 540.0), // because the pose was changed in the previous step
            // //tagAssist.aimToApriltagCommand(swerve, aimTargetApriltagID, 4, 20, secondPickPose, true),
            // //notething.driveToNoteCommand(swerve, 6.8, 9, 0.1, 10, 40, secondPickPose),
            // // tagAssist.aimToApriltagCommand(swerve, aimTargetApriltagID, 4, 20, secondPickPose, true),
            // // shoot twice 
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
            //Commands.none()
        //);
    }

    public class FarAuto //extends Command
    {
        public enum AutoState
        {
            INIT,
            PATH_PLAN_TO_NOTE,
            IMU_DRIVE_ZERO,
            SEEK_NOTE_ROTATION,
            DRIVE_TO_NOTE,
            PICKUP_CONFIMATION,
            PATH_PLAN_TO_SHOOT_POSE,
            APRILTAG_AIM_SHOOT,
            PATH_PLAN_TO_NEXT_NOTE,
            PATH_PLAN_FROM_SHOOT,
            WAIT,
            EXIT
        }

        public AutoState NextState = AutoState.INIT;
        //public AutoState CurrentState = AutoState.WAIT;
        public AutoState PreviousState = AutoState.EXIT;

        SwerveDrivetrain swerveDrive;
        String autoPath;
        NoteAssistance noteAssist;
        DriverAssist tagAssist;

        // Use the PathPlannerAuto class to get a path group from an auto
        PathPlannerPath InitToFirstNotePath;
        PathPlannerPath FirstNoteShootPath;
        PathPlannerPath ShootToSecondNotePath;
        PathPlannerPath SecondNoteShootPath;
        PathPlannerPath ShootToThirdNotePath;
        PathPlannerPath ThirdNoteShootPath;
        PathPlannerPath FirstToSecondNotePath;
        PathPlannerPath SecondToThirdNotePath;

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startPose2d;
        Pose2d FirstPickPose2d;
        Pose2d SecondPickPose2d;
        Pose2d ThirdPickPose2d;
        Pose2d FirstShootPose2d;


        int aimTargetApriltagID = 7;

        public FarAuto(SwerveDrivetrain swerve, String autoPath, NoteAssistance notething, DriverAssist tagAssist)
        {
            List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);
            startPose2d = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

            // Blue side
            InitToFirstNotePath = pathGroup.get(0);            
            FirstNoteShootPath = pathGroup.get(1);
            ShootToSecondNotePath = pathGroup.get(2);
            SecondNoteShootPath = pathGroup.get(3);
            ShootToThirdNotePath = pathGroup.get(4);
            ThirdNoteShootPath = pathGroup.get(5);
            FirstToSecondNotePath = pathGroup.get(6);
            SecondToThirdNotePath = pathGroup.get(7);

            FirstPickPose2d = GetEndPointInPath(InitToFirstNotePath);
            SecondPickPose2d = GetEndPointInPath(ShootToSecondNotePath);
            ThirdPickPose2d = GetEndPointInPath(ShootToThirdNotePath);

            FirstShootPose2d = GetEndPointInPath(FirstNoteShootPath);

            aimTargetApriltagID = 7;

            if(RobotContainer.IsRedSide())// double check if it's needed, because path may be fliped already
            {
                FirstPickPose2d = GeometryUtil.flipFieldPose(FirstPickPose2d);
                SecondPickPose2d = GeometryUtil.flipFieldPose(SecondPickPose2d);
                ThirdPickPose2d = GeometryUtil.flipFieldPose(ThirdPickPose2d);

                FirstShootPose2d = GeometryUtil.flipFieldPose(FirstShootPose2d);

                aimTargetApriltagID = 4;
            }
        }
        public void setState(AutoState next)
        {
            // return new Command() {
            //     NextState = next;
            // };
            NextState = next;
        }

        public void StateJump()
        {

            if(false)
                Stop = false;
        }
        boolean Stop = false;
        int RountIndex = 0;
        Pose2d ArrivalPose;
        //@Override
        public void execute()
        {
            switch (NextState) {// each state only can be called once!!!
                case INIT:
                    tagAssist.resetInitPoseByVision(swerveDrive, startPose2d, aimTargetApriltagID);
                    PreviousState = AutoState.INIT;
                    NextState = AutoState.WAIT;
                    break;
                case PATH_PLAN_TO_NOTE:
                    PreviousState = AutoState.PATH_PLAN_TO_NOTE;
                    NextState = AutoState.WAIT; 
                    if(RountIndex == 0)
                    {
                        ArrivalPose = FirstPickPose2d;
                        AutoBuilder.followPath(InitToFirstNotePath).andThen(Commands.run(() -> setState(AutoState.IMU_DRIVE_ZERO))).withTimeout(3).end(Stop);
                    }
                    else if(RountIndex == 1)
                    {
                        ArrivalPose = SecondPickPose2d;
                        AutoBuilder.followPath(ShootToSecondNotePath);
                    }
                    else if(RountIndex == 2)
                    {
                        ArrivalPose = ThirdPickPose2d;
                        AutoBuilder.followPath(ShootToThirdNotePath);
                    }
                    else
                        NextState = AutoState.EXIT;
                    break;
                case IMU_DRIVE_ZERO:
                    TurnToAngleWithVision(0, ArrivalPose, noteAssist); // don't turn if the note is there and current angle is very good
                    NextState = AutoState.SEEK_NOTE_ROTATION;
                    break;
                case SEEK_NOTE_ROTATION:
                    //noteAssist.SeekNoteRotation(20, -10, ArrivalPose); // angle range for scanning
                    if(true) // saw one
                        NextState = AutoState.DRIVE_TO_NOTE;
                    else // none
                        NextState = AutoState.PATH_PLAN_TO_NEXT_NOTE;
                    break;
                case DRIVE_TO_NOTE:
                    //noteAssist.drivePickupNoteCommand(swerveDrive, 7, 1, 1, 2, 10, ArrivalPose);// move forward to pick and then move backward
                    NextState = AutoState.PICKUP_CONFIMATION;
                    // timeout or cross line 
                    break;
                case PICKUP_CONFIMATION:
                    //colorSensor.Read();
                    if(false)
                    {
                        NextState = AutoState.PATH_PLAN_TO_NEXT_NOTE;
                    }
                    else
                    {
                        NextState = AutoState.PATH_PLAN_TO_SHOOT_POSE;
                    }
                    break;
                case PATH_PLAN_TO_SHOOT_POSE:
                    if(RountIndex == 0)
                    {
                        ArrivalPose = FirstShootPose2d;
                        AutoBuilder.followPath(FirstNoteShootPath);
                    }
                    else if(RountIndex == 1)
                    {
                        ArrivalPose = FirstShootPose2d; // may changed to diff path
                        AutoBuilder.followPath(SecondNoteShootPath);
                    }
                    else if(RountIndex == 2)
                    {
                        ArrivalPose = FirstShootPose2d;// may changed to diff path
                        AutoBuilder.followPath(ThirdNoteShootPath);
                    }
                    else
                        NextState = AutoState.EXIT; //Error
                    NextState = AutoState.APRILTAG_AIM_SHOOT;
                    break;
                case APRILTAG_AIM_SHOOT:
                    //tagAssist.TagAimingRotation(swerveDrive, aimTargetApriltagID, 10, ArrivalPose);
                    //shooter.run(ArrivalPose);
                    RountIndex++;
                    NextState = AutoState.PATH_PLAN_TO_NOTE;
                    break;
                case PATH_PLAN_TO_NEXT_NOTE:
                    NextState = AutoState.IMU_DRIVE_ZERO; 
                    if(RountIndex == 0)
                    {
                        ArrivalPose = SecondPickPose2d;
                        AutoBuilder.followPath(FirstToSecondNotePath);
                    }
                    else if(RountIndex == 1)
                    {
                        ArrivalPose = ThirdPickPose2d;
                        AutoBuilder.followPath(SecondToThirdNotePath);
                    }
                    else
                    {
                        NextState = AutoState.EXIT; //Error
                    }
                    RountIndex++;
                    break;
                case WAIT:
                    StateJump();

                default: //EXIT
                    break;
            }            
        }

        public Command TurnToAngleWithVision(double targetDegrees, Pose2d ArrivalPose, NoteAssistance noteAssist) {
            // Create a controller for the inline command to capture
            double kTurnToAngleP = 0.;
            double kTurnToAngleTolerance = 0.;
            PIDController controller = new PIDController(kTurnToAngleP, 0, 0);
            // We can do whatever configuration we want on the created state before returning from the factory
            controller.setTolerance(kTurnToAngleTolerance);

            // Try to turn at a rate proportional to the heading error until we're at the setpoint, then stop
            return Commands.run(() -> swerveDrive.drive(0., 0.,-controller.calculate(swerveDrive.getImu().getHeading(), targetDegrees)))
                .until(controller::atSetpoint)
                .andThen(Commands.runOnce(() -> swerveDrive.drive(0, 0, 0)));
        }

       // @Override
        public void initialize() {
        }

        //@Override
        public boolean isFinished() {
            return true;
        }

        //@Override
        public void end(boolean interrupted) {}
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