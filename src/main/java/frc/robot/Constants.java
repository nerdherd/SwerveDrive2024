// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.preferences.PrefDouble;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 // COMMENT ROBOT IDS INSTEAD OF DELETING
public final class Constants {

  public static class DriveConstants {
    public static final double kDriveAlpha = 0.11765;
    public static final double kDriveOneMinusAlpha = 0.88235;
    public static final double kErrorBound = 0;
  }

  public static class ControllerConstants {
    public static final double kDeadband = 0.05;
    public static final double kRotationDeadband = 0.1;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 1 / 21.428; // 150 : 7 : 1 MK4i
    // public static final double kDriveDistanceLoss = 0.95; // from measuring IRL
    public static final double kDriveDistanceLoss = 1; // from measuring IRL
    public static final double kMetersPerRevolution = kWheelDiameterMeters * Math.PI * kDriveDistanceLoss;
    public static final double kDriveTicksToMeters = (1 / 2048.0) * kMetersPerRevolution; 
    public static final double kAbsoluteTurningTicksToRad = (1.0 / 4096.0) * 2 * Math.PI;
    public static final double kIntegratedTurningTicksToRad = (1.0 / 2048.0) * 2 * Math.PI;
    public static final double kDriveTicksPer100MsToMetersPerSec = kDriveTicksToMeters * 10;
    public static final double kAbsoluteTurningTicksPer100MsToRadPerSec = kAbsoluteTurningTicksToRad * 10;
    public static final double kIntegratedTurningTicksPer100MsToRadPerSec = kIntegratedTurningTicksToRad * 10;

    public static final double kDriveMotorDeadband = 0.02;
    public static final double kTurnMotorDeadband = 0.001;

    public static final PrefDouble kPTurning = new PrefDouble("kPTurning",0.55); // 0.6
    public static final PrefDouble kITurning = new PrefDouble("kITurning",0);
    public static final PrefDouble kDTurning = new PrefDouble("kDTurning",0.02); 
    public static final PrefDouble kFTurning = new PrefDouble("kFTurning",0.015); 

    public static final PrefDouble kPDrive = new PrefDouble("kPDrive",0.13); // 0.6
    public static final PrefDouble kIDrive = new PrefDouble("kIDrive",0);
    public static final PrefDouble kDDrive = new PrefDouble("kDDrive",0); 
    public static final PrefDouble kVDrive = new PrefDouble("kVDrive",0.0469); 

    public static final String kCANivoreName = "rio";
  } 

  public static final class SwerveDriveConstants {

    public static final double kVisionSTDx = 0.7; //0.9
    public static final double kVisionSTDy = 0.7; //0.9
    public static final double kVisionSTDtheta = 1000; //Old: 69696969
    public static final Matrix<N3, N1> kBaseVisionPoseSTD = MatBuilder.fill(
                                                              Nat.N3(), Nat.N1(), 
                                                              kVisionSTDx,
                                                              kVisionSTDy,
                                                              kVisionSTDtheta);
    // VecBuilder.fill(kVisionSTDx, kVisionSTDy, kVisionSTDtheta);


    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(21);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
    public static final int kFRDriveID = 11;
    public static final int kFLDriveID = 21;
    public static final int kBLDriveID = 31;
    public static final int kBRDriveID = 41;

    public static final int kFRTurningID = 12;
    public static final int kFLTurningID = 22;
    public static final int kBLTurningID = 32;
    public static final int kBRTurningID = 42;

    public static final boolean kFRTurningReversed = true;
    public static final boolean kFLTurningReversed = true; 
    public static final boolean kBLTurningReversed = true; 
    public static final boolean kBRTurningReversed = true; 

    public static final boolean kFRDriveReversed = false;
    public static final boolean kFLDriveReversed = false;     
    public static final boolean kBLDriveReversed = false;      
    public static final boolean kBRDriveReversed = false;

    public static final class CANCoderConstants {
      public static final int kFRCANCoderID = 14;
      public static final int kFLCANCoderID = 24;
      public static final int kBLCANCoderID = 34;
      public static final int kBRCANCoderID = 44;

      public static final boolean kFRCANCoderReversed = false;    
      public static final boolean kFLCANCoderReversed = false;      
      public static final boolean kBLCANCoderReversed = false;       
      public static final boolean kBRCANCoderReversed = false; 
    }

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;    
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleMaxAcceleration = 5;
    // THIS CONSTANT HAS TO BE NEGATIVE OTHERWISE THE ROBOT WILL CRASH
    //TODO: Change deceleration with driver feedback, only in small increments (<= -2 is dangerous)
    public static final double kTeleMaxDeceleration = -5; // Russell says he likes 2.5 from sims, but keep at 3 until tested on real robot 

    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
      kPhysicalMaxAngularSpeedRadiansPerSecond * 0.75;
    public static final double kTurnToAngleMaxAngularSpeedRadiansPerSecond 
      = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTurnToBigAngleMaxAngularSpeedRadiansPerSecond = 1.5 * Math.PI;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double kMinimumMotorOutput = 0.05; // Minimum percent output on the falcons
    
    public static final double kDriveAlpha = 0.11765;
    public static final double kDriveOneMinusAlpha = 0.88235;

    public static final SwerveModuleState[] towModuleStates = 
    new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(135))
    };

    public static final double kGravityMPS = 9.80665; 
  }

  public static final class SwerveAutoConstants {
    public static final double kPTurnToAngle = SmartDashboard.getNumber("kP Theta Teleop", 6);
    public static final double kITurnToAngle = SmartDashboard.getNumber("kI Theta Teleop", 0);
    public static final double kDTurnToAngle = SmartDashboard.getNumber("kD Theta Teleop", 0.2);
    public static final double kTurnToAnglePositionToleranceAngle = 5;
    public static final double kTurnToAngleVelocityToleranceAnglesPerSec = 2;

    // public static final double kPXController = new PrefDouble("kPXSpeed", 0.5).get();
    // public static final double kIXController = new PrefDouble("kIXSpeed", 0).get();
    // public static final double kDXController = new PrefDouble("kDXSpeed", 0).get();
    // public static final double kPYController = new PrefDouble("kPYSpeed", 0.5).get();
    // public static final double kIYController = new PrefDouble("kIYSpeed", 0).get();
    // public static final double kDYController = new PrefDouble("kDYSpeed", 0).get();
    // public static final double kPThetaController = new PrefDouble("kPThetaAuto", 6.0).get();
    // public static final double kIThetaController = new PrefDouble("kIThetaAuto", 0).get();
    // public static final double kDThetaController = new PrefDouble("kDThetaAuto", 0).get();
    
  }

  public static final class PathPlannerConstants {
    public static final double kPPMaxVelocity = 3;
    public static final double kPPMaxAcceleration = 3;
    public static final double kPPMaxAngularVelocity = Math.PI * 2;
    public static final double kPPMaxAngularAcceleration = Math.PI * 2;
    public static final PathConstraints kPPPathConstraints = new PathConstraints(kPPMaxVelocity, kPPMaxAcceleration, 
                                                                                kPPMaxAngularVelocity, kPPMaxAngularAcceleration);

    public static final double kPP_P = new PrefDouble("PP_kP", 0.25).get();
    public static final double kPP_I = new PrefDouble("PP_kI", 0.0).get();
    public static final double kPP_D = new PrefDouble("PP_kD", 0.0).get();
    public static final PIDConstants kPPTranslationPIDConstants = new PIDConstants(kPP_P, kPP_I, kPP_D);

    public static final double kPP_ThetaP = new PrefDouble("PP_kThetaP", 0.25).get();
    public static final double kPP_ThetaI = new PrefDouble("PP_kThetaI", 0).get();
    public static final double kPP_ThetaD = new PrefDouble("PP_kThetaD", 0).get();
    public static final PIDConstants kPPRotationPIDConstants = new PIDConstants(kPP_ThetaP, kPP_ThetaI, kPP_ThetaD);

    public static final boolean kUseAllianceColor = true;
  }

  public static final class VisionConstants {
    public static final double kFrontCameraHeightMeters = 0;
    public static final double kNoteHeightMeters = 0;
    public static final double kCameraPitchRadians = 0;
    public static final double kSunflowerP = 0.2;
    public static final double kSunflowerI = 0;
    public static final double kSunflowerD = 0;
    public static final String kLimelightFrontName = "limelight-front";
    public static final String kLimelightFrontIP = "10.6.87.25:5802";
    public static final int kAprilTagPipeline = 4;
    public static final double fieldXOffset = 8.27; // Certified (Half field dimensions)
    public static final double fieldYOffset = 4.01; // Certified (Half as well)
    public static final double kMinimumTA = 0.7;
    public static final Transform3d fieldPoseOffset = new Transform3d(
      new Translation3d(-VisionConstants.fieldXOffset, -VisionConstants.fieldYOffset, 0), 
      new Rotation3d()
    );

    public static final String kPhotonVisionFrontName = "laserbean";
    public static final Transform3d kCameraToRobot = new Transform3d(
      new Translation3d(),
      new Rotation3d()
    ); // distance from camera to center of robot
  }

  public static final class ShooterConstants {
    public static final int kTopMotorID = 17;
    public static final int kBottomMotorID = 18;
    public static final int kIntakePower = 0;
    public static final int kOuttakePower = 0;
    public static final PrefDouble kPTopMotor = new PrefDouble("P Top Motor", 0);
    public static final PrefDouble kITopMotor = new PrefDouble("I Top Motor", 0);
    public static final PrefDouble kDTopMotor = new PrefDouble("D Top Motor", 0);
    public static final PrefDouble kVTopMotor = new PrefDouble("V Top Motor", 0);

    public static final PrefDouble kPBottomMotor = new PrefDouble("P Bottom Motor", 0);
    public static final PrefDouble kIBottomMotor = new PrefDouble("I Bottom Motor", 0);
    public static final PrefDouble kDBottomMotor = new PrefDouble("D Bottom Motor", 0);
    public static final PrefDouble kVBottomMotor =new PrefDouble("V Bottom Motor", 0);
  }
}
