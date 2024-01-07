// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.preferences.PrefBool;
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
  public static class WristConstants {
    public static final int kWristID = 20;
    // public static final int kRightWristID = 0;
    public static final int kLeftEncoderID = 15;
    public static final int kWristStow = 39000; //43087; //39000; // multiplied by 1.5 
    public static final int kWristStowPowerOff = 39501; // multiplied by 1.5
    
    public static final int kWristSuperLowPickup = 5000; //1500; // multiplied by 1.5
    public static final int kWristLowPickup = 6500; // Tune later for low but not too low

    public static final int kWristLow = kWristStow;
    public static final int kWristMid = kWristStow;
    public static final int kWristHigh = kWristStow;
    public static final int kWristOff = 32400; //34500; // everything after this is basically stow // multiplied by 1.5
    
    public static final int kWristLowerLimit = 0;
    public static final int kWristUpperLimit = kWristStow;

    public static final PrefDouble kWristP = new PrefDouble("WristP", 0.25);
    public static final PrefDouble kWristI = new PrefDouble("WristI", 0);
    public static final PrefDouble kWristD = new PrefDouble("WristD", 0);
    public static final PrefDouble kWristF = new PrefDouble("WristF", 0);
    public static final double kLowPassAlpha = 0.11765;
    public static final double kJoystickScale = 500;
    public static final int kWristCruiseVelocity = 9000;
    public static final int kWristMotionAcceleration = 30000;
    public static final double kWristDeadband = 0.05;
    public static final PrefDouble kWristFF = new PrefDouble("Wrist_FF", 0.05);
    public static final double kWristGearRatio = 60;
    public static final double kFalconTicksPerAbsoluteTicks = kWristGearRatio * 2048/4096;
    public static final double kDegreesPerTick = 360 / kWristGearRatio / 2048;

  }

  public static class ShooterConstants {
    public static final int kTopMotorID = 18;
    public static final int kBottomMotorID = 19;
    public static final PrefDouble kTopIntakePower = new PrefDouble("TopIntakePower", -0.35); //-0.3
    public static final PrefDouble kTopLowOuttakePower = new PrefDouble("TopLowOuttakePower", 0.1); //0.5
    public static final PrefDouble kTopMidOuttakePower = new PrefDouble("TopMidOuttakePower", 0.3); //0.5
    public static final PrefDouble kTopHighOuttakePower = new PrefDouble("TopHighOuttakePower", 0.2); //0.5
    public static final PrefDouble kTopHighFlatOuttakePower = new PrefDouble("TopHighFlatOuttakePower", 0.2); //0.5
    public static final PrefDouble kTopFullOuttakePower = new PrefDouble("TopFullOuttakePower", (0.8));
    public static final PrefDouble kTopIntakeNeutralPower = new PrefDouble("TopIntakeNeutralPower", -0.05); //-0.25
    public static final PrefDouble kTopAutoOuttakePower = new PrefDouble("TopAutoOuttakePower", 0.4); // tune this!
    public static final PrefDouble kTopCubeCurrent = new PrefDouble("TopCubeCurrent", 30); // tune this!

    public static final PrefDouble kBottomIntakePower = new PrefDouble("BottomIntakePower", -0.35); //-0.3
    public static final PrefDouble kBottomLowOuttakePower = new PrefDouble("BottomLowOuttakePower", 0.2); //0.5
    public static final PrefDouble kBottomMidOuttakePower = new PrefDouble("BottomMidOuttakePower",0.4); //0.5
    public static final PrefDouble kBottomHighOuttakePower = new PrefDouble("BottomHighOuttakePower", 0.6); //0.5
    public static final PrefDouble kBottomHighFlatOuttakePower = new PrefDouble("BottomHighFlatOuttakePower", 0.6); //0.5
    public static final PrefDouble kBottomFullOuttakePower = new PrefDouble("BottomFullOuttakePower", 0.75);
    public static final PrefDouble kBottomIntakeNeutralPower = new PrefDouble("BottomIntakeNeutralPower", -0.05); //-0.25
    public static final PrefDouble kBottomAutoOuttakePower = new PrefDouble("BottomAutoOuttakePower", 0.5); // tune this!
    public static final PrefDouble kBottomCubeCurrent = new PrefDouble("BottomCubeCurrent", 30); // tune this!
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
    
    public static final double kPDrive = 0.13;
    public static final double kIDrive = 0;
    public static final double kDDrive = 0;
    public static final double kFDrive = 0.0469;

    public static final PrefBool ktunePID = new PrefBool("TuneDrivePID", false);
    public static final String kCANivoreName = "CANivore1";
  } 

  public static final class SwerveDriveConstants {

    public static final double kVisionSTDx = 0.9;
    public static final double kVisionSTDy = 0.9;
    public static final double kVisionSTDtheta = 69696969;
    public static final Matrix<N3, N1> kBaseVisionPoseSTD = VecBuilder.fill(kVisionSTDx, kVisionSTDy, kVisionSTDtheta);


    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(21);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
    public static final double kDodgeDistance = Units.inchesToMeters(12);
    
    public static final Translation2d[] kRotationCenters = new Translation2d[] {
      new Translation2d(kDodgeDistance, kDodgeDistance),  // FL
      new Translation2d(kDodgeDistance, -kDodgeDistance),   // FR
      new Translation2d(-kDodgeDistance, kDodgeDistance), // BL
      new Translation2d(-kDodgeDistance, -kDodgeDistance)   // BR
    };
    
    public static final int[] kLeftRotationCenters = new int[] {0, 1, 3, 2};
    public static final int[] kRightRotationCenters = new int[] {1, 3, 2, 0};

    public static final double kRotationOffset = 0.5 * kTrackWidth;

    public static final int kFRDriveID = 31; //11
    public static final int kFLDriveID = 41; //21
    public static final int kBLDriveID = 11; //31
    public static final int kBRDriveID = 21; //41

    public static final int kFRTurningID = 32; //12
    public static final int kFLTurningID = 42; //22
    public static final int kBLTurningID = 12; //32
    public static final int kBRTurningID = 22; //42

    public static final boolean kFRTurningReversed = true;
    public static final boolean kFLTurningReversed = true; 
    public static final boolean kBLTurningReversed = true; 
    public static final boolean kBRTurningReversed = true; 

    public static final boolean kFRDriveReversed = false;
    public static final boolean kFLDriveReversed = false;     
    public static final boolean kBLDriveReversed = false;      
    public static final boolean kBRDriveReversed = false;

    public static final class CANCoderConstants {
      public static final int kFRCANCoderID = 34;
      public static final int kFLCANCoderID = 44;
      public static final int kBLCANCoderID = 14;
      public static final int kBRCANCoderID = 24;

      public static final boolean kFRCANCoderReversed = false;    
      public static final boolean kFLCANCoderReversed = false;      
      public static final boolean kBLCANCoderReversed = false;       
      public static final boolean kBRCANCoderReversed = false; 

      // public static final double kFRCANCoderOffsetDegrees = 25.75;       
      // public static final double kFLCANCoderOffsetDegrees = -53.174;         
      // public static final double kBLCANCoderOffsetDegrees = 85;          
      // public static final double kBRCANCoderOffsetDegrees = 46.85;

      public static final PrefDouble kFROffsetDeg = new PrefDouble("kFROffsetDeg", 0); //-29
      public static final PrefDouble kFLOffsetDeg = new PrefDouble("kFLOffsetDeg", 0); //51
      public static final PrefDouble kBLOffsetDeg = new PrefDouble("kBLOffsetDeg", 0); //185
      public static final PrefDouble kBROffsetDeg = new PrefDouble("kBROffsetDeg", 0); //316
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
    
    public static final double kPBalancingInitial = 4.8;
    public static final double kPBalancing = 2.6; //2.7 worked once //2.37; // 0.4
    public static final double kIBalancing = 0;
    public static final double kDBalancing = 0;
    public static final double kPOneWayBalancing = 2.6; //2.7 worked once //2.37; // 0.4
    public static final double kIOneWayBalancing = 0;
    public static final double kDOneWayBalancing = 0;
    public static final double kBalancingDeadbandDegrees = Math.toRadians(2);
    public static final double kBalancingTowPeriod = 0.5;
  }

  public static final class PathPlannerConstants {
    public static final double kPPMaxVelocity = 3;
    public static final double kPPMaxAcceleration = 3;
    public static final PathConstraints kPPPathConstraints = new PathConstraints(kPPMaxVelocity, kPPMaxAcceleration);

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
    public static final double kSunflowerP = 0.2;
    public static final double kSunflowerI = 0;
    public static final double kSunflowerD = 0;
    public static final String kLimelightBackName = "limelight-back";
    public static final String kLimelightFrontName = "limelight-front";
    public static final int kAprilTagPipeline = 4;
    public static final double kSlidingOffset = 0.4; // Meters away from grid while robot is sliding.
    public static final double fieldXOffset = 8.27; // Certified (Half field dimensions)
    public static final double fieldYOffset = 4.01; // Certified (Half as well)
  }
}
