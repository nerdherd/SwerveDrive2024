// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.autos.Auto4Notes;
import frc.robot.commands.autos.Squarto;
import frc.robot.subsystems.Shooter;
// import frc.robot.commands.autos.SquareTest;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.NavX;
// import frc.robot.subsystems.imu.Pigeon;
import frc.robot.subsystems.imu.PigeonV2;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.swerve.SwerveDrivetrain.SwerveModuleType;
import frc.robot.subsystems.vision.NoteAssistance;
import frc.robot.subsystems.vision.farfuture.DriverAssist;
import frc.robot.subsystems.vision.farfuture.EMPeach;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public Shooter shooter = new Shooter();
  public Gyro imu = new PigeonV2(1);
  public SwerveDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

  private final CommandPS4Controller commandDriverController = new CommandPS4Controller(
      ControllerConstants.kDriverControllerPort);
  private final PS4Controller driverController = commandDriverController.getHID();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller commandOperatorController = new CommandPS4Controller(
      ControllerConstants.kOperatorControllerPort);
  private final PS4Controller operatorController = commandOperatorController.getHID();
  // private final Joystick joystick = new Joystick(2);

  private final LOG_LEVEL loggingLevel = LOG_LEVEL.MINIMAL;

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // private PrimalSunflower backSunflower = new PrimalSunflower(VisionConstants.kLimelightBackName);
  // private PrimalSunflower frontSunflower = new PrimalSunflower(VisionConstants.kLimelightFrontName, 0.3); //0.6 is threshold for consistent ATag detection

  private NoteAssistance noteCamera; 
  private DriverAssist apriltagCamera;// = new DriverAssist(VisionConstants.kLimelightFrontName, 4);
  //private Citron frontCitron = new Citron(VisionConstants.kPhotonVisionFrontName);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try {
      // Pass in "sunflowers" in reverse order of priority (most important last)
      // swerveDrive = new SwerveDrivetrain(imu, SwerveModuleType.CANCODER, frontSunflower);

      noteCamera = new NoteAssistance(VisionConstants.kLimelightFrontName);
      apriltagCamera = new DriverAssist(VisionConstants.kLimelightBackName, 4);
      swerveDrive = new SwerveDrivetrain(imu, SwerveModuleType.CANCODER, apriltagCamera);

    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    }

    // driverAssist.changePipeline(4);
    apriltagCamera.toggleLight(false);

    initAutoChoosers();
    initShuffleboard();

    // Configure the trigger bindings
    configureBindings();

    DriverStation.reportWarning("Initalization complete", false);
  }

  public static boolean IsRedSide()
    {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

  public void initDefaultCommands() {
    swerveDrive.setDefaultCommand(
      new SwerveJoystickCommand(
        swerveDrive,
        () -> -commandDriverController.getLeftY(), // Horizontal translation
        commandDriverController::getLeftX, // Vertical Translation
        // () -> 0.0, // debug
        commandDriverController::getRightX, // Rotationaq

        // driverController::getSquareButton, // Field oriented
        () -> false, // Field oriented

        driverController::getCrossButton, // Towing
        // driverController::getR2Button, // Precision/"Sniper Button"
        () -> driverController.getR2Button(), // Precision mode (disabled)
        () -> driverController.getCircleButton(), // Turn to angle
        // () -> false, // Turn to angle (disabled)
        () -> { // Turn To angle Direction
          return 0.0;
        }
      ));
  }

  private void configureBindings() {
    // Note: whileTrue() does not restart the command if it ends while the button is
    // still being held
    commandDriverController.share().onTrue(Commands.runOnce(imu::zeroHeading).andThen(() -> imu.setOffset(0)));
    commandDriverController.triangle()
      .onTrue(Commands.runOnce(() -> swerveDrive.setVelocityControl(true)))
      .onFalse(Commands.runOnce(() -> swerveDrive.setVelocityControl(false)));

    // swerveDrive.drive(vision.getMovePowerToImp(0.5)[0], vision.getMovePowerToImp(0.5)[1], vision.getMovePowerToImp(0.5)[2])));
    // commandDriverController.L2().whileTrue(Commands.run(() -> driverAssist.calculateTag(1.8, 0, 0, 7)));
    // commandDriverController.L1().whileTrue(Commands.run(() -> swerveDrive.drive(driverAssist.getForwardPower(), driverAssist.getSidewaysPower(), driverAssist.getAngledPower())));
    
    
    // Please Comment out one set of these two to run!!!
    // commandDriverController.L2().whileTrue(Commands.run(() -> noteCamera.speedToNote(4.1, 0, 0)))
    //   .onFalse(Commands.run(() -> noteCamera.resetBuffer()));
    commandDriverController.L1().whileTrue(Commands.run(() -> noteCamera.driveToNote(swerveDrive, 4.1, 0, 0.1, -1)))
      .onFalse(Commands.run(() -> noteCamera.reset()));
      
    
    //commandDriverController.L2().whileTrue(Commands.run(() -> apriltagCamera.calculateTag(1.8, 0, 0, 7))); // testing
    //commandDriverController.L1().whileTrue(Commands.run(() -> apriltagCamera.TagDriving(swerveDrive, 1, 0, 0, 7))); //1.8, 0, 0, 7
    //   .onFalse(Commands.run(() -> swerveDrive.stopModules()));
    
    commandDriverController.L2().whileTrue(Commands.run(() -> apriltagCamera.TagAimingRotation(swerveDrive, 7, -1)))
       .onFalse(Commands.run(() -> apriltagCamera.reset()));



    // driverAssist.changePipeline(1); // Change to pipeline 1 for drive to ring
      // commandDriverController.povUp().onTrue(shooter.increaseTop());
      // commandDriverController.povDown().onTrue(shooter.decreaseTop());
  
      // commandDriverController.povLeft().onTrue(shooter.increaseBottom());
      // commandDriverController.povRight().onTrue(shooter.decreaseBottom());
  
      // commandDriverController.cross()
      //   .onTrue(shooter.setIndex(0).andThen(shooter.setSpeed()))
      //   .onFalse(shooter.setPowerZeroCommand());
      
      // commandDriverController.square()
      //   .onTrue(shooter.setIndex(1).andThen(shooter.setSpeed()))
      //   .onFalse(shooter.setPowerZeroCommand());
  }

  private void initAutoChoosers() {
    // Remember to load the pathplanner paths here
    // final String[] paths = {
    //   // "2 piece"
      
    // };
    
    // for (String path : paths) {
    //   PathPlannerAutos.initPath(path);
    //   PathPlannerAutos.initPathGroup(path);
    // }

    // autoChooser.addOption("Do Nothing", Commands::none);
    // // autoChooser.addOption("SquareTest", () -> new SquareTest(swerveDrive));
    // autoChooser.addOption("BackwardsSquareTest", () -> PathPlannerAutos.pathplannerAuto("TestSquare2", swerveDrive));
    // autoChooser.addOption("LTest", () -> PathPlannerAutos.pathplannerAuto("LTest", swerveDrive));
    // autoChooser.addOption("LTest 2", () -> PathPlannerAutos.pathplannerAuto("LTest Copy", swerveDrive));
    // autoChooser.addOption("2 piece", () -> new PathPlannerAuto("2 Piece"));
    // // autoChooser.addOption("TwoMeterNinetyDegree", () -> PathPlannerAutos.pathplannerAuto("TwoMeterNinetyDegree", swerveDrive));
    // //autoChooser.addOption("FiveMeterNinetyDegree", () -> PathPlannerAutos.pathplannerAuto("FiveMeterNinetyDegree", swerveDrive));
    // // autoChooser.addOption("TwoMeterFortyFiveDegree", () -> PathPlannerAutos.pathplannerAuto("TwoMeterFortyFiveDegree", swerveDrive));
    // autoChooser.addOption("FiveMeterFortyFiveDegree Copy", () -> PathPlannerAutos.pathplannerAuto("FiveMeterFortyFiveDegree Copy", swerveDrive));
    // autoChooser.addOption("ZeroDegreeLine", () -> PathPlannerAutos.pathplannerAuto("ZeroDegreeLine", swerveDrive));
    // autoChooser.addOption("GetBackWithVision", () -> PathPlannerAutos.pathplannerAuto("GetBackWithVision", swerveDrive));

	List<String> paths = AutoBuilder.getAllAutoNames();
    autoChooser.addOption("Do Nothing", Commands.none());

    for (String path : paths) {
      if(path.equals("4PAuto"))
        autoChooser.addOption(path, new Auto4Notes(swerveDrive, path, noteCamera, apriltagCamera));
      else if(path.equals("Squarto")) {
        autoChooser.addOption(path, new Squarto(swerveDrive, path));
      }
    }

    // these are the auto paths in the old format (not the actual full auto command)
    // autoChooser.addOption("Path Planner Test Auto", () -> PathPlannerAutos.pathplannerAuto("TestPath", swerveDrive));

    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
  }
  
  public void initShuffleboard() {
    imu.initShuffleboard(loggingLevel);
    shooter.initShuffleboard(loggingLevel);
    // backSunflower.initShuffleboard(loggingLevel);
    // frontSunflower.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(loggingLevel);
    apriltagCamera.initShuffleboard(LOG_LEVEL.MEDIUM);
    noteCamera.initShuffleboard(LOG_LEVEL.MEDIUM);

    ShuffleboardTab tab = Shuffleboard.getTab("Main");
    // tab.addNumber("Total Current Draw", pdp::getTotalCurrent);
    tab.addNumber("Voltage", () -> Math.abs(pdp.getVoltage()));
  }

  // public void reportAllToSmartDashboard() {
  //   imu.reportToSmartDashboard(loggingLevel);
  //   wrist.reportToSmartDashboard(loggingLevel);
  //   shooter.reportToSmartDashboard(loggingLevel);
  //   swerveDrive.reportToSmartDashboard(loggingLevel);
  //   swerveDrive.reportModulesToSmartDashboard(loggingLevel);
  // }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command currentAuto = autoChooser.getSelected();

    swerveDrive.setDriveMode(DRIVE_MODE.AUTONOMOUS);
    return currentAuto;
  }
}