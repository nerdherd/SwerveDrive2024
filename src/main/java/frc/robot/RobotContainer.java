// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.SwerveJoystickCommand.DodgeDirection;
// import frc.robot.commands.VisionAutos.ToNearestGridDebug;
import frc.robot.commands.autos.PathPlannerAutos;
import frc.robot.commands.autos.SquareTest;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.NavX;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.swerve.SwerveDrivetrain.SwerveModuleType;
import frc.robot.subsystems.vision.farfuture.Citron;
import frc.robot.subsystems.vision.primalWallnut.PrimalSunflower;

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

  public Gyro imu = new NavX();
  // public Gyro imu = new Pigeon(60);
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
  private final POVButton upButton = new POVButton (operatorController,0);
  private final POVButton rightButton = new POVButton (operatorController, 90);
  private final POVButton downButton = new POVButton (operatorController, 180);
  private final POVButton leftButton = new POVButton (operatorController, 270);

  private final POVButton upButtonDriver = new POVButton (driverController, 0);
  private final POVButton rightButtonDriver = new POVButton (driverController, 90);
  private final POVButton downButtonDriver = new POVButton (driverController, 180);
  private final POVButton leftButtonDriver = new POVButton (driverController, 270);

  private SendableChooser<Supplier<CommandBase>> autoChooser = new SendableChooser<Supplier<CommandBase>>();

  // private PrimalSunflower backSunflower = new PrimalSunflower(VisionConstants.kLimelightBackName);
  // private PrimalSunflower frontSunflower = new PrimalSunflower(VisionConstants.kLimelightFrontName, 0.7); //0.6 is threshold for consistent ATag detection
  private Citron frontCitron = new Citron(VisionConstants.kPhotonVisionFrontName);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try {
      // Pass in "sunflowers" in reverse order of priority (most important last)
      swerveDrive = new SwerveDrivetrain(imu, SwerveModuleType.CANCODER, frontCitron);
    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    }

    initAutoChoosers();
    initShuffleboard();

    // Configure the trigger bindings
    configureBindings();

    DriverStation.reportWarning("Initalization complete", false);
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
    commandDriverController.options().onTrue(Commands.runOnce(swerveDrive::resetEncoders));
    commandDriverController.triangle()
      .onTrue(Commands.runOnce(() -> swerveDrive.setVelocityControl(true)))
      .onFalse(Commands.runOnce(() -> swerveDrive.setVelocityControl(false)));
  }

  private void initAutoChoosers() {
    // Remember to load the pathplanner paths here
    final String[] paths = {
      "TestSquare", "TestSquare2", "LTest", "LTest Copy", "TwoMeterNinetyDegree", "FiveMeterNinetyDegree", "TwoMeterFortyFiveDegree", "FiveMeterFortyFiveDegree"
    };
    
    PathPlannerAutos.init(swerveDrive);

    for (String path : paths) {
      PathPlannerAutos.initPath(path);
      PathPlannerAutos.initPathGroup(path);
    }

    autoChooser.addOption("Do Nothing", Commands::none);
    autoChooser.addOption("SquareTest", () -> new SquareTest(PathPlannerAutos.autoBuilder, swerveDrive));
    autoChooser.addOption("BackwardsSquareTest", () -> PathPlannerAutos.pathplannerAuto("TestSquare2", swerveDrive));
    autoChooser.setDefaultOption("LTest", () -> PathPlannerAutos.pathplannerAuto("LTest", swerveDrive));
    autoChooser.addOption("LTest Copy", () -> PathPlannerAutos.pathplannerAuto("LTest Copy", swerveDrive));
    autoChooser.addOption("TwoMeterNinetyDegree", () -> PathPlannerAutos.pathplannerAuto("TwoMeterNinetyDegree", swerveDrive));
    autoChooser.addOption("FiveMeterNinetyDegree", () -> PathPlannerAutos.pathplannerAuto("FiveMeterNinetyDegree", swerveDrive));
    autoChooser.addOption("TwoMeterFortyFiveDegree", () -> PathPlannerAutos.pathplannerAuto("TwoMeterFortyFiveDegree", swerveDrive));
    autoChooser.addOption("FiveMeterFortyFiveDegree", () -> PathPlannerAutos.pathplannerAuto("FiveMeterFortyFiveDegree", swerveDrive));


    // these are the auto paths in the old format (not the actual full auto command)
    // autoChooser.addOption("Path Planner Test Auto", () -> PathPlannerAutos.pathplannerAuto("TestPath", swerveDrive));

    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
  }
  
  public void initShuffleboard() {
    imu.initShuffleboard(loggingLevel);
    // backSunflower.initShuffleboard(loggingLevel);
    // frontSunflower.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(loggingLevel);
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
    Command currentAuto = autoChooser.getSelected().get();

    swerveDrive.setDriveMode(DRIVE_MODE.AUTONOMOUS);
    return currentAuto;
  }
}
