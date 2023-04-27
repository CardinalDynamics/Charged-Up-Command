// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.arm.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  // Mostly Limelight code, can be ignored
  public static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  // SlewRateLimiter, basically an acceleration curve for the drivetrain
  SlewRateLimiter limit = new SlewRateLimiter(1.2);

  SendableChooser<Command> autoChooser;
  // SendableChooser<Boolean> limelightMode;

  // The robot's subsystems and commands are defined here...
  
  // Subsystems
  private final DriveSubsystem drive = new DriveSubsystem();
  // private final ArmSubsystem arm = new ArmSubsystem();
  private final PneumaticsSubsystem pneumatics = new PneumaticsSubsystem();
  // private final VisionSubsystem vision = new VisionSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();

  // Commands
  private final Intake intakeCommand = new Intake(intake);
  private final Outtake outtakeCommand = new Outtake(intake);
  private final IntakeFullSend intakeFullSend = new IntakeFullSend(intake);

  // Controller initialization
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // private final CommandXboxController m_operatorController =
  //     new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // dropdown menu for selecting the autonomous
    autoChooser = new SendableChooser<Command>();
    // limelightMode = new SendableChooser<Boolean>();

    // adding options for the dropdown menu
    autoChooser.setDefaultOption("Clack Auto", new ClackAuto(drive));
    // autoChooser.addOption("Clack Auto (ORIGINAL)", new ClackAutoAlt(drive));
    autoChooser.addOption("Cube Auto", new CubeAuto(drive, pneumatics, intake));
    autoChooser.addOption("Auto Level", new AutoLevel(drive));
    // autoChooser.addOption("Hybrid Cube", new HybridCube(drive, pneumatics, intake));
    // autoChooser.addOption("Test Auto", new TestAuto(drive));
    autoChooser.addOption("Auto Disabled", new AutoDisabled());

    // limelightMode.setDefaultOption("Drive", true);
    // limelightMode.addOption("Vision", false);

    // putting the dropdown menu on the dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // SmartDashboard.putData("Limelight Mode", limelightMode);

    // setting the default command for the drive subsystem
    drive.setDefaultCommand(new DriveCommand(
      drive, 
      () -> {
        return Math.abs(m_driverController.getLeftY()) > Constants.OperatorConstants.yDeadband ?
        -limit.calculate(m_driverController.getLeftY()) : 
        0;
      }, 
      () -> -m_driverController.getRightX())
    );
    // arm.setDefaultCommand(new ArmLift(arm, m_operatorController::getRightTriggerAxis, () -> false));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // setting the various buttons on the drive controller
    m_driverController.rightBumper().whileTrue(intakeCommand);
    m_driverController.leftBumper().whileTrue(outtakeCommand);

    m_driverController.x().whileTrue(intakeFullSend);

    m_driverController.a().whileTrue(pneumatics.runOnce(pneumatics::toggleArm));

    // m_operatorController.a().whileTrue(pneumatics.runOnce(pneumatics::toggleArm));
    // m_operatorController.x().whileTrue(pneumatics.runOnce(pneumatics::toggleManipulator));

    // m_operatorController.rightBumper().whileTrue(intakeCommand);
    // m_operatorController.leftBumper().whileTrue(outtakeCommand);

    // m_operatorController.pov(-1).whileTrue(new SetArm(arm, 70));
    // m_operatorController.pov(90).onTrue(new SetArm(arm, 175));
    // m_operatorController.pov(0).onTrue(new SetArm(arm, 359));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // telling the robot what to do in autonomous
    return autoChooser.getSelected();
  }

  // public void limelightMode() {
  //   if (limelightMode.getSelected()) {
  //     vision.setModeDriver();
  //   } else {
  //     vision.setModeVision();
  //   }
  // }

  public void debug() {
    // putting some values on the dashboard
    SmartDashboard.putData(drive);
    // SmartDashboard.putData(arm);
    SmartDashboard.putData(pneumatics);
    // SmartDashboard.putData(vision);
    SmartDashboard.putData(intake);
    pneumatics.debug();
  }
}