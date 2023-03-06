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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  SendableChooser<Command> autoChooser;

  // The robot's subsystems and commands are defined here...
  
  // Subsystems
  private final DriveSubsystem drive = new DriveSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final PneumaticsSubsystem pneumatics = new PneumaticsSubsystem();
  



  // Commands
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    autoChooser = new SendableChooser<>();

    autoChooser.setDefaultOption("Clack Auto", new ClackAuto(drive));
    autoChooser.addOption("Hybrid Cube", new HybridCube(drive, arm, pneumatics));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    drive.setDefaultCommand(new DriveCommand(drive, driverController::getLeftY, driverController::getRightX));
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
    m_operatorController.a().whileTrue(pneumatics.runOnce(pneumatics::toggleArm));
    m_operatorController.x().whileTrue(pneumatics.runOnce(pneumatics::toggleManipulator));

    m_operatorController.pov(180).onTrue(new SetArm(arm, 0));
    m_operatorController.pov(90).onTrue(new SetArm(arm, 45));
    m_operatorController.pov(0).onTrue(new SetArm(arm, 90));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
