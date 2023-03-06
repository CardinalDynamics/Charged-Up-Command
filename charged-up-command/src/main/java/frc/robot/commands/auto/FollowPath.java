package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowPath extends CommandBase {
    private final RamseteCommand ramseteCommand;
    private final DriveSubsystem drive;
    private final Trajectory path;

    public FollowPath(DriveSubsystem m_drive, String path, boolean reversed) {
        drive = m_drive;
        Trajectory trajectory;
        try {
            trajectory = PathPlanner.loadPath(path, Constants.MotionConstants.maxVelocity, Constants.MotionConstants.maxAcceleration, reversed);
            DriverStation.reportError("Trajectory loaded", false);
        } catch (TrajectoryParameterizer.TrajectoryGenerationException exception) {
            trajectory = new Trajectory();
            DriverStation.reportError("Failed to load trajectory", false);
        }
        this.path = trajectory;
        ramseteCommand = new RamseteCommand(
            trajectory,
            drive::getRobotPos,
            new RamseteController(Constants.MotionConstants.ramseteB, Constants.MotionConstants.ramseteZeta),
            new SimpleMotorFeedforward(
                Constants.MotionConstants.ksVolts,
                Constants.MotionConstants.kvVoltSecondsPerMeter,
                Constants.MotionConstants.kaVoltSecondsSquaredPerMeter),
            drive.getKinematics(),
            drive::getWheelSpeeds,
            new PIDController(Constants.MotionConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.MotionConstants.kPDriveVel, 0, 0),
            drive::tankDriveVolts,
            drive);
    }

    public RamseteCommand getRamseteCommand() {
        return ramseteCommand;
    }
    
}
