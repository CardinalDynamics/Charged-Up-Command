// sam don't look either
package frc.robot.commands.drive;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.PathPlanner;

// this command was for pathplanner autonomous, but we scrapped it just before wilsonville.
public class MotionProfileCommand extends CommandBase {
    private static final double ramseteB = Constants.MotionConstants.ramseteB;
    private static final double ramseteZeta = Constants.MotionConstants.ramseteZeta;
    
    private final DriveSubsystem drive;
    private Trajectory trajectory;
    private final RamseteController controller = new RamseteController(ramseteB, ramseteZeta);
    private final Timer timer = new Timer();

    public MotionProfileCommand(DriveSubsystem drive, String pathName, boolean reversed) {
        addRequirements(drive);
        this.drive = drive;

        double maxVelocity, maxAcceleration;

        maxVelocity = Constants.MotionConstants.maxVelocity;
        maxAcceleration = Constants.MotionConstants.maxAcceleration;

        try {
            trajectory = PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration, reversed);
        } catch (TrajectoryParameterizer.TrajectoryGenerationException exception) {
            trajectory = new Trajectory();
            DriverStation.reportError("Failed to load trajectory", false);
        }
    }

    double getTime() {
        return trajectory.getTotalTimeSeconds();
    }

    @Override
    public void initialize() {
        drive.setRobotPos(trajectory.getInitialPose());
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        Trajectory.State setpoint = trajectory.sample(timer.get());
        ChassisSpeeds chassisSpeeds = controller.calculate(drive.getRobotPos(), setpoint);
        drive.voltageDrive(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
