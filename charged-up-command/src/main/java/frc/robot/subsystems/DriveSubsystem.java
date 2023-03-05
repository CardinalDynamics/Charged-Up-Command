// A decent chunk of this code is from 7034's 2022 robot code

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogGyro;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public class DriveSubsystem extends SubsystemBase {

    private final CANSparkMax leftFront, leftBack, rightFront, rightBack;
    private final RelativeEncoder leftEncoder, rightEncoder;

    private final DifferentialDrive drive;

    private final AnalogGyro gyro;
    private final AHRS navx;

    private final DifferentialDriveOdometry odometry;
    DifferentialDriveKinematics kinematics;

    private double rightOffset;
    private double leftOffset;

    public DriveSubsystem() {
        super();
        
        leftFront = new CANSparkMax(Constants.DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
        leftBack = new CANSparkMax(Constants.DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
        rightFront = new CANSparkMax(Constants.DriveConstants.kRightMotor1Port, MotorType.kBrushless);
        rightBack = new CANSparkMax(Constants.DriveConstants.kRightMotor2Port, MotorType.kBrushless);

        leftBack.follow(leftFront);
        rightBack.follow(rightFront);

        rightFront.setInverted(true);

        drive = new DifferentialDrive(leftFront, rightFront);

        leftFront.setIdleMode(IdleMode.kCoast);
        leftBack.setIdleMode(IdleMode.kCoast);
        rightFront.setIdleMode(IdleMode.kCoast);
        rightBack.setIdleMode(IdleMode.kCoast);

        leftFront.setSmartCurrentLimit(80);
        leftBack.setSmartCurrentLimit(80);
        rightFront.setSmartCurrentLimit(80);
        rightBack.setSmartCurrentLimit(80);

        gyro = new AnalogGyro(0);
        navx = new AHRS(SPI.Port.kMXP);

        leftEncoder = leftFront.getEncoder();
        rightEncoder = rightFront.getEncoder();

        resetEncoders();
        odometry = new DifferentialDriveOdometry(getCurrentAngle(), getLeftEncoderPosition(), getRightEncoderPosition());
    }

    public Pose2d getRobotPos() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    public double getRightEncoderPosition() {
        return rightEncoder.getPosition() - rightOffset;
    }

    public double getLeftEncoderPosition() {
        return leftEncoder.getPosition() - leftOffset;
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(navx.getYaw() + Constants.DriveConstants.gyroOffset);
    }

    public void resetEncoders() {
        leftOffset = leftEncoder.getPosition();
        rightOffset = rightEncoder.getPosition();
    }

    public void setRobotPos(Pose2d pose) {
        odometry.resetPosition(getCurrentAngle(), getLeftEncoderPosition(), getRightEncoderPosition(), pose);
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public void arcadeDrive(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Kinematic drive method for differential drive platform.
     * @param chassisSpeeds
     */
    public void voltageDrive(ChassisSpeeds chassisSpeeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        leftFront.setVoltage(wheelSpeeds.leftMetersPerSecond);
        rightFront.setVoltage(wheelSpeeds.rightMetersPerSecond);
        drive.feed();
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     * @param leftVolts
     * @param rightVolts
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftFront.setVoltage(leftVolts);
        rightFront.setVoltage(rightVolts);
        drive.feed();
    }

    public void resetGyro() {
        gyro.reset();
        navx.reset();
    }

    /**
     * Prints drive values
     */
    public void debug() {
        SmartDashboard.putNumber("NavX", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Right Encoder", getRightEncoderPosition());
        SmartDashboard.putNumber("Left Encoder", getLeftEncoderPosition());
        SmartDashboard.putNumber("Odometry X", getRobotPos().getX());
        SmartDashboard.putNumber("Odometry Y", getRobotPos().getY());
        SmartDashboard.putNumber("Right Velocity", rightEncoder.getVelocity());
        SmartDashboard.putNumber("Left Velocity", leftEncoder.getVelocity());
    }

    public void stop() {
        leftFront.setVoltage(0);
        rightFront.setVoltage(0);
    }

    @Override
    public void periodic() {
        odometry.update(getCurrentAngle(), getLeftEncoderPosition(), getRightEncoderPosition());
        debug();
    }
    
}
