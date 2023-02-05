package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    private CANSparkMax leftFront;
    private CANSparkMax leftBack;
    private CANSparkMax rightFront;
    private CANSparkMax rightBack;

    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;

    private DifferentialDrive drive;

    private AnalogGyro gyro;

    public DriveSubsystem() {
        super();
        
        leftFront = new CANSparkMax(1, MotorType.kBrushless);
        leftBack = new CANSparkMax(2, MotorType.kBrushless);
        rightFront = new CANSparkMax(3, MotorType.kBrushless);
        rightBack = new CANSparkMax(4, MotorType.kBrushless);

        leftMotors = new MotorControllerGroup(leftFront, leftBack);
        rightMotors = new MotorControllerGroup(rightFront, rightBack);

        rightMotors.setInverted(true);

        drive = new DifferentialDrive(leftMotors, rightMotors);

        leftFront.setIdleMode(IdleMode.kCoast);
        leftBack.setIdleMode(IdleMode.kCoast);
        rightFront.setIdleMode(IdleMode.kCoast);
        rightBack.setIdleMode(IdleMode.kCoast);

        leftFront.setSmartCurrentLimit(80);
        leftBack.setSmartCurrentLimit(80);
        rightFront.setSmartCurrentLimit(80);
        rightBack.setSmartCurrentLimit(80);

        addChild("Gyro", gyro);
    }

    public void log() {
        SmartDashboard.putNumber("Gyro", gyro.getAngle());
    }

    public void arcadeDrive(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void stop() {
        drive.stopMotor();
    }
    
}
