package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

// this subsystem is for the arm, it's disabled for now due to us not having an arm motor.
public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax armMotor;
    private RelativeEncoder armEncoder;
    // private ArmFeedforward armFeedforward;
    // private ProfiledPIDController motionController;
    // private double input;
    // private double setpoint;

    /** Creates a new ArmSubsystem. */
    public ArmSubsystem() {
        armMotor = new CANSparkMax(Constants.DriveConstants.kArmMotorPort, MotorType.kBrushless);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setSmartCurrentLimit(80);
        armMotor.setInverted(false);
        // motionController = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(2.5, 3));
        // armFeedforward = new ArmFeedforward(0, 2.5, 0.97, 0.20);
        
        armEncoder = armMotor.getEncoder();
        armEncoder.setPositionConversionFactor(50);
        armEncoder.setVelocityConversionFactor(50 / 60);
        armEncoder.setPosition(0);

        // setpoint = Constants.DriveConstants.kArmSetpoint;
    }

    public void setArmSpeed(double speed) {
        double armSpeed = speed;
        armMotor.set(armSpeed);
    }

    public void setVoltage(double voltage) {
        armMotor.setVoltage(voltage);
    }

    public void debug() {
        SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition());
    }

    public void setArmPosition(double position) {
        // setpoint = position;
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // input = motionController.calculate(armEncoder.getPosition(), setpoint);
        
        // double betterFeedforward = armFeedforward.calculate(motionController.getSetpoint().position, motionController.getSetpoint().velocity);
        // setVoltage((MathUtil.clamp(input + betterFeedforward, -12, 12)) / 4);

        debug();
    }
    
}
