package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// this subsystem is for the two intake motors on the end of the arm.
public class IntakeSubsystem extends SubsystemBase {
    // private final CANSparkMax intakeMotorL, intakeMotorR;
    private final CANSparkMax intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.DriveConstants.kIntakeMotorPort, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(60);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public void toggleIntake() {
        if (intakeMotor.get() == 0) {
            setIntakeSpeed(.5);
        } else {
            stopIntake();
        }
    }

    public void periodic() {
        // This method will be called once per scheduler run
    }
}
