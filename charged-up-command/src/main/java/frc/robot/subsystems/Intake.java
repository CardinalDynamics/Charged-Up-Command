package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotorL, intakeMotorR;

    public Intake() {
        intakeMotorL = new CANSparkMax(Constants.IntakeConstants.intakeMotorPortL, CANSparkMaxLowLevel.MotorType.kBrushless);
        intakeMotorR = new CANSparkMax(Constants.IntakeConstants.intakeMotorPortR, CANSparkMaxLowLevel.MotorType.kBrushless);

        intakeMotorR.follow(intakeMotorL);

        intakeMotorL.setInverted(false);
        intakeMotorR.setInverted(true);

        intakeMotorL.setIdleMode(IdleMode.kBrake);
        intakeMotorR.setIdleMode(IdleMode.kBrake);

        intakeMotorL.setSmartCurrentLimit(80);
        intakeMotorR.setSmartCurrentLimit(80);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotorL.set(speed);
    }

    public void stopIntake() {
        intakeMotorL.set(0);
    }

    public void toggleIntake() {
        if (intakeMotorL.get() == 0) {
            setIntakeSpeed(.5);
        } else {
            stopIntake();
        }
    }

    public void periodic() {
        // This method will be called once per scheduler run
    }
}
