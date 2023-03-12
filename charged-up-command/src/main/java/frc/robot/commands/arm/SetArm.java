// package frc.robot.commands.arm;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.ArmSubsystem;

// public class SetArm extends CommandBase {
//     private final double setpoint;
//     private final ArmSubsystem arm;
    

//     public SetArm(ArmSubsystem arm, double setpoint) {
//         this.arm = arm;
//         this.setpoint = setpoint;
//         addRequirements(arm);
//     }

//     @Override
//     public void initialize() {
//         arm.setArmPosition(setpoint);
//     }

//     @Override
//     public void execute() {
//     }

//     @Override
//     public boolean isFinished() {
//         return true;
//     }

//     @Override
//     public void end(boolean inturrupted) {

//     }
// }
