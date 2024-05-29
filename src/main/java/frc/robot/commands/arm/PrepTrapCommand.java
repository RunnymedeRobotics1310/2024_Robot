package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class PrepTrapCommand extends ArmBaseCommand {

    ClimbSubsystem climbSubsystem;
    double shooterStart;
    boolean noteReversed = false;

    public PrepTrapCommand(ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem) {
        super(armSubsystem);
        this.climbSubsystem = climbSubsystem;
    }

    @Override
    public void initialize() {
        logCommandStart();
        shooterStart = armSubsystem.getShooterPosition();
    }

    @Override
    public void execute() {
        driveToArmPosition(Constants.ArmConstants.SHOOT_AMP_ARM_POSITION, 10, 10);
        climbSubsystem.setClimbSpeeds(1, 1);

        if  (armSubsystem.getShooterPosition() - shooterStart >= .5) {
            armSubsystem.setShooterSpeed(0, 0);
            armSubsystem.setIntakeSpeed(0);
            noteReversed = true;
        }
        else{
            armSubsystem.setShooterSpeed(0.1, 0.1);
            armSubsystem.setIntakeSpeed(0.3);
        }

    }

    @Override
    public boolean isFinished() {
        return isAtArmPosition(Constants.ArmConstants.SHOOT_AMP_ARM_POSITION, 10)
                && climbSubsystem.isLeftClimbAtMax()
                && climbSubsystem.isRightClimbAtMax()
                && noteReversed;
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stop();
        armSubsystem.stop();
        armSubsystem.setAimPivotSpeed(.02);
    }
}
