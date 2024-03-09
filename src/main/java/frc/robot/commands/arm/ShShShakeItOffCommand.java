package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ShShShakeItOffCommand extends ArmBaseCommand {

    public ShShShakeItOffCommand(ArmSubsystem arm) {
        super(arm);
    }

    private long start = 0;
    private int  count = 0;

    @Override
    public void initialize() {
        start = System.currentTimeMillis();
        count = 0;
    }


    @Override
    public void execute() {
        long   now    = System.currentTimeMillis();

        double factor = 1.0;
        if (now - start > 250) {
            factor *= -1;
            count++;
        }
        armSubsystem.setIntakeSpeed(factor * ArmConstants.INTAKE_EJECT_INTAKE_SPEED);

    }

    @Override
    public boolean isFinished() {
        return count > 10;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
    }
}
