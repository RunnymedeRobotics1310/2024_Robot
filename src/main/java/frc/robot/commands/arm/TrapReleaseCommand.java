package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;


public class TrapReleaseCommand extends ArmBaseCommand {
    private final LightingSubsystem lightingSubsystem;

    public TrapReleaseCommand(ArmSubsystem armSubsystem, LightingSubsystem lightingSubsystem) {
        super(armSubsystem);
        this.lightingSubsystem = lightingSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(lightingSubsystem);
    }

    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {
        armSubsystem.releaseTrap();
        // if (armSubsystem.trapReleased()) {
        // armSubsystem.setShooterSpeed(-.5, -.5);
    }


    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return isTimeoutExceeded(0.5);
    }

    @Override
    public void end(boolean interrupted) {

        armSubsystem.stop();
    }
}
