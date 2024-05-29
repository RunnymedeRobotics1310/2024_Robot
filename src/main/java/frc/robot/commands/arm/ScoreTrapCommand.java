package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class ScoreTrapCommand extends ArmBaseCommand {

    private enum State {
        ARM_FORWARDS_AND_CLIMB, CHARGE_INTAKE_AND_RELEASE_TRAP, SCORE_TRAP, FINISHED
    };

    private State   state = State.ARM_FORWARDS_AND_CLIMB;
    ClimbSubsystem climbSubsystem;

    public ScoreTrapCommand(ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem) {
        super(armSubsystem);
        this.climbSubsystem = climbSubsystem;
    }

    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {

        switch (state) {
            case ARM_FORWARDS_AND_CLIMB:

                armSubsystem.setAimPivotSpeed(0.5);
                armSubsystem.setLinkPivotSpeed(-0.5);
                climbSubsystem.setClimbSpeeds(-1, -1);

                if (climbSubsystem.leftAllTheWayDown() && climbSubsystem.rightAllTheWayDown()) {
                    armSubsystem.stop();
                    state = State.CHARGE_INTAKE_AND_RELEASE_TRAP;
                }
                break;

            case CHARGE_INTAKE_AND_RELEASE_TRAP:
                armSubsystem.setIntakeSpeed(-1);
                armSubsystem.releaseTrap();

                if (armSubsystem.getIntakeEncoderSpeed() > 50) {
                    state = State.SCORE_TRAP;
                }


                break;

            case SCORE_TRAP:
                armSubsystem.setShooterSpeed(-.25, -.25);

                long shooterStart = System.currentTimeMillis();

                if (System.currentTimeMillis() - shooterStart > 2/1000) {
                    state = State.FINISHED;
                }

                break;

            case FINISHED:
                armSubsystem.stop();
        }

    }

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }

    @Override
    public void end(boolean interrupted) {

        armSubsystem.stop();
        climbSubsystem.stop();

    }
}
