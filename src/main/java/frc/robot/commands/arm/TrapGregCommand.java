package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;


public class TrapGregCommand extends ArmBaseCommand {

    private enum State {
        UNLOCK, MOVE_BOTH, CLIMBERS_UP, REVERSE_NOTE, CHARGE_INTAKE, RELEASE_TRAP, FINISHED
    };

    private TrapGregCommand.State   state = TrapGregCommand.State.UNLOCK;

    private final ClimbSubsystem    climbSubsystem;
    private final LightingSubsystem lightingSubsystem;
    private double                  intakeStartPose;
    private double                  shooterStartPose;


    public TrapGregCommand(ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem, LightingSubsystem lightingSubsystem) {
        super(armSubsystem);
        this.climbSubsystem    = climbSubsystem;
        this.lightingSubsystem = lightingSubsystem;
        addRequirements(climbSubsystem, lightingSubsystem);

    }

    @Override
    public void initialize() {

        // TODO: add lighting
        intakeStartPose  = armSubsystem.getIntakePosition();
        shooterStartPose = armSubsystem.getShooterPosition();


    }

    @Override
    public void execute() {

        switch (state) {

        case UNLOCK:

            armSubsystem.setLinkPivotSpeed(.5);

            if (armSubsystem.getLinkAngle() > Constants.ArmConstants.UNLOCK_POSITION.linkAngle) {
                state = state.MOVE_BOTH;
            }
            break;

        case MOVE_BOTH:

            if (driveToArmPosition(Constants.ArmConstants.INVERSE_TRAP_ARM_POSITION, 2, 2)) {
                state = State.CLIMBERS_UP;
            }

            break;

        case CLIMBERS_UP:

            climbSubsystem.setClimbSpeeds(1, 1);

            if (climbSubsystem.isLeftClimbAtMax() && climbSubsystem.isRightClimbAtMax()) {
                state = State.REVERSE_NOTE;
            }

            break;

        case REVERSE_NOTE:

            if (Math.abs(armSubsystem.getIntakePosition() - intakeStartPose) > 2) {
                armSubsystem.setIntakeSpeed(0);

                if (Math.abs(armSubsystem.getShooterPosition() - shooterStartPose) > 1) {
                    armSubsystem.setShooterSpeed(0);
                    state = State.CHARGE_INTAKE;
                }
                else {
                    armSubsystem.setShooterSpeed(-.05, -.05);

                }
            }
            else {
                armSubsystem.setIntakeSpeed(.075);
            }

            break;
        case RELEASE_TRAP:

            armSubsystem.releaseTrap();
            state = State.CHARGE_INTAKE;
            break;

        case CHARGE_INTAKE:

            armSubsystem.setIntakeSpeed(-.5);
            state = State.FINISHED;
            break;

        case FINISHED:

            break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }

}
