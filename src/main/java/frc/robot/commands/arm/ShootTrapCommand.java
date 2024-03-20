package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class ShootTrapCommand extends ArmBaseCommand {

    private enum State {
        LOWER_CLIMB, MOVE_ARM, RETRACT_NOTE, START_INTAKE, FEED_NOTE, FINISH
    };

    private State          state;
    private ClimbSubsystem climbSubsystem;
    private double         startShooterPosition;

    public ShootTrapCommand(ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem) {
        super(armSubsystem);

        this.climbSubsystem = climbSubsystem;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {

        logCommandStart();
        state                = State.LOWER_CLIMB;

        startShooterPosition = armSubsystem.getShooterPosition();

    }

    @Override
    public void execute() {

        switch (state) {
        case LOWER_CLIMB:

            climbSubsystem.setClimbSpeeds(-1, -1);

            if (climbSubsystem.leftAllTheWayDown() && climbSubsystem.rightAllTheWayDown()) {
                logStateTransition("Climb to move arm", "climbs down");
                state = State.MOVE_ARM;
            }

            break;

        case MOVE_ARM:

            double linkAngle = armSubsystem.getLinkAngle();
            double aimAngle = armSubsystem.getAimAngle();
            double aimSpeed = 0.2;
            double linkSpeed = -0.3;

            if (aimAngle >= ArmConstants.TRAP_ARM_POSITION.aimAngle) {
                aimSpeed = 0;
            }
            if (linkAngle <= ArmConstants.TRAP_ARM_POSITION.linkAngle) {
                linkSpeed = 0;
            }

            armSubsystem.setAimPivotSpeed(aimSpeed);
            armSubsystem.setLinkPivotSpeed(linkSpeed);

            if (aimAngle >= ArmConstants.TRAP_ARM_POSITION.aimAngle && linkAngle <= ArmConstants.TRAP_ARM_POSITION.linkAngle) {
                logStateTransition("Move arm to retract", "Arm in position");
                state = State.RETRACT_NOTE;
            }

            break;

        case RETRACT_NOTE:

            armSubsystem.setShooterSpeed(0.3);
            armSubsystem.setIntakeSpeed(0.1);

            // Reverse the note for a number of rotations
            if (Math.abs(armSubsystem.getShooterPosition() - startShooterPosition) > 2) {
                armSubsystem.stop();
                logStateTransition("Reverse -> Start Intake", "Shooter Reversed");
                state = State.START_INTAKE;
            }

            break;

        case START_INTAKE:

            armSubsystem.setIntakeSpeed(ArmConstants.INTAKE_TRAP_SPEED);
            armSubsystem.releaseTrap();

            if (isStateTimeoutExceeded(.5)) {
                logStateTransition("Start Intake -> Feed Note", "Intake up to speed " + armSubsystem.getIntakeEncoderSpeed());
                state = State.FEED_NOTE;
            }

            break;

        case FEED_NOTE:

            armSubsystem.setShooterSpeed(-0.5);

            if (isStateTimeoutExceeded(.5)) {
                logStateTransition("Shoot -> Finished", "Trap scored");
                state = State.FINISH;
            }

            break;

        case FINISH:
            break;

        default:
            break;
        }

    }

    @Override
    public boolean isFinished() {

        return state == State.FINISH;

    }

    @Override
    public void end(boolean interrupted) {

        armSubsystem.stop();
        logCommandEnd(interrupted);

    }

}
