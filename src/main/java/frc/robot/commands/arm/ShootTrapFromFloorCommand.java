package frc.robot.commands.arm;

import static frc.robot.Constants.LightingConstants.SIGNAL;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.lighting.pattern.Shooting;
import frc.robot.subsystems.swerve.SwerveSubsystem;

// Shoot. That's it.
public class ShootTrapFromFloorCommand extends ArmBaseCommand {

    private enum State {
        START_SHOOTER, START_FEEDER, FINISHED
    };

    private State             state               = State.START_SHOOTER;
    private LightingSubsystem lighting;

    private double            startIntakePosition = 0;

    private OperatorInput     operatorInput;
    private SwerveSubsystem     swerveSubsystem;

    NetworkTable table                       = NetworkTableInstance.getDefault().getTable("Testing");
    NetworkTableEntry bottomMotorSpeed       = table.getEntry("bottomMotorSpeed");
    NetworkTableEntry topMotorSpeed          = table.getEntry("topMotorSpeed");


    public ShootTrapFromFloorCommand(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem, LightingSubsystem lighting, OperatorInput operatorInput) {

        super(armSubsystem);
        this.swerveSubsystem = swerveSubsystem;
        this.lighting = lighting;
        this.operatorInput = operatorInput;
    }

    @Override
    public void initialize() {

        state               = State.START_SHOOTER;

        startIntakePosition = armSubsystem.getIntakePosition();

        logCommandStart("Intake Position " + startIntakePosition);
        lighting.addPattern(SIGNAL, Shooting.getInstance());
    }

    @Override
    public void execute() {

        double intakeSpeed  = 0;
        double shooterSpeed = 0;

        switch (state) {

        case START_SHOOTER:

            armSubsystem.setIntakeSpeed(0);
            armSubsystem.setShooterSpeed(topMotorSpeed.getDouble(0.4), bottomMotorSpeed.getDouble(0.5));

            // Wait for the shooter to get up to speed
            if (isStateTimeoutExceeded(.75)) {
                StringBuilder sb = new StringBuilder("Shooter up to speed.");
                sb.append(" TopShooter ")
                        .append(String.format("%.2f", armSubsystem.getTopShooterEncoderSpeed()))
                        .append(" BottomShooter ")
                        .append(String.format("%.2f", armSubsystem.getBottomShooterEncoderSpeed()))
                        .append(" BotPose " )
                        .append(swerveSubsystem.getPose().getTranslation());
                logStateTransition("Start Shooter -> Shoot", sb.toString());;
                state = State.START_FEEDER;
            }

            break;

        case START_FEEDER:

            armSubsystem.setIntakeSpeed(1);

            if (isStateTimeoutExceeded(.5)) {
                logStateTransition("Shoot -> Finished", "Shot fired");
                state = State.FINISHED;
            }
            break;

        default:
            break;
        }
    }

    @Override
    public boolean isFinished() {

        if (state == State.FINISHED) {
            return true;
        }
        return false;
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        armSubsystem.setIntakeSpeed(0);
        armSubsystem.setShooterSpeed(0);
        lighting.removePattern(Shooting.class);
        logCommandEnd(interrupted);

        if (!interrupted) {
            if (DriverStation.isTeleop()) {
                CommandScheduler.getInstance().schedule(new CompactCommand(armSubsystem));
            }

        }
    }

}
