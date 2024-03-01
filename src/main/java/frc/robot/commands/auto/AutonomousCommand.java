package frc.robot.commands.auto;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants.AutoPattern;

public class AutonomousCommand extends SequentialCommandGroup {

    private AutoPattern        autoPattern = null;

    private Optional<Alliance> alliance    = null;

    public AutonomousCommand(
        SendableChooser<AutoPattern> autoPatternChooser) {

        // Default is to do nothing.
        // If more commands are added, the instant command will end and
        // the next command will be executed.
        addCommands(new InstantCommand());

        autoPattern = autoPatternChooser.getSelected();

        alliance    = DriverStation.getAlliance();

        if (!alliance.isPresent()) {
            System.out.println("*** ERROR **** unknown Alliance ");
        }

        StringBuilder sb = new StringBuilder();
        sb.append("Auto Selections");
        sb.append("\n   Auto Pattern  : ").append(autoPattern);
        sb.append("\n   Alliance      : ").append(alliance);

        System.out.println(sb.toString());

        // If any inputs are null, then there was some kind of error.
        if (autoPattern == null) {
            System.out.println("*** ERROR - null found in auto pattern builder ***");
            return;
        }

        /*
         * Compose the appropriate auto commands
         */
        switch (autoPattern) {

        case DO_NOTHING:
            return;

        case DRIVE_FORWARD:
            return;
        // What should we put here?
        // Drive forward for 1 second
        // addCommands(new DriveForwardCommand(1, driveSubsystem));
        case THREE_NOTE:
            addCommands(

            );
        }
    }

}

