package frc.robot.commands.arm;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ArmSubsystem;

// Shoot. That's it.
public class ShootCommand extends LoggingCommand {

	private final ArmSubsystem armSubsystem;
	
    public ShootCommand(ArmSubsystem armSubsystem) {

    	this.armSubsystem = armSubsystem;
    			
    	addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

    	logCommandStart();
    }

    @Override
    public void execute() {

    	armSubsystem.setShooterSpeed(.3);
    	
    	if (isTimeoutExceeded(1.5)) {
    		armSubsystem.setIntakeSpeed(.3);
    	}
    }
    
    public boolean isFinished() {
    	
    	if (isTimeoutExceeded(4)) {
    		return true;
    	}
    	return false;
    }
    
    public void end(boolean interrupted) {
    	logCommandEnd(interrupted);
    	
    	armSubsystem.stop();
    }


}
