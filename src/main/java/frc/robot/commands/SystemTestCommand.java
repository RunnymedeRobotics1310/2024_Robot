package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.operator.GameController;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

/**
 * This command is used to safely stop the robot in its current position, and to cancel any running
 * commands
 */
public class SystemTestCommand extends LoggingCommand {

    private enum Motor {
        NONE,
        // Arm Motors
        LINK, AIM, AIM_AND_LINK, INTAKE, SHOOTER
    }

    private final OperatorInput  operatorInput;
    private final ArmSubsystem   armSubsystem;
    private final GameController controller;

    private long                 startTime           = 0;
    private Motor                selectedMotor       = Motor.NONE;
    private double               povMotorSpeed       = 0;
    private double               povMotorSpeed2      = 0;

    private boolean              previousLeftBumper  = false;
    private boolean              previousRightBumper = false;

    /**
     * System Test Command
     *
     * All subsystems must be passed to this command, and each subsystem should have a stop command
     * that safely stops the robot
     * from moving.
     */
    public SystemTestCommand(OperatorInput operatorInput,
        ArmSubsystem armSubsystem) {

        this.operatorInput = operatorInput;
        this.armSubsystem  = armSubsystem;

        controller         = operatorInput.getDriverController();

        addRequirements(armSubsystem);
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        /*
         * The SystemTestCommand is not interruptible, and prevents all other commands that try to
         * interrupt it. Only the cancel button will end the SystemTestCommand.
         */
        return InterruptionBehavior.kCancelIncoming;
    }

    @Override
    public void initialize() {

        logCommandStart();

        stopAllMotors();

        startTime = System.currentTimeMillis();

        SmartDashboard.putBoolean("Test Mode", true);
        SmartDashboard.putString("Test Motor", selectedMotor.toString());
        SmartDashboard.putNumber("Test Motor Speed", povMotorSpeed);
        SmartDashboard.putNumber("Test Motor Speed 2", povMotorSpeed2);

        clearMotorIndicators();
    }

    @Override
    public void execute() {

        /*
         * Use the bumpers to select the next / previous motor in the motor ring.
         *
         * Switching motors will cause all motors to stop
         */

        boolean rightBumper = controller.getRightBumper() && !previousRightBumper;
        previousRightBumper = controller.getRightBumper();

        boolean leftBumper = controller.getLeftBumper() && !previousLeftBumper;
        previousLeftBumper = controller.getLeftBumper();

        if (rightBumper || leftBumper) {

            int nextMotorIndex = selectedMotor.ordinal();

            if (rightBumper) {

                // Select the next motor in the ring
                nextMotorIndex = (nextMotorIndex + 1) % Motor.values().length;
            }
            else {

                // Select the previous motor in the ring
                nextMotorIndex--;
                if (nextMotorIndex < 0) {
                    nextMotorIndex = Motor.values().length - 1;
                }
            }

            clearMotorIndicators();
            stopAllMotors();

            selectedMotor = Motor.values()[nextMotorIndex];

            SmartDashboard.putString("Test Motor", selectedMotor.toString());
        }

        /*
         * The SystemTestCommand can use either the POV or the triggers to control
         * the motor speed. If the triggers are used, the POV is cleared.
         *
         * Once the motor is selected, use the POV up and down to
         * adjust the motor speed.
         *
         * The speed is adjusted 50 times / second as the user holds the
         * POV control. Allow a 5 seconds to ramp the speed from 0 to full value.
         *
         * increment = 1.0 (full) / 50 adjustments/sec / 5 sec = .004 adjustment size / loop.
         */

        int    pov          = controller.getPOV();
        double leftTrigger  = controller.getLeftTriggerAxis();
        double rightTrigger = controller.getRightTriggerAxis();

        double motorSpeed   = 0;
        double motorSpeed2  = 0;

        if (leftTrigger > 0 && rightTrigger > 0) {

            // If both triggers are pressed, then stop the motor
            motorSpeed    = 0;
            povMotorSpeed = 0;
        }
        else if (leftTrigger > 0) {

            motorSpeed    = -leftTrigger;
            povMotorSpeed = 0;
        }
        else if (rightTrigger > 0) {

            motorSpeed    = rightTrigger;
            povMotorSpeed = 0;
        }
        else {

            // No triggers are pressed, use the POV to control the motor speed
            if (pov == 0) {

                povMotorSpeed += 0.004;

                if (povMotorSpeed > 1.0) {
                    povMotorSpeed = 1.0;
                }
            }

            if (pov == 180) {

                povMotorSpeed -= 0.004;

                if (povMotorSpeed < -1.0) {
                    povMotorSpeed = -1.0;
                }
            }

            motorSpeed = povMotorSpeed;
        }

        // Use the POV left right to control the second motor speed
        if (pov == 90) {

            povMotorSpeed2 += 0.004;

            if (povMotorSpeed2 > 1.0) {
                povMotorSpeed2 = 1.0;
            }
        }

        if (pov == 270) {

            povMotorSpeed2 -= 0.004;

            if (povMotorSpeed2 < -1.0) {
                povMotorSpeed2 = -1.0;
            }
        }

        motorSpeed  = povMotorSpeed;
        motorSpeed2 = povMotorSpeed2;

        SmartDashboard.putNumber("Test Motor Speed", motorSpeed);
        SmartDashboard.putNumber("Test Motor Speed 2", motorSpeed2);

        /*
         * If the X button is pressed, reset the motor speed to zero
         */

        if (controller.getXButton()) {
            motorSpeed     = 0;
            motorSpeed2    = 0;
            povMotorSpeed  = 0;
            povMotorSpeed2 = 0;
        }

        /*
         * Apply the motor speed to the selected motor
         */

        SmartDashboard.putNumber("Test Motor Speed", motorSpeed);

        switch (selectedMotor) {

        case NONE:
            break;

        case LINK:
            armSubsystem.setLinkPivotSpeed(motorSpeed);
            SmartDashboard.putBoolean("Test Link", true);
            break;

        case AIM:
            armSubsystem.setAimPivotSpeed(motorSpeed);
            SmartDashboard.putBoolean("Test Aim", true);
            break;

        case AIM_AND_LINK:
            armSubsystem.setArmSpeeds(motorSpeed, motorSpeed2);
            SmartDashboard.putBoolean("Test Link", true);
            SmartDashboard.putBoolean("Test Aim", true);
            break;

        case INTAKE:
            armSubsystem.setIntakeSpeed(motorSpeed);
            SmartDashboard.putBoolean("Test Intake", true);
            break;

        case SHOOTER:
            armSubsystem.setShooterSpeed(motorSpeed);
            SmartDashboard.putBoolean("Test Shooter", true);
            break;
        }
    }

    @Override
    public boolean isFinished() {

        // Wait 1/2 second before finishing.
        // This allows the user to start this command using the start and back
        // button combination without cancelling on the start button as
        // the user releases the buttons
        if (System.currentTimeMillis() - startTime < 500) {
            return false;
        }

        // Cancel on the regular cancel button after the first 0.5 seconds
        if (operatorInput.isCancelPressed()) {
            setFinishReason("Cancelled by driver controller");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        stopAllMotors();

        selectedMotor = Motor.NONE;

        SmartDashboard.putBoolean("Test Mode", false);
        SmartDashboard.putString("Test Motor", selectedMotor.toString());
        SmartDashboard.putNumber("Test Motor Speed", 0);

        logCommandEnd(interrupted);
    }

    private void clearMotorIndicators() {
        SmartDashboard.putBoolean("Test Link", false);
        SmartDashboard.putBoolean("Test Aim", false);
        SmartDashboard.putBoolean("Test Intake", false);
        SmartDashboard.putBoolean("Test Shooter", false);
    }

    // Safely stop all motors in all subsystems used by this command.
    private void stopAllMotors() {

        armSubsystem.stop();
        povMotorSpeed = 0;
    }
}
