package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JackmanVisionSubsystem extends SubsystemBase {


    private static final long            LED_MODE_PIPELINE                    = 0;
    private static final long            LED_MODE_OFF                         = 1;
    @SuppressWarnings("unused")
    private static final long            LED_MODE_BLINK                       = 2;
    @SuppressWarnings("unused")
    private static final long            LED_MODE_ON                          = 3;

    private static final long            CAM_MODE_VISION                      = 0;
    private static final long            CAM_MODE_DRIVER                      = 1;

    // configure more pipelines here
    @SuppressWarnings("unused")
    private static final long            PIPELINE_RETROREFLECTIVE_NOTE_DETECT = 0;
    private static final long            PIPELINE_VISUAL                      = 2;
    private static final long            PIPELINE_NEURALNET_NOTE_DETECT       = 7;

    private static final String MODEL_CLASS_NOTE = "note";
    NetworkTable                         table                                = NetworkTableInstance.getDefault()
            .getTable("limelight-jackman");

    // inputs/configs
    NetworkTableEntry                    ledMode                              = table.getEntry("ledMode");
    NetworkTableEntry                    camMode                              = table.getEntry("camMode");
    NetworkTableEntry                    pipeline                             = table.getEntry("pipeline");

    // output
    NetworkTableEntry                    tv                                   = table.getEntry("tv");
    NetworkTableEntry                    tx                                   = table.getEntry("tx");
    NetworkTableEntry                    ty                                   = table.getEntry("ty");
    NetworkTableEntry                    ta                                   = table.getEntry("ta");

    NetworkTableEntry                    tl                                   = table.getEntry("tl");
    NetworkTableEntry                    cl                                   = table.getEntry("cl");

    NetworkTableEntry                    tclass                               = table.getEntry("tclass");


    public JackmanVisionSubsystem() {
        this.pipeline.setNumber(PIPELINE_NEURALNET_NOTE_DETECT);
        this.camMode.setNumber(CAM_MODE_VISION);
        this.ledMode.setNumber(LED_MODE_PIPELINE);
    }



    public double getTargetAreaPercent() {
        return ta.getDouble(-1.0);
    }

    public boolean isVisionTargetFound() {
        return tv.getDouble(-1) == 1;
    }


    public double getTargetDistanceCm() {
        return -1.0; // fixme: calculate distance; not possible if the limelight is on the moving arm
    }


    public Rotation2d getNoteOffset(){
        if (tclass.getString("").equals(MODEL_CLASS_NOTE)) {
            double angleToTarget = getTargetX();
            return Rotation2d.fromDegrees(angleToTarget);
        }
        else{
            return null;
        }
    }





    @Override
    public void periodic() {
        // read values periodically and post to smart dashboard periodically
        SmartDashboard.putBoolean("LimelightJackman/Target Found", isVisionTargetFound());
        SmartDashboard.putNumber("LimelightJackman/tx-value", tx.getDouble(-1.0));
        SmartDashboard.putNumber("LimelightJackman/ty-value", ty.getDouble(-1.0));
        SmartDashboard.putNumber("LimelightJackman/ta-value", ta.getDouble(-1.0));
        SmartDashboard.putNumber("LimelightJackman/l-value", tl.getDouble(-1.0));
        SmartDashboard.putNumber("LimelightJackman/Cam Mode", camMode.getInteger(-1L));
        SmartDashboard.putNumber("LimelightJackman/LED mode", ledMode.getInteger(-1L));
        SmartDashboard.putNumber("LimelightJackman/Pipeline", pipeline.getInteger(-1L));
    }

    /**
     * Get the limelight coordinates for the target (i.e. with respect to the limelight origin, NOT
     * the robot!!)
     *
     * @return limelight target coordinates
     */
    private double[] getTarget() {
        double[] d = new double[2];
        d[0] = tx.getDouble(-1.0);
        d[1] = ty.getDouble(-1.0);
        return d;
    }

    /**
     * Get the limelight X angle measurement to the target.
     *
     * @return limelight X target coordinates
     */
    private double getTargetX() {
        return tx.getDouble(-1.0);
    }

    /**
     * Get the limelight Y angle measurement to the target.
     *
     * @return limelight Y target coordinates
     */
    private double getTargetY() {
        return ty.getDouble(-1.0);
    }



}