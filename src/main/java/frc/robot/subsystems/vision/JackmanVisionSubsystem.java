package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.RunnymedeSubsystemBase;
import frc.robot.telemetry.Telemetry;

public class JackmanVisionSubsystem extends RunnymedeSubsystemBase {


    private static final long   LED_MODE_PIPELINE                    = 0;
    private static final long   LED_MODE_OFF                         = 1;
    @SuppressWarnings("unused")
    private static final long   LED_MODE_BLINK                       = 2;
    @SuppressWarnings("unused")
    private static final long   LED_MODE_ON                          = 3;

    private static final long   CAM_MODE_VISION                      = 0;
    private static final long   CAM_MODE_DRIVER                      = 1;

    // configure more pipelines here
    @SuppressWarnings("unused")
    private static final long   PIPELINE_RETROREFLECTIVE_NOTE_DETECT = 0;
    private static final long   PIPELINE_VISUAL                      = 2;
    private static final long   PIPELINE_NEURALNET_NOTE_DETECT       = 7;

    private static final String MODEL_CLASS_NOTE                     = "note";

    private static final double CLOSE_NOTE_AREA_THRESHOLD            = 7.5;

    NetworkTable                table                                = NetworkTableInstance.getDefault()
        .getTable("limelight-jackman");

    // inputs/configs
    NetworkTableEntry           ledMode                              = table.getEntry("ledMode");
    NetworkTableEntry           camMode                              = table.getEntry("camMode");
    NetworkTableEntry           pipeline                             = table.getEntry("pipeline");

    // output
    NetworkTableEntry           tv                                   = table.getEntry("tv");
    NetworkTableEntry           tx                                   = table.getEntry("tx");
    NetworkTableEntry           ty                                   = table.getEntry("ty");
    NetworkTableEntry           ta                                   = table.getEntry("ta");

    NetworkTableEntry           tl                                   = table.getEntry("tl");
    NetworkTableEntry           cl                                   = table.getEntry("cl");

    NetworkTableEntry           tclass                               = table.getEntry("tclass");


    public JackmanVisionSubsystem() {
        this.pipeline.setNumber(PIPELINE_NEURALNET_NOTE_DETECT);
        this.camMode.setNumber(CAM_MODE_VISION);
        this.ledMode.setNumber(LED_MODE_PIPELINE);
    }

    public boolean isNoteClose() {
        return ta.getDouble(-1) >= CLOSE_NOTE_AREA_THRESHOLD;
    }

    public boolean isVisionTargetFound() {
        return tv.getDouble(-1) == 1;
    }

    public Rotation2d getNoteOffset() {
        if (tclass.getString("").equals(MODEL_CLASS_NOTE)) {
            double angleToTarget = getTargetX();
            return Rotation2d.fromDegrees(angleToTarget);
        }
        else {
            return null;
        }
    }



    @Override
    public void periodic() {
        Telemetry.jackman.isVisionTargetFound = isVisionTargetFound();
        Telemetry.jackman.tx                  = tx.getDouble(-1.0);
        Telemetry.jackman.ty                  = ty.getDouble(-1.0);
        Telemetry.jackman.ta                  = ta.getDouble(-1.0);
        Telemetry.jackman.tl                  = tl.getDouble(-1.0);
        Telemetry.jackman.camMode             = camMode.getDouble(-1.0);
        Telemetry.jackman.ledMode             = ledMode.getDouble(-1.0);
        Telemetry.jackman.pipeline            = pipeline.getDouble(-1.0);
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

    @Override
    public String toString() {
        return "Jackman Vision Subsystem";
    }

}