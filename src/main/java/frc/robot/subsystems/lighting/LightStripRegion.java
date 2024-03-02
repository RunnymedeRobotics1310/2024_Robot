package frc.robot.subsystems.lighting;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LightStripRegion {
    public int start;
    public int end;

    public LightStripRegion(int start, int end) {
        this.start = start;
        this.end   = end;
    }

    public int getLength() {
        return end - start;
    }

    public AddressableLEDBuffer createBuffer() {
        return new AddressableLEDBuffer(getLength());
    }

}
