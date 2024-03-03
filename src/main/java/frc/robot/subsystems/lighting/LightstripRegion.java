package frc.robot.subsystems.lighting;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LightstripRegion {
    public int start;
    public int end;

    public LightstripRegion(int start, int end) {
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
