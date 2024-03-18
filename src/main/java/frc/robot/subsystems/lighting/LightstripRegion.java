package frc.robot.subsystems.lighting;

import frc.robot.subsystems.lighting.pattern.LightingPattern;
import frc.robot.subsystems.lighting.pattern.NoPattern;
import frc.robot.telemetry.Telemetry;

import java.util.LinkedHashMap;

public class LightstripRegion {
    public final String                                  name;
    public final int                                     start;
    public final int                                     length;
    private final LinkedHashMap<String, LightingPattern> activePatterns;
    public final NoPattern                               blank;

    public LightstripRegion(String name, int start, int length) {
        this.name           = name;
        this.start          = start;
        this.length         = length;
        this.activePatterns = new LinkedHashMap<>();
        this.blank          = new NoPattern(length);
    }

    public void addPattern(LightingPattern pattern) {

        // If the pattern doesn't fit, log a warning and ignore it.
        if (pattern.length != length) {
            Telemetry.light.lightstripRegionWarning = String.format(
                "Pattern %s has a length of %d but the region %s has a length of %d.  IGNORING PATTERN.",
                pattern.getClass().getSimpleName(), pattern.length, name, length);
            return;
        }

        String key = pattern.getClass().getName();
        // reuse previous pattern if already there, to preserve state of an
        // active pattern
        if (activePatterns.containsKey(key)) {
            LightingPattern p = activePatterns.get(key);
            // reorder in map to end.
            activePatterns.remove(key);
            activePatterns.put(key, p);
        }
        else {
            activePatterns.put(key, pattern);
        }
    }

    public void removePattern(Class<? extends LightingPattern> patternClass) {
        activePatterns.remove(patternClass.getName());
    }

    public void setPattern(LightingPattern pattern) {
        activePatterns.clear();
        addPattern(pattern);
    }

    public LightingPattern getPattern() {

        // none - return an empty pattern
        if (activePatterns.isEmpty()) {
            return blank;
        }

        // just one - return it
        if (activePatterns.size() == 1) {
            return activePatterns.values().iterator().next();
        }

        // more than one - return the last one and log a warning
        LightingPattern result = null;
        StringBuilder   errmsg = new StringBuilder("More than one active pattern for region ").append(name).append(": ");
        for (LightingPattern pattern : activePatterns.values()) {
            if (result == null) {
                errmsg.append(pattern.getClass().getSimpleName());
            }
            else {
                errmsg.append(", ").append(pattern.getClass().getSimpleName());
            }
            result = pattern;
        }
        errmsg.append(". Using the last version.");
        Telemetry.light.lightstripRegionWarning = errmsg.toString();
        return result;

    }

}
