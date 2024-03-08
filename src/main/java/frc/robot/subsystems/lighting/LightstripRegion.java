package frc.robot.subsystems.lighting;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.lighting.pattern.LightingPattern;
import frc.robot.telemetry.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

public class LightstripRegion {
    public String                                  name;
    public int                                     start;
    public int                                     end;

    private final Class<? extends LightingPattern> defaultPatternClass;
    private LightingPattern                        _defaultPattern;
    private final Map<String, LightingPattern>     activePatterns;

    public LightstripRegion(String name, int start, int end, Class<? extends LightingPattern> defaultPatternClass) {
        this.name                = name;
        this.start               = start;
        this.end                 = end;
        this.defaultPatternClass = defaultPatternClass;
        this.activePatterns      = new LinkedHashMap<>();
    }

    /**
     * Lazy initialization of the default pattern
     */
    private LightingPattern getDefaultPattern() {
        if (_defaultPattern == null) {
            try {
                _defaultPattern = (LightingPattern) defaultPatternClass.getMethod("getInstance").invoke(null);
            }
            catch (Exception e) {
                throw new IllegalArgumentException(
                    "Configuration error - cannot create default lighting pattern for region " + name + ": " + e, e);
            }
        }
        return _defaultPattern;
    }

    public int getLength() {
        return end - start;
    }

    public AddressableLEDBuffer createBuffer() {
        return new AddressableLEDBuffer(getLength());
    }

    public void addPattern(LightingPattern pattern) {
        // don't put the default pattern into the map
        String key = pattern.getClass().getName();
        if (key.equals(getDefaultPattern().getClass().getName())) {
            return;
        }
        // reuse previous pattern if already there, to preserve state of an
        // active pattern
        if (activePatterns.containsKey(key)) {
            LightingPattern p = activePatterns.get(key);
            // reorder in map to end.
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

        // none - return the default
        if (activePatterns.isEmpty()) {
            return getDefaultPattern();
        }

        // just one - return it
        if (activePatterns.size() == 1) {
            return activePatterns.values().iterator().next();
        }

        // more than one - return the first one and log a warning
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
