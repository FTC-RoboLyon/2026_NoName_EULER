package org.firstinspires.ftc.teamcode.euler;

public class RobotTelemetry {
    private final String caption;
    private final String value;

    public RobotTelemetry(String caption, String value) {
        this.caption = caption;
        this.value = value;
    }

    public String getCaption() {
        return caption;
    }

    public String getValue() {
        return value;
    }
}
