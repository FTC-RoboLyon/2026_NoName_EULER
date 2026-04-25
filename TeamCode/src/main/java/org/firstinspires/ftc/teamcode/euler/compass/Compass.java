package org.firstinspires.ftc.teamcode.euler.compass;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.euler.RobotTelemetry;
import org.firstinspires.ftc.teamcode.euler.TelemetryAware;

import java.util.Locale;

/**
 * Boussole intégré au control hub
 */
public class Compass implements TelemetryAware {
    private final IMU imu;

    public Compass(IMU imu) {
        this.imu = imu;

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        this.imu.initialize(parameters);
        this.imu.resetYaw();
    }

    public double getHeading(AngleUnit unit) {
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }

    @Override
    public RobotTelemetry getTelemetry() {
        return new RobotTelemetry("Compass",
                String.format(Locale.FRENCH, "Heading: %.1f°", getHeading(AngleUnit.DEGREES)));
    }
}
