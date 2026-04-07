package org.firstinspires.ftc.teamcode.euler.compass;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Boussole intégré au control hub
 */
public class Compass {
    private final IMU imu;

    public Compass(IMU imu) {
        this.imu = imu;

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        this.imu.initialize(parameters);
    }

    public double getHeading(AngleUnit unit) {
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }
}
