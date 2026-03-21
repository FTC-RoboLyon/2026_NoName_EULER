package FRC_ALDNC.Auto.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.*;

public class NavXSubsystem extends SubsystemBase {

    private NavxMicroNavigationSensor navx;

    private double angle;
    private double offset;

    public NavXSubsystem(HardwareMap hmap) {
        navx = hmap.get(NavxMicroNavigationSensor.class, "navx");
        offset = 0;
    }

    public double getAngle() {
        return angle;
    }

    public void reset_A_MettreDans_L_Init() {
    }

    @Override
    public void periodic() {

        Orientation angles = navx.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.RADIANS
        );


        angle = angles.firstAngle;
    }
}