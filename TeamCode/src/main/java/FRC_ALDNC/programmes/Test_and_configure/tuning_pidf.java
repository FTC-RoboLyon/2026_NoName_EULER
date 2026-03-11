package FRC_ALDNC.programmes.Test_and_configure;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.SHOOTER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class tuning_pidf extends LinearOpMode {
    DcMotorEx shooter;
    PIDFCoefficients pidf_tuning;


    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER);
    }
}
