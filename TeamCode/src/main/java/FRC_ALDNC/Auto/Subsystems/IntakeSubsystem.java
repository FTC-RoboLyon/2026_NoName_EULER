package FRC_ALDNC.Auto.Subsystems;

import FRC_ALDNC.Auto.Constant;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    HardwareMap hmap;
    private double powerIntake;
    DcMotorEx motorIntake;
    Telemetry telemetry;
    public IntakeSubsystem(HardwareMap hmap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.hmap = hmap;
        motorIntake = hmap.get(DcMotorEx.class, Constant.INTAKE);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void configureIntake(String intakeOuPas){
        if(intakeOuPas.equals("Intake")){
            powerIntake = 0.7;
        }else if(intakeOuPas.equals("Reject")){
            powerIntake = -0.7;
        }else {
            powerIntake = 0.0;
        }
    }

    @Override
    public void periodic() {
        motorIntake.setPower(powerIntake);
    }
}
