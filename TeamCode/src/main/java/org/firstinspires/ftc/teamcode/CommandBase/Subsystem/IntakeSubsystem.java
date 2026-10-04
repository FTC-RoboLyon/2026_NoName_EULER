package org.firstinspires.ftc.teamcode.CommandBase.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotor intakeMotor;
    private Gamepad gamepad;



    public enum IntakeMode{
        DISABLED,
        WITH_GAMEPAD,
        INTAKING,
        EJECTING
    }
    private IntakeMode intakeMode = IntakeMode.DISABLED;

    public void setIntakeMode(IntakeMode intakeMode) {this.intakeMode = intakeMode;}
    public IntakeMode getIntakeMode(){return intakeMode;}

    public IntakeSubsystem(HardwareMap hmap, Gamepad gamepad1){
        intakeMotor = hmap.get(DcMotor.class, "Intake");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        gamepad = gamepad1;
    }



    @Override
    public void periodic(){
        switch (intakeMode){
            case DISABLED:
                intakeMotor.setPower(0.0);
                break;
            case WITH_GAMEPAD:
                intakeMotor.setPower(gamepad.right_trigger - gamepad.left_trigger);
                break;
            case INTAKING:
                intakeMotor.setPower(1);
                break;
            case EJECTING:
                intakeMotor.setPower(-1);
                break;
        }
    }
}
