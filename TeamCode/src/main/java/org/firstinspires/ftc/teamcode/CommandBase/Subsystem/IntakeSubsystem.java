package org.firstinspires.ftc.teamcode.CommandBase.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotor intakeMotor;
    private Gamepad gamepad; //-> are you sure about this



    public enum IntakeMode{ //Souviens toi il n'y a que la base qui a un enum mode les autres c WantedState et SystemState
        DISABLED,
        WITH_GAMEPAD,
        INTAKING,
        EJECTING
    }
    private IntakeMode intakeMode = IntakeMode.DISABLED;

    public void setIntakeMode(IntakeMode intakeMode) {this.intakeMode = intakeMode;}
    public IntakeMode getIntakeMode(){return intakeMode;}

    public IntakeSubsystem(HardwareMap hmap){
        intakeMotor = hmap.get(DcMotor.class, "Intake");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //->tu as oublie un detail de la config moteur : il y en a normalement deux min ou 3 : Direction, ZeroPowerBehavior, Mode (des fois optionnel mais ne coute rien)

    }
    public void setGamepad(Gamepad gamepad1){
        gamepad = gamepad1; //->depuis quand on donne l'acces direct de l'intake au gamepad, je crois pas que la base ait un acces direct au joysticks...
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

                //Donc tu donne toujour le max que fournit ta batterie a l'intake (en voltage).
                //C'est aussi a ca que sert un voltage compensation : brider un moteur a un certain voltage parce qu'on considère qu'il n'a pas besoin de
                // plus pour faire son boulot correctement pour preserver la batterie et mettre toute l'energie dans la base et le shooter par exemple qui doivent aller vite
        }
    }
}
