package org.firstinspires.ftc.teamcode.CommandBase.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.CommandBase.robotContainer;
import org.firstinspires.ftc.teamcode.Lib.utils;

import java.util.function.Supplier;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotor intakeMotor;
    private robotContainer robot;
    private Supplier<Float> intakeSupplier;
    private Supplier<Float> ejectSupplier;

    public final static double INTAKE_VOLTAGE_SETPOINT = 10; //TUNEME



    public enum WanteState {
        IDLE,
        INTAKE,
        EJECT
    }
    public enum ControlMode { //C bien deja y'a les Control mode mais dcp fait comme sur le shooter et utilise l'enum Control Mode general
        DISABLE,
        MANUAL_VOLTAGE, //normalement voltage c'est vrm quand tu donne une valeur en volt, en FTC on donne plutot une valeur entre 1 et -1 et c'est ce qu'on appelle le duty Cycle (en tt cas Adam l'a appelé comme ça)
        AUTO_ON_OFF //Ca c pas un Control Mode de la liste Auto ca ne dit rien de comment on donne la consigne, la ca serait plutot juste DUTY_CYCLE (pas de Manual suggere l'auto, a la limite sinon VOLTAGE mais en lien avec le commentaire juste au-dessus)
    }
    public enum SystemState {
        STAND_BY, //C'est l'inverse IDLE et STAND_BY, on veut qu'il stand_by (attende) donc il est IDLE (innocupé)
        INTAKING,
        EJECTING
    }
    private WanteState intakeWanteState = WanteState.IDLE;
    private SystemState intakeSystemState = SystemState.STAND_BY;
    private ControlMode intakeControlMode = ControlMode.DISABLE;

    public void setIntakeControlMode(ControlMode controlMode){intakeControlMode = controlMode;}
    public void setIntakeWantedState(WanteState intakeWanteState) {this.intakeWanteState = intakeWanteState;} //pas besoin de préciser intake dans le nom de tes fonctions, on le sait deja car l'objet lui-meme est un intake

    public ControlMode getIntakeControlMode() {
        return intakeControlMode;
    }
    public SystemState getIntakeSystemState() {
        return intakeSystemState;
    }

    public IntakeSubsystem(HardwareMap hmap, robotContainer robot){

        this.robot = robot;

        intakeMotor = hmap.get(DcMotor.class, "Intake"); //normalement dans le nom du device on precisera ce que c'est donc plutot IntakeMotor

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (intakeSupplier == null) //comment il pourraient ne pas être nules si tu viens de les creer ?
            intakeSupplier = ()-> 0f;

        if (ejectSupplier == null)
            ejectSupplier = ()-> 0f;

    }
    public void setSuppliers(Supplier<Float> intakeSupplier, Supplier<Float> ejectSupplier){
        this.intakeSupplier = intakeSupplier;
        this.ejectSupplier = ejectSupplier;
    }

    @Override
    public void periodic(){

        switch (intakeControlMode)
        { //je trouve plus facile a lire comme ca
            case DISABLE:
                break;
            case MANUAL_VOLTAGE:
                intakeMotor.setPower(
                        utils.getVoltageCompensated(
                                intakeSupplier.get() - ejectSupplier.get(),
                                robot.getVoltage(),
                                INTAKE_VOLTAGE_SETPOINT
                        )
                );
                break;

            case AUTO_ON_OFF:

                RunStateMachine();

                switch (intakeSystemState) {
                case STAND_BY:
                    intakeMotor.setPower(0.0);
                    break;
                case INTAKING:
                    intakeMotor.setPower(utils.getVoltageCompensated(1.0, robot.getVoltage(), INTAKE_VOLTAGE_SETPOINT));
                    break;
                case EJECTING:
                    intakeMotor.setPower(utils.getVoltageCompensated(-1.0, robot.getVoltage(), INTAKE_VOLTAGE_SETPOINT));
                    break;
                    //et le default ?
            }
            break;
            //et le default ?

        }
    }
    private void RunStateMachine(){
        switch (intakeWanteState){
            case IDLE:
                intakeSystemState = SystemState.STAND_BY;
                break;
            case INTAKE:
                intakeSystemState = SystemState.INTAKING;
                break;
            case EJECT:
                intakeSystemState = SystemState.EJECTING;
                break;
                //defalut ?
        }

        switch (intakeSystemState){
            case STAND_BY:
            case INTAKING:
            case EJECTING:
                break;// et le default ?
        }
    }
}
