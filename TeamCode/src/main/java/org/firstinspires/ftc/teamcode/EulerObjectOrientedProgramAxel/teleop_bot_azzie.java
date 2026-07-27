package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import static org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel.Drivetrain.KP_X;
import static org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel.Feeder.FeederActivePos;
import static org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel.Feeder.FeederIdlePos;
import static org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel.allianceShifter.ALLIANCE_KEY;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class teleop_bot_azzie extends OpMode {
    Drivetrain drivetrain;
    Intake intake;
    Shooter shooter;
    Feeder feeder;
    Camera camera;
    private Object alliance; //pourquoi object et pas string ???

    VoltageSensor voltageSensor;

    private boolean searchingForBalls = false;


    @Override
    public void init() {
         drivetrain = new Drivetrain(hardwareMap);
         intake = new Intake(hardwareMap);
         shooter = new Shooter(hardwareMap);
         feeder = new Feeder(hardwareMap);
         camera = new Camera(hardwareMap);

         voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

         alliance = blackboard.get(ALLIANCE_KEY);
         alliance = (String) alliance; //a quoi sers-tu ligne mysterieuse...
    }

    @Override
    public void loop() {
        double voltage = voltageSensor.getVoltage();

        if (gamepad1.rightBumperWasPressed()){
            searchingForBalls = !searchingForBalls;
        }
        camera.setColorProcessorEnabled(searchingForBalls);
        camera.setAprilTagProcessorEnabled(!searchingForBalls);
        //Changer l'etat d'un processeur et potentiellement long donc le faire a chaque boucle n'est pas super (en plus x2)
        //il faudrait s'arranger pour le faire uniquement quand il faut vraiment changer

        if (gamepad1.left_stick_button && camera.getBearing(24) != 7 && !searchingForBalls) { //la y'a pas un probleme par rapport au getBearing() en fonction de l'alliance
            //et d'ailleurs pourquoi ne pas profiter de ton gyro et de ta localisation pour un premier alignement tant qu'il ne voit pas l'april tag
            drivetrain.AlignWithTarget(
                    camera.getBearing(alliance == "red" ? 24 : 21),
                    gamepad1.right_stick_y,
                    -gamepad1.left_stick_x //on a dit que ca devait etre positif a gauche donc -
            );
        }
        else if (searchingForBalls && camera.getBigestBlob() != null) {
            //Alors la j'avoue ton code commence à ne plus vraiment avoir de sens prc qu'il s'aligne pour les goals de DECODE mais cherche les ball de biobuzz
            //il faut les separer dans differents OpMode (et meme dans plusieurs projets d'ou travailler dans l'autre repo)
            drivetrain.Drive(
                    -gamepad1.left_stick_x, //pour que ca tourne dans le sens trigo quand tu va vers le + il faut mettre un -
                    1 * (1 / camera.getBigestBlob().getBoxFit().size.area()),
                    camera.getBigestBlob().getBoxFit().center.x * KP_X //soit plus clair sur le nommage de KP_X par son nom on sait pas si c'est le X du robot le x de la cam ou jsp quel autre x qu'il corrige
                    //pas capte tes calculs du fwd j'avoue la j'avoue (surtout le *1 tres utile...)
            );
        }
        else {
            drivetrain.Drive(-gamepad1.left_stick_x, gamepad1.right_stick_y, -gamepad1.right_stick_x); //same for the two x axes
        }
        intake.setIntake_power((double) (gamepad1.right_trigger - gamepad1.left_trigger));
        feeder.setFeederPos(gamepad2.x ? FeederActivePos : FeederIdlePos);



        if (gamepad2.a)
        {
            shooter.SetShooterTargets(Shooter.nearSpeed, Shooter.nearPosHood, voltage);
        }
        else if (gamepad2.b)
        {
            shooter.SetShooterTargets(Shooter.midSpeed, Shooter.midPosHood, voltage);
        }
        else if (gamepad2.y)
        {
            shooter.SetShooterTargets(Shooter.farSpeed, Shooter.farPosHood, voltage);
        }
        else if (gamepad2.left_bumper && camera.getDistanceMeters(24)>0)//it seems that this part doesn't handle (handle = supporter) alliance shifting
        {
            shooter.SetShooterDistanceTarget(camera.getDistanceMeters(24), voltage);//it seems that this part doesn't handle (handle = supporter) alliance shifting
        }
        else
        {
            shooter.stopShooter();
        }

    }

    @Override
    public void stop()
    {
        camera.close();
    }

    private double leftXFilter (double input){//mmmmmmmmhh... Je pense que c'est la fonction la plus clean que tu ait jamais faite vrm bravo !!! ;)
        double output = input;
        return  output;
    }
}
