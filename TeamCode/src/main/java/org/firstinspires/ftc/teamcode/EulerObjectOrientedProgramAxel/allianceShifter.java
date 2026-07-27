package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//L'idee est pas bonne mais c'est tres complexe pour pas grand chose en plus faut connaitre le code pour savoir comment il marche
// donc au moins un peu de telemetry ne serait pas de refus. Apres j'ai pas compris l'interet d'utiliser le "blackboard" plutot
// qu'une simple variable static dans un type plus basique (string ou boolean). Pour l'instant laisse le il est quand même utile mais
// c'est aussi un autre avantage des subsystem et du command base qui gere ca tres bien quand on passera la dessus. Aussi vu que c'est dcp
// le meme opmode quelque soit la couleur peut etre afficher la couleur selectionnee dans l'init de la teleop.

@TeleOp(name = "allianceShifter")
public class allianceShifter extends OpMode {
    public static final String ALLIANCE_KEY = "Alliance";
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if (gamepad1.leftBumperWasPressed()) {
            blackboard.put(ALLIANCE_KEY, "RED");
        } else if (gamepad1.rightBumperWasPressed()) {
            blackboard.put(ALLIANCE_KEY, "BLUE");
        }
        telemetry.addData("Alliance", blackboard.get(ALLIANCE_KEY));
    }
}
