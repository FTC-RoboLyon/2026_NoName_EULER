package org.firstinspires.ftc.teamcode.Euler_niveau_supérieur_axel;

/*
 * Commentaire général:
 * - Le nom du package n'est pas fou puisqu'il contient un e accentue qui n'est pas un caractere natif de langage de programmation puisqu'il n'est pas dans la table ASCII
 * - Ce serait peut etre mieux de trouver un nom plus clair pour qu'on puisse plus facilement différencié ce code de EulerTeleop
 * - Respectes toujours les mêmes conventions pour le nommage des variables / fonctions / classes. Tu peux trouver dans ce fichier
 *   celle que l'on utilisait avec Adam que j'aimerai bien que tu utilises aussi
 *   (https://github.com/Team5553-RoboLyon/CodingConventionRBL/blob/main/Convention.java)
 * - mettre une voltage compensation a 13V est beaucoup trop haut au contraire normalement elle est censée être un peu plus basse que la normale
 *   pour etre sur que la batterie puisse fournir assez donc plutôt vers 10-11V la (sachant qu'on peu la changer pour chaque mecanisme pour mettre plus a un shooter ou moins a un intake par exemple)
 *   En plus dans tes voltages compensation il manque les securites pour eviter que l'output depasse la valeur de 1.0 ou de -1.0
 * - pour tout tes mecanismes tu pourrais pour l'instant supprimer la fonction loop et la fusionner directement avec sa fonction autre puisqu'ici le loop ne t'ajoute absolument rien
 * - les attributs de tes classes (leurs variables en gros y compris moteurs et tout) devraient être en private et non en public avec un Getter pour y acceder (une fonction qui renvoie sa valeur)
 *   sauf pour tes constantes qui peuvent être en public static
 */



import static org.firstinspires.ftc.teamcode.Euler_niveau_supérieur_axel.Feeder.PosFeederActive;
import static org.firstinspires.ftc.teamcode.Euler_niveau_supérieur_axel.Feeder.PosFeederIddle;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp //c mieux avec ca
public class teleop_bot_azzie extends OpMode {
    DriveTrain base; //Tu peux l'appeler drivetrain plutot (base ne veut rien dire en anglais)
    Intake intake;
    Shooter shooter;
    Feeder feeder;
    Path path; //C quoi la difference entre path et feeder (le nom path n'est pas tres clair)
    VoltageSensor voltageSensor;

    @Override
    public void init() {
         base = new DriveTrain(hardwareMap);
         intake = new Intake(hardwareMap);
         shooter = new Shooter(hardwareMap);
         feeder = new Feeder(hardwareMap);
         path = new Path(hardwareMap);

         voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    @Override
    public void loop() {
        double voltage = voltageSensor.getVoltage(); //tres bon reflexe de stocker cette valeur dans une variable pour eviter de la demander plusieurs fois au capteur. Bien joue

        base.Drive(gamepad1.left_stick_x, gamepad1.right_stick_y);
        intake.setIntake_power((double) (gamepad1.right_trigger - gamepad1.left_trigger));
        feeder.setPosFeeder( gamepad2.x ? PosFeederActive : PosFeederIddle);
        path.setPathPower(Math.abs(intake.intake_power) > 0 ? 1 : -1);        //inverser les 1 et -1 en fonction du sens voulu
        // fait les commentaires en anglais (t'as déjà de la prog en français toi ? et mes corrections ne comptent pas)
        /*
         * Petit beug je pense
         * rappel : pour n'importe quel x la valeur absolue de x tjr > 0 sauf quand x = 0
         * Relis la ligne précédente et je pense que tu verra que ton code ne fait pas trop ce que tu veux faire ;)
         */


        if (gamepad2.a)
            shooter.setShooterState(Shooter.ShooterState.VeloShootFar);
        else if (gamepad2.b) {
            shooter.setShooterState(Shooter.ShooterState.VeloShootNear);
        } else if (gamepad2.y) {
            shooter.setShooterState(Shooter.ShooterState.VeloShootMid);
        }


        base.loop(voltage);
        intake.intake_loop();
        shooter.shooterLoop(voltage);
        feeder.feeder_loop();
        path.Path_loop();
        // tu peux juste laisser loop dans les noms des fonctions (comme t'as fait pour la base) et on préfèrera même periodic apres avec le command based
    }
}
