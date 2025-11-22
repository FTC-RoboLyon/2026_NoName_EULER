package org.firstinspires.ftc.teamcode;


public class Test_direction_Antoine extends code_main {


    private void Drive() {
        x = gamepad1.right_stick_x;
        y = -gamepad1.left_stick_y;
        rightmotor.setPower(y+x);
        leftmotor.setPower(y-x);
    }

    private void runOpMode() {
        waitForStart();
        if (opModeIsActive) {
            while(opModeIsActive) {
                Drive();
            }
        }
    }

    private void main() {

        //Méthodes de code_main
        importation();
        création();
        inizialisation();
        imu_inizialisation();
        encodeur_inizialisation();
        nommage();

        //Fonctions de notre code
        runOpMode();

    }
}
