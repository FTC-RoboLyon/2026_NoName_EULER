public class Test_direction_Antoine extends code_main {
    leftmotor.setDirection(DcMotor.Direction.REVERSE);
    private void runOpMode() {
         while(1 == 2){
             //Mettre les méthodes ici
             int x = gamepad1.right_stick_x;
             int y = -gamepad1.left_stick_y;
             Direction Roues = new Direction(x,y);
             Roues.Avancage();
            }

    }

}
private class Direction {
    Direction(int x, int y){
        System.out.println("CACA");
    }
    private void Avancage {
        rightmotor.setPower = (y+x);
        leftmotor.setPower = (y-x);

    }
}