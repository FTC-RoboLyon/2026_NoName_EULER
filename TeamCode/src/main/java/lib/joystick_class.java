package lib;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Objects;

public class joystick_class {
    public double X, Y;
    Gamepad gamepad;
    enum  Witch_stick{
        left, right
    }
    private Witch_stick stick;
    public joystick_class (Gamepad gamepad, String name){
        this.gamepad = gamepad;
        if (Objects.equals(name, "left")){
            stick = Witch_stick.left;
        } else if (Objects.equals(name, "right")) {
            stick = Witch_stick.right;
        }
    }
    public void actualise(){
        if (stick == Witch_stick.left){
            X = gamepad.left_stick_x;
            Y = gamepad.left_stick_y;
        } else if (stick == Witch_stick.right){
            X = gamepad.right_stick_x;
            Y = gamepad.right_stick_y;
        }
    }

}
