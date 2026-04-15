package org.firstinspires.ftc.teamcode.euler;

public interface Step {

    /**
     * Methode qui sera appelé une fois lorsque la step commencera
     *
     * @param robot
     */
    void init(Robot robot);

    /**
     * Methode qui sera appelé tant que le isFinished est faux
     *
     * @param robot
     */
    void run(Robot robot);

    /**
     * Methode qui sera appelé une fois lorsque la step est terminée
     *
     * @param robot
     */
    void finish(Robot robot);

    boolean isInitialized();

    boolean isFinished();
}
