package org.firstinspires.ftc.teamcode;

public class LowerRobot {
    //static final double COUNTS_PER_MOTOR_REV = 1080;
    //6 rotations of the motor needed to lift/lower claws, so 6480 clicks/counts
    static Lift lift = new Lift();
    static ClawSet claws = new ClawSet();
    public static void run(){
        lift.target(-6480);//unsure of +/-
        claws.open();
        lift.target(6480); // opposite sign of above
        claws.close();
    }
}
