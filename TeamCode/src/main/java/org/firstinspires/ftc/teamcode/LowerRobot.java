package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LowerRobot {
    RobotHardware robot;
    Telemetry telemetry;

    public LowerRobot(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }
    //static final double COUNTS_PER_MOTOR_REV = 1080;
    //6 rotations of the motor needed to lift/lower claws, so 6480 clicks/counts
//    static Lift lift = new Lift();
    public void run(){

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int target = 6480;
        robot.liftMotor.setTargetPosition(target);
        robot.liftMotor.setPower(0.5);

        while (robot.liftMotor.isBusy()) {
            Thread.yield();
            telemetry.addData("lift clicks", robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.liftMotor.setPower(0);
    }
}
