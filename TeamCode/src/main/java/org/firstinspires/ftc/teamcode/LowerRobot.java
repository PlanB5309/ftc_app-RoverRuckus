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
    //    static Lift lift = new Lift();
    public void run() throws InterruptedException{

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int target = 8500;
        robot.liftMotor.setTargetPosition(target);
        robot.liftMotor.setPower(0.5);

        while (robot.liftMotor.isBusy()) {
            telemetry.addData("lift clicks", robot.liftMotor.getCurrentPosition());
            telemetry.addData("mineral lift clicks", robot.mineralMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.liftMotor.setPower(0);
        Thread.sleep(0250);

//        robot.leftDrive.setPower(-0.25);
//        robot.rightDrive.setPower(-0.25);
        Thread.sleep(0200);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
}
