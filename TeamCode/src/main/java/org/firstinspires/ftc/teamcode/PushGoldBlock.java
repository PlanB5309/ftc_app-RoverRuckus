package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PushGoldBlock {
    Telemetry telemetry;
    RobotHardware robot = new RobotHardware(telemetry);

    public PushGoldBlock(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void run () throws InterruptedException{
        //2240 counts per rotation
        robot.sweeperMotor.setPower(-1);

        robot.rightDrive.setTargetPosition(3629);
        robot.leftDrive.setTargetPosition(3629);
        robot.rightDrive.setPower(0.5);
        robot.leftDrive.setPower(0.5);

        Thread.sleep(2500);
//        while (robot.rightDrive.isBusy()) {
//            Thread.yield();
            telemetry.addData("right wheel: ", robot.rightDrive.getCurrentPosition());
            telemetry.update();
//        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.sweeperMotor.setPower(0);
        Thread.sleep(1500);
    }

}
