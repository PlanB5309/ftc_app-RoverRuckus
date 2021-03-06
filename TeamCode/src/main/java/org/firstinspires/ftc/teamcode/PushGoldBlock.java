package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PushGoldBlock {
    Telemetry telemetry;
    RobotHardware robot = new RobotHardware(telemetry);

    public PushGoldBlock(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void run(int loc) throws InterruptedException{
        GyroTurn gyroTurn = new GyroTurn(robot, telemetry);
        Drive drive = new Drive(robot, telemetry);
        if(loc == robot.LEFT){
            drive.forward(0.5, 1);
            gyroTurn.left(25);
//            gyroTurn.twoWheel(25, robot.LEFT);
            robot.sweeperMotor.setPower(1);
            drive.forward(0.3, 30);
        }else if(loc == robot.RIGHT){
            drive.forward(0.5, 1);
            gyroTurn.right(20);
//            gyroTurn.twoWheel(20, robot.RIGHT);
            robot.sweeperMotor.setPower(1);//prev. -0.5
            drive.forward(0.3, 20);
        }else if (loc == robot.CENTER) {
            robot.sweeperMotor.setPower(1);//prev. -0.5
            drive.forward(0.3, 30);
        }
    }
//    //2240 counts per rotation
//        robot.sweeperMotor.setPower(-1);
//
//        robot.rightDrive.setTargetPosition(3629);
//        robot.leftDrive.setTargetPosition(3629);
//        robot.rightDrive.setPower(0.5);
//        robot.leftDrive.setPower(0.5);
//
//        Thread.sleep(2000);
//        while (robot.rightDrive.isBusy()) {
////            Thread.yield();
//        telemetry.addData("right wheel: ", robot.rightDrive.getCurrentPosition());
//        telemetry.update();
//    }
//
//        robot.leftDrive.setPower(0);
//        robot.rightDrive.setPower(0);
//        robot.sweeperMotor.setPower(0);
//        Thread.sleep(1500);
}
