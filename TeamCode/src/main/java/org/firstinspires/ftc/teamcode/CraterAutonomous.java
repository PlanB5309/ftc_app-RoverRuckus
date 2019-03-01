package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Crater Autonomous", group="Auto")

public class CraterAutonomous extends LinearOpMode {
    RobotHardware robot = new RobotHardware(telemetry);
    LowerRobot lowerRobot = new LowerRobot(robot, telemetry);
    OpenHooks openHooks = new OpenHooks(robot, telemetry);
    PushGoldBlock pushGoldBlock = new PushGoldBlock(robot, telemetry);
    FindGold findGold = new FindGold(robot,telemetry);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry);
    Drive drive = new Drive(robot, telemetry);
    MineralLift mineralLift = new MineralLift(robot, telemetry);

    public void runOpMode() throws InterruptedException {
        int goldPosition;

        robot.init(hardwareMap);
        waitForStart();
        lowerRobot.run();
        openHooks.open();
        robot.bucketServo.setPosition(robot.BUCKET_SCOOP_POSITION);

        goldPosition = findGold.run();
        telemetry.addData("Gold Position: ", goldPosition);
        telemetry.update();
        pushGoldBlock.run(goldPosition);

        if (goldPosition == robot.CENTER) {
            drive.backward(0.4, 10);
            robot.sweeperMotor.setPower(0);

            gyroTurn.twoWheel(75, robot.LEFT);//previously 85degrees
            drive.forward(0.4, 30);
            gyroTurn.twoWheel(32, robot.LEFT);

            robot.extenderMotor.setPower(0.75);
            Thread.sleep(6000);
            robot.markerServo.setPosition(robot.MARKER_OUT);
            robot.extenderMotor.setPower(0);
            Thread.sleep(500);

            robot.markerServo.setPosition(robot.MARKER_IN);
            robot.extenderMotor.setPower(-1);
            gyroTurn.twoWheel(45, robot.LEFT);
            drive.backward(0.5, 28);
            robot.extenderMotor.setPower(0);
        }
        else if (goldPosition == robot.LEFT) {
            drive.backward(0.5, 4);
            robot.sweeperMotor.setPower(0);

            gyroTurn.twoWheel(40, robot.LEFT);
            drive.forward(0.5, 14);
            gyroTurn.twoWheel(35, robot.LEFT);
            drive.forward(0.5, 10);

            robot.extenderMotor.setPower(0.75);
            Thread.sleep(6000);
            robot.markerServo.setPosition(robot.MARKER_OUT);
            robot.extenderMotor.setPower(0);
            Thread.sleep(500);

            robot.markerServo.setPosition(robot.MARKER_IN);
            robot.extenderMotor.setPower(-1);
            gyroTurn.twoWheel(10, robot.LEFT);
            drive.backward(0.5, 22);
            robot.extenderMotor.setPower(0);
        }
        else if (goldPosition == robot.RIGHT) {
            drive.backward(0.4, 6);
            robot.sweeperMotor.setPower(0);

            gyroTurn.twoWheel(90, robot.LEFT);
            drive.forward(0.4, 35);
            gyroTurn.twoWheel(40, robot.LEFT);
            robot.extenderMotor.setPower(0.75);
            drive.forward(0.4, 7);

            Thread.sleep(4500);
            robot.markerServo.setPosition(robot.MARKER_OUT);
            robot.extenderMotor.setPower(0);
            Thread.sleep(500);

            robot.markerServo.setPosition(robot.MARKER_IN);
            robot.extenderMotor.setPower(-1);
            gyroTurn.twoWheel(45, robot.LEFT);
            drive.backward(0.5, 28);
            robot.extenderMotor.setPower(0);
        }
//
        while(isStopRequested() == false){
        }
    }
}
