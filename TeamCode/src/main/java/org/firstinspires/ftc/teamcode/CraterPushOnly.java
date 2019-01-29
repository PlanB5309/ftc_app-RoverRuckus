package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

<<<<<<< HEAD
@Autonomous(name="Crater Push Only", group="Auto")
=======
@Autonomous(name="Crater Side Only", group="Auto")
>>>>>>> 9b357a06173ab2e69da4b8c1d3a85811477da986

public class CraterPushOnly extends LinearOpMode {
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
        //Uncomment to gyroturn.abs
        lowerRobot.run();
//        mineralLift.setHalfway();
        openHooks.open();
        gyroTurn.absolute(0);
        robot.bucketServo.setPosition(robot.BUCKET_SCOOP_POSITION);
//        robot.liftMotor.setTargetPosition(100);
//        robot.liftMotor.setPower(0.5);
//        while (robot.liftMotor.isBusy()) {
//            Thread.yield();
//        }
//        robot.liftMotor.setPower(0);
        goldPosition = findGold.run();
        telemetry.addData("Gold Position: ", goldPosition);
        telemetry.update();
//        mineralLift.setHalfway();
        pushGoldBlock.run(goldPosition);
        if (goldPosition == robot.CENTER) { //Test these three if clauses
            drive.forward(0.5, 7);
        }
        else if (goldPosition == robot.LEFT) {
            drive.backward(0.6, 10);
        }
        else if (goldPosition == robot.RIGHT) {
            drive.backward(0.6, 10);
        }
//
        while(isStopRequested() == false){
        }
    }
}