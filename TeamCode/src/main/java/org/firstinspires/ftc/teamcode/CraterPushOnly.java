package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Crater Push Only", group="Auto")

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
        lowerRobot.run();
        openHooks.open();
        gyroTurn.absolute(0);
        robot.bucketServo.setPosition(robot.BUCKET_SCOOP_POSITION);

        goldPosition = findGold.run();
        telemetry.addData("Gold Position: ", goldPosition);
        telemetry.update();

        pushGoldBlock.run(goldPosition);
        if (goldPosition == robot.CENTER) { //Test these three if clauses
            drive.forward(0.5, 7);
        }
        else if (goldPosition == robot.LEFT) {
            drive.forward(0.5, 2);
        }
        else if (goldPosition == robot.RIGHT) {
            drive.forward(0.5, 2);
        }
//
        while(isStopRequested() == false){
        }
    }
}