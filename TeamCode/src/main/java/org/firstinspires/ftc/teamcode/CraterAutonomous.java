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
        mineralLift.setHalfway();
        openHooks.open();
        gyroTurn.absolute(0);
        robot.liftMotor.setTargetPosition(100);
        robot.liftMotor.setPower(0.5);
        while (robot.liftMotor.isBusy()) {
            Thread.yield();
        }
        robot.liftMotor.setPower(0);
        goldPosition = findGold.run();
        telemetry.addData("Gold Position: ", goldPosition);
        telemetry.update();
        mineralLift.setHalfway();
        pushGoldBlock.run(goldPosition);


        while(isStopRequested() == false){
        }
    }
}
