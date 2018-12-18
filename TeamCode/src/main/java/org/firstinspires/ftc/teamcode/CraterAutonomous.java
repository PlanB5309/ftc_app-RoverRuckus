package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sun.tools.javac.comp.Lower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
@Autonomous(name="Crater Autonomous", group="Auto")

public class CraterAutonomous extends LinearOpMode {
    RobotHardware robot = new RobotHardware(telemetry);
    LowerRobot lowerRobot = new LowerRobot(robot, telemetry);
    OpenHooks openHooks = new OpenHooks(robot, telemetry);
    PushGoldBlock pushGoldBlock = new PushGoldBlock(robot, telemetry);
    DropTeamMarker dropTeamMarker = new DropTeamMarker(robot, telemetry);
    FindGold findGold = new FindGold(robot,telemetry);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry);
    Drive drive = new Drive(robot, telemetry);


    public void runOpMode() throws InterruptedException {
        int goldPosition;

        robot.init(hardwareMap);
        waitForStart();
//        lowerRobot.run();
//        drive.backward(0.25, 1);
//        openHooks.open();
//        gyroTurn.absolute(0);
        goldPosition = findGold.run();
        telemetry.addData("Gold Position: ", robot.CENTER);
        telemetry.update();

        robot.mineralMotor.setPower(-0.75);
        sleep(750);
        robot.mineralMotor.setPower(0);

        robot.sweeperMotor.setPower(-1);
        pushGoldBlock.run(goldPosition);
        robot.sweeperMotor.setPower(0);

        if (goldPosition == robot.LEFT) {
            gyroTurn.right(45);
        }
        else if (goldPosition == robot.RIGHT) {
            gyroTurn.left(45);
        }
        else { //Position = center
            drive.forward(0.25, 2);
        }
        drive.forward(0.25, 6);
//        dropTeamMarker.drop();
    }
}
