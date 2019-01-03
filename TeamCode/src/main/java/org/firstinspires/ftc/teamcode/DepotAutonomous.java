package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sun.tools.javac.comp.Lower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Depot Autonomous", group = "Auto")


public class DepotAutonomous extends LinearOpMode {
    RobotHardware robot = new RobotHardware(telemetry);
    LowerRobot lowerRobot = new LowerRobot(robot, telemetry);
    OpenHooks openHooks = new OpenHooks(robot, telemetry);
    PushGoldBlock pushGoldBlock = new PushGoldBlock(robot, telemetry);
    DropTeamMarker dropTeamMarker = new DropTeamMarker(robot, telemetry);
    FindGold findGold = new FindGold(robot, telemetry);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry);
    Drive drive = new Drive(robot, telemetry);
    KickMarker kickMarker = new KickMarker(robot,telemetry);
    RaiseMineralLift raiseMineralLift = new RaiseMineralLift(robot, telemetry);
//    static DriveToDepot driveToDepot = new DriveToDepot();
//    static DropTeamMarker dropTeamMarker = new DropTeamMarker();
//    static DriveToCrater driveToCrater = new DriveToCrater();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
//        lowerRobot.run();
//        openHooks.open();
        raiseMineralLift.setHalfway();
        gyroTurn.absolute(0);
        int goldPosition = findGold.run();
        telemetry.addData("Gold position: ", goldPosition);
        telemetry.update();
        robot.bucketServo.setPosition(robot.BUCKET_DUMP_POSITION);
        //pushGoldBlock.run(goldPosition);
        if (goldPosition == robot.LEFT) {
            gyroTurn.left(25);
            robot.sweeperMotor.setPower(-0.5);
            drive.forward(0.5, 33);
            gyroTurn.right(55);
            drive.forward(0.5, 6);
            robot.sweeperMotor.setPower(0);
//            kickMarker.run();
            gyroTurn.right(18);
            drive.backward(0.5, 56);
        } else if (goldPosition == robot.RIGHT) {
            gyroTurn.right(29);
            robot.sweeperMotor.setPower(-0.5);
            drive.forward(0.5, 30);
            robot.sweeperMotor.setPower(0);
            gyroTurn.left(65);
            drive.forward(0.5, 12);
//            kickMarker.run();
        } else {
            robot.sweeperMotor.setPower(-0.5);
            drive.forward(0.5, 30);
            robot.sweeperMotor.setPower(0);
            drive.forward(0.5, 12);
//            kickMarker.run();
        }
        raiseMineralLift.setDown();
    }
}
