package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ThreadLocalRandom;

@Autonomous(name = "Depot Autonomous V3 5 Second Wait", group = "Auto")


public class DepotAutonomousV3Wait5Sec extends LinearOpMode {
    RobotHardware robot = new RobotHardware(telemetry);
    LowerRobot lowerRobot = new LowerRobot(robot, telemetry);
    OpenHooks openHooks = new OpenHooks(robot, telemetry);
    FindGold findGold = new FindGold(robot, telemetry);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry);
    Drive drive = new Drive(robot, telemetry);
    KickMarker kickMarker = new KickMarker(robot,telemetry);
    MineralLift mineralLift = new MineralLift(robot, telemetry);
//    static DriveToDepot driveToDepot = new DriveToDepot();
//    static DropTeamMarker dropTeamMarker = new DropTeamMarker();
//    static DriveToCrater driveToCrater = new DriveToCrater();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        Thread.sleep(5000);
        telemetry.addData("Encoder Value: ", robot.mineralMotor.getCurrentPosition());
        telemetry.update();
        robot.extenderMotor.setPower(0.75);
        lowerRobot.run();
        openHooks.open();
        gyroTurn.absolute(0);
        int goldPosition = findGold.run();
        telemetry.addData("Gold position: ", goldPosition);
        telemetry.update();
        drive.forward(0.4,10);
        gyroTurn.twoWheel(5, robot.RIGHT);
        robot.markerServo.setPosition(1);
        Thread.sleep(500);
        robot.markerServo.setPosition(0.05);
        robot.extenderMotor.setPower(0);
        gyroTurn.twoWheel(5, robot.LEFT);
        Thread.sleep(500);
        robot.extenderMotor.setPower(-0.75);
        if (goldPosition == robot.LEFT) {
            drive.backward(0.4, 4);
            gyroTurn.twoWheel(20, robot.LEFT);
            robot.sweeperMotor.setPower(0.5);
            robot.extenderMotor.setPower(0);
            drive.forward(0.4, 24);
            robot.sweeperMotor.setPower(0);
            drive.backward(0.4, 15);
            gyroTurn.twoWheel(80, robot.RIGHT);
            gyroTurn.twoWheel(20, robot.RIGHT);
            drive.forward(0.4, 30);
            gyroTurn.twoWheel(40, robot.RIGHT);
            robot.extenderMotor.setPower(0.75);
            robot.markerServo.setPosition(1);
        } else if (goldPosition == robot.RIGHT) {
            drive.backward(0.4, 4);
            gyroTurn.twoWheel(29, robot.RIGHT);
            robot.sweeperMotor.setPower(0.5);
            robot.extenderMotor.setPower(0);
            drive.forward(0.4, 24);
            robot.sweeperMotor.setPower(0);
            gyroTurn.twoWheel(95, robot.RIGHT);
            drive.forward(0.4, 8);
            robot.extenderMotor.setPower(0.75);
            robot.markerServo.setPosition(1);
        } else {
            robot.sweeperMotor.setPower(0.5);
            drive.forward(0.4, 20);
            robot.sweeperMotor.setPower(0);
            Thread.sleep(1000);
            robot.extenderMotor.setPower(0);
            drive.backward(0.4, 9);
            gyroTurn.twoWheel(90, robot.RIGHT);
            drive.forward(0.4, 18);
            gyroTurn.twoWheel(24, robot.RIGHT);
            robot.extenderMotor.setPower(0.75);
            robot.markerServo.setPosition(1);
        }
        while(!isStopRequested()){

        }
    }
}
