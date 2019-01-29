package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Depot Autonomous V3 Push Only", group = "Auto")


public class DepotAutonomousV3PushOnly extends LinearOpMode {
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
        telemetry.addData("Encoder Value: ", robot.mineralMotor.getCurrentPosition());
        telemetry.update();
        robot.extenderMotor.setPower(0.75);
        lowerRobot.run();
        openHooks.open();
        gyroTurn.absolute(0);
        int goldPosition = findGold.run();
        telemetry.addData("Gold position: ", goldPosition);
        telemetry.update();
        drive.forward(0.5,10);
        gyroTurn.twoWheel(5, robot.RIGHT);
        robot.markerServo.setPosition(1);
        Thread.sleep(500);
        robot.markerServo.setPosition(0.05);
        robot.extenderMotor.setPower(0);
        gyroTurn.twoWheel(7, robot.LEFT);
        Thread.sleep(500);
        robot.extenderMotor.setPower(-0.75);
        drive.backward(0.5, 4);
        if (goldPosition == robot.LEFT) {
            gyroTurn.twoWheel(20, robot.LEFT);
            robot.sweeperMotor.setPower(-0.5);
            robot.extenderMotor.setPower(0);
            drive.forward(0.5, 24);
            robot.sweeperMotor.setPower(0);
        } else if (goldPosition == robot.RIGHT) {
            gyroTurn.twoWheel(29, robot.RIGHT);
            robot.sweeperMotor.setPower(-0.5);
            robot.extenderMotor.setPower(0);
            drive.forward(0.5, 24);
            robot.sweeperMotor.setPower(0);
        } else {
            robot.sweeperMotor.setPower(-0.5);
            drive.forward(0.5, 24);
            robot.sweeperMotor.setPower(0);
            Thread.sleep(1000);
            robot.extenderMotor.setPower(0);
        }
    }
}
