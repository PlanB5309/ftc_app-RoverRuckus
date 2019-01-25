package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Depot Autonomous V3", group = "Auto")


public class DepotAutonomousV3 extends LinearOpMode {
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
        robot.extenderMotor.setPower(0);
        gyroTurn.absolute(0);
        int goldPosition = findGold.run();
        telemetry.addData("Gold position: ", goldPosition);
        telemetry.update();
        robot.bucketServo.setPosition(robot.BUCKET_SCOOP_POSITION);
        drive.forward(0.5,6);
        gyroTurn.twoWheel(5, robot.RIGHT);
        robot.markerServo.setPosition(1);
        Thread.sleep(500);
        robot.markerServo.setPosition(0.05);
        gyroTurn.twoWheel(5, robot.LEFT);
        robot.extenderMotor.setPower(-0.75);
        if (goldPosition == robot.LEFT) {
            gyroTurn.left(20);
            robot.sweeperMotor.setPower(-0.5);
            robot.extenderMotor.setPower(0);
            drive.forward(0.5, 24);
            robot.sweeperMotor.setPower(0);
            drive.backward(0.5, 14);
            gyroTurn.twoWheel(105, robot.RIGHT);
            drive.forward(0.5, 6);
        } else if (goldPosition == robot.RIGHT) {
            gyroTurn.right(29);
            robot.sweeperMotor.setPower(-0.5);
            robot.extenderMotor.setPower(0);
            drive.forward(0.5, 24);
            robot.sweeperMotor.setPower(0);
            drive.backward(0.5, 22);
            gyroTurn.right(56);
        } else {
            robot.sweeperMotor.setPower(-0.5);
            drive.forward(0.5, 24);
            robot.sweeperMotor.setPower(0);
            drive.backward(0.5, 22);
            robot.extenderMotor.setPower(0);
            gyroTurn.right(85);
        }
        Thread.sleep(100);
        drive.forward(0.5, 5);
        Thread.sleep(100);
        gyroTurn.right(17);
        robot.markerServo.setPosition(1);
        while(isStopRequested() == false){
            robot.extenderMotor.setPower(0.75);
        }
    }
}
