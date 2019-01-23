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
        lowerRobot.run();
        openHooks.open();
        gyroTurn.absolute(0);
        int goldPosition = findGold.run();
        telemetry.addData("Gold position: ", goldPosition);
        telemetry.update();
        robot.bucketServo.setPosition(robot.BUCKET_SCOOP_POSITION);
        drive.forward(0.75,6);
        robot.extenderMotor.setPower(0.75);
        Thread.sleep(6000);
        robot.markerServo.setPosition(1);
        Thread.sleep(500);
        robot.markerServo.setPosition(0);
        robot.extenderMotor.setPower(-0.75);
        Thread.sleep(6000);
        robot.extenderMotor.setPower(0);
        if (goldPosition == robot.LEFT) {
            gyroTurn.left(20);
            robot.sweeperMotor.setPower(-0.5);
            drive.forward(0.75, 24);
            robot.sweeperMotor.setPower(0);
            drive.backward(0.75, 20);
            gyroTurn.right(20);
        } else if (goldPosition == robot.RIGHT) {
            gyroTurn.right(29);
            robot.sweeperMotor.setPower(-0.5);
            drive.forward(0.75, 24);
            robot.sweeperMotor.setPower(0);
            drive.backward(0.75, 20);
            gyroTurn.left(29);
        } else {
            robot.sweeperMotor.setPower(-0.5);
            drive.forward(0.75, 24);
            robot.sweeperMotor.setPower(0);
            drive.backward(0.75, 20);
        }
        Thread.sleep(100);
        gyroTurn.right(90);
        drive.forward(0.5, 12);
        gyroTurn.right(45);
        while(isStopRequested() == false){
            robot.extenderMotor.setPower(0.75);
        }
    }
}
