package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Depot Autonomous V2", group = "Auto")


public class DepotAutonomousV2 extends LinearOpMode {
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
        mineralLift.setHalfway();
        telemetry.addData("Encoder Value: ", robot.mineralMotor.getCurrentPosition());
        telemetry.update();
        lowerRobot.run();
        openHooks.open();
        gyroTurn.absolute(0);
        int goldPosition = findGold.run();
        telemetry.addData("Gold position: ", goldPosition);
        telemetry.update();
        robot.bucketServo.setPosition(robot.BUCKET_DUMP_POSITION);
        //pushGoldBlock.run(goldPosition);
        if (goldPosition == robot.LEFT) {
            gyroTurn.left(20);
            robot.sweeperMotor.setPower(-0.5);
            drive.forward(0.5, 24);
            robot.sweeperMotor.setPower(0);
            gyroTurn.right(15);
            drive.forward(0.5, 20);
            robot.bucketServo.setPosition(0.58);
            Thread.sleep(1000);
            robot.sweeperMotor.setPower(-0.5);
            Thread.sleep(2000);
            robot.sweeperMotor.setPower(0);
            drive.backward(0.5, 36);
            gyroTurn.left(90);
            drive.backward(0.5, 60);
            gyroTurn.right(45);
            drive.backward(0.5, 28);
//            gyroTurn.right(18);
//            drive.backward(0.5, 56);
        } else if (goldPosition == robot.RIGHT) {
            gyroTurn.right(29);
            robot.sweeperMotor.setPower(-0.5);
            drive.forward(0.5, 30);
            robot.sweeperMotor.setPower(0);
            drive.backward(0.5, 20);
            gyroTurn.left(155);
            drive.backward(0.5, 24);
        } else {
            robot.sweeperMotor.setPower(-0.5);
            drive.forward(0.5, 40);
            robot.sweeperMotor.setPower(0);
            robot.bucketServo.setPosition(0.58);
            Thread.sleep(1000);
            robot.sweeperMotor.setPower(-0.5);
            Thread.sleep(2000);
            robot.sweeperMotor.setPower(0);
            drive.backward(0.5, 22);
            gyroTurn.right(90);
            drive.forward(0.5, 42);
            gyroTurn.right(45);
            drive.forward(0.5, 28);
        }
        mineralLift.setDown();
        while(isStopRequested() == false){

        }
    }
}
