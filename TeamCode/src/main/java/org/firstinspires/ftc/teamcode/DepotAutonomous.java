package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Depot Autonomous", group = "Auto")


public class DepotAutonomous extends LinearOpMode {
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
            drive.forward(0.5, 33);
            gyroTurn.right(55);
            drive.forward(0.5, 12);
            robot.sweeperMotor.setPower(0);
            robot.bucketServo.setPosition(0.58);
//            gyroTurn.right(18);
//            drive.backward(0.5, 56);
        } else if (goldPosition == robot.RIGHT) {
            gyroTurn.right(29);
            robot.sweeperMotor.setPower(-0.5);
            drive.forward(0.5, 30);
            robot.sweeperMotor.setPower(0);
            gyroTurn.left(65);
            drive.forward(0.5, 18);
            robot.bucketServo.setPosition(0.58);
        } else {
            robot.sweeperMotor.setPower(-0.5);
            drive.forward(0.5, 30);
            robot.sweeperMotor.setPower(0);
            drive.forward(0.5, 22);
            robot.bucketServo.setPosition(0.58);
        }
        while(isStopRequested() == false){

        }
    }
}
