package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="ForwardTest", group="Auto")



public class ForwardTest extends LinearOpMode {
    RobotHardware         robot   = new RobotHardware(telemetry);
    LowerRobot lowerRobot = new LowerRobot(robot, telemetry);
    OpenHooks openHooks = new OpenHooks(robot, telemetry);
    PushGoldBlock pushGoldBlock = new PushGoldBlock(robot, telemetry);
    DropTeamMarker dropTeamMarker = new DropTeamMarker(robot, telemetry);
    Drive drive = new Drive(robot,telemetry);
    GyroTurn gyroTurn = new GyroTurn(robot,telemetry);


    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        gyroTurn.right(90);
        drive.forward(.5,12);
        gyroTurn.left(90);
    }
}
