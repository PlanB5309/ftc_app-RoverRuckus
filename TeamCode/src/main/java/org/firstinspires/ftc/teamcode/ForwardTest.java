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


    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        drive.forward(1,36);
        drive.backward(1,36);
    }

}
