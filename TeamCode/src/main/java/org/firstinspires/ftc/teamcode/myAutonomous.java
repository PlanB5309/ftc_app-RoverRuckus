package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Left Blue Autonomous", group="Blue")

/* Lifter Servo Values
* servo 1 - left claw: open = 0, closed = 0.8
*servo 0 - right claw: 0pen = 1, closed = 0.25
* 6 rotations of lifter motor to lift up chain
*/

public class myAutonomous extends LinearOpMode {
    RobotHardware         robot   = new RobotHardware(telemetry);
    LowerRobot lowerRobot = new LowerRobot();
    ReleaseHook releaseHook = new ReleaseHook();
    PushGoldBlock pushGoldBlock = new PushGoldBlock();
    DriveToDepot driveToDepot = new DriveToDepot();
    DropTeamMarker dropTeamMarker = new DropTeamMarker();
    DriveToCrater driveToCrater = new DriveToCrater();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
    }

}
