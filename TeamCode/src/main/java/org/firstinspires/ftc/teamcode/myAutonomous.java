package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sun.tools.javac.comp.Lower;

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
    static LowerRobot lowerRobot = new LowerRobot();
    static PushGoldBlock pushGoldBlock = new PushGoldBlock();
    static DriveToDepot driveToDepot = new DriveToDepot();
    static DropTeamMarker dropTeamMarker = new DropTeamMarker();
    static DriveToCrater driveToCrater = new DriveToCrater();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        LowerRobot.run();
    }

}
