package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Left Blue Autonomous", group="Blue")
public class Autonomous extends LinearOpMode {
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