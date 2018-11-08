package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DropTeamMarker {
    Telemetry telemetry;
    RobotHardware robot = new RobotHardware(telemetry);

    public DropTeamMarker(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void drop () throws InterruptedException {
        robot.markerServo.setPosition(robot.MARKER_CLAW_OPEN);
        Thread.sleep(1000);
    }
}
