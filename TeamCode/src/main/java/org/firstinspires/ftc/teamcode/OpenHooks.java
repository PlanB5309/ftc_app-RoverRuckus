package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OpenHooks {
    Telemetry telemetry;
    RobotHardware robot = new RobotHardware(telemetry);

    public OpenHooks(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void open () throws InterruptedException{
        robot.leftClaw.setPosition(robot.LEFT_CLAW_OPEN);
        robot.rightClaw.setPosition(robot.RIGHT_CLAW_OPEN);
        Thread.sleep(250);
    }
}
