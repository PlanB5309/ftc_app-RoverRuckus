package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSet {
    Telemetry telemetry;
    RobotHardware robot = new RobotHardware(telemetry);
    public ClawSet(){}
    public void open(){
//        robot.leftClaw.setPosition(0);
        robot.rightClaw.setPosition(1);
    }
    public void close() {
//        robot.leftClaw.setPosition(0.8);
        robot.rightClaw.setPosition(0.25);
    }
}
