package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sweeper {
    Telemetry telemetry;
    RobotHardware robot = new RobotHardware(telemetry);
    public Sweeper(){}
    public void on(){
        robot.sweeperMotor.setPower(robot.SWEEPER_POWER);
    }
    public void off(){
        robot.sweeperMotor.setPower(0);
    }
    public void reverse(){
        robot.sweeperMotor.setPower(-robot.SWEEPER_POWER);
    }
}
