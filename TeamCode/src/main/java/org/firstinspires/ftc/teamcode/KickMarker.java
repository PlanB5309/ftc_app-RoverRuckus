package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class KickMarker {

    RobotHardware robot;
    Telemetry telemetry;


    public KickMarker(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void run() throws  InterruptedException{
        Drive drive = new Drive(robot, telemetry);
        robot.mineralMotor.setPower(0.5);
        Thread.sleep(500);
        robot.mineralMotor.setPower(0);
        robot.bucketServo.setPosition(0.5);
        robot.sweeperMotor.setPower(-0.5);
        drive.backward(0.5, 3);
        robot.sweeperMotor.setPower(0);
    }

}

