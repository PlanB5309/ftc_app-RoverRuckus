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
//        robot.mineralMotor.setPower(0.5);
//        Thread.sleep(500);
//        robot.mineralMotor.setPower(0);
//        robot.mineralMotor.setTargetPosition(500);
//        Thread.sleep(3000);
        robot.bucketServo.setPosition(0.58);
        Thread.sleep(500);
        robot.sweeperMotor.setPower(-0.5);
        drive.backward(0.5, 6);
        Thread.sleep(5000);
        robot.sweeperMotor.setPower(0);
    }

}

