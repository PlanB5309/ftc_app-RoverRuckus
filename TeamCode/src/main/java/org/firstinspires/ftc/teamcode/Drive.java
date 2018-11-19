package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drive {

    RobotHardware robot;
    Telemetry telemetry;

    public Drive(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void forward(double speed, int distance) throws InterruptedException {
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (distance * robot.COUNTS_PER_INCH);
        robot.rightDrive.setTargetPosition(target);
        robot.leftDrive.setTargetPosition(target);
        robot.rightDrive.setPower(speed);
        robot.leftDrive.setPower(speed);


        while (robot.rightDrive.isBusy()) {
            Thread.yield();
            robot.rightDrive.setPower(speed);
            robot.leftDrive.setPower(speed);
        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
    public void backward(double speed, int distance) throws InterruptedException {
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (distance * robot.COUNTS_PER_INCH);
        robot.rightDrive.setTargetPosition(-target);
        robot.leftDrive.setTargetPosition(-target);
        robot.rightDrive.setPower(-speed);
        robot.leftDrive.setPower(-speed);


        while (robot.rightDrive.isBusy()) {
            Thread.yield();
            robot.rightDrive.setPower(-speed);
            robot.leftDrive.setPower(-speed);
        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
}

