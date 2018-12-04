package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class GyroTurn {
    RobotHardware robot;
    Telemetry telemetry;
    double currHeading;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    public GyroTurn(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;

    }

    public void right(double degrees) throws InterruptedException {
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        updateHeading();
        double target = currHeading - degrees;
        double diff;
        diff = target - currHeading;
        while (Math.abs(diff) > 1) {
            diff = target - currHeading;
            telemetry.addData("diff: ", diff);
            if (diff < 0) {
                if (Math.abs(diff) > 30)
                    robot.leftDrive.setPower(robot.HIGH_TURN_POWER);
                else {
                    robot.leftDrive.setPower(robot.LOW_TURN_POWER);
                }
            }
            if (diff > 0) {
                if (Math.abs(diff) > 30)
                    robot.leftDrive.setPower(-robot.HIGH_TURN_POWER);
                else {
                    robot.leftDrive.setPower(-robot.LOW_TURN_POWER);
                }
            }
            updateHeading();
        }
        robot.leftDrive.setPower(0);
    }

    public void left(double degrees) throws InterruptedException {
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        updateHeading();
        double target = currHeading + degrees;
        double diff;
        diff = target - currHeading;
        while (Math.abs(diff) > 1) {
            diff = target - currHeading;
            telemetry.addData("diff:", diff);
            if (diff < 0) {
                if (Math.abs(diff) > 30)
                    robot.rightDrive.setPower(-robot.HIGH_TURN_POWER);
                else {
                    robot.rightDrive.setPower(-robot.LOW_TURN_POWER);
                }
            }
            if (diff > 0) {
                if (Math.abs(diff) > 30)
                    robot.rightDrive.setPower(robot.HIGH_TURN_POWER);
                else {
                    robot.rightDrive.setPower(robot.LOW_TURN_POWER);
                }
            }
            updateHeading();
        }
        robot.rightDrive.setPower(0);
    }

    public void absolute(double target) {
//        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        updateHeading();
        double diff;
        diff = target - currHeading;
        while (Math.abs(diff) > 1) {
            diff = target - currHeading;
            telemetry.addData("diff:", diff);
            if (diff < 0) {
                if (Math.abs(diff) > 30) {
                    robot.rightDrive.setPower(-robot.HIGH_TURN_POWER);
                    robot.leftDrive.setPower(robot.HIGH_TURN_POWER);
                } else {
                    robot.leftDrive.setPower(robot.LOW_TURN_POWER);
                    robot.rightDrive.setPower(-robot.LOW_TURN_POWER);
                }
            }
            if (diff > 0) {
                if (Math.abs(diff) > 30) {
                    robot.rightDrive.setPower(robot.HIGH_TURN_POWER);
                    robot.leftDrive.setPower(-robot.HIGH_TURN_POWER);
                } else {
                    robot.leftDrive.setPower(-robot.LOW_TURN_POWER);
                    robot.rightDrive.setPower(robot.LOW_TURN_POWER);
                }
            }
            updateHeading();
        }
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);
    }

    public void updateHeading() {
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = robot.imu.getGravity();
        currHeading = angles.firstAngle;
        telemetry.addData("Heading: ", currHeading);
        telemetry.update();
    }
//    String formatAngle(AngleUnit angleUnit, double angle) {
//        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
//    }
//
//    String formatDegrees(double degrees){
//        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
//    }

}
