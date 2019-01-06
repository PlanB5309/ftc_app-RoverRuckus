package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RaiseMineralLift {
    RobotHardware robot;
    Telemetry telemetry;

    public RaiseMineralLift(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void setHalfway() {
        robot.mineralMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mineralMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int target = 8500;
        robot.mineralMotor.setTargetPosition(target);
        robot.mineralMotor.setPower(0.5);
        while (robot.liftMotor.isBusy()) {
            telemetry.addData("lift clicks", robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.liftMotor.setPower(0);
    }

    public void setDown() {
        robot.mineralMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mineralMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int target = 0;
                robot.mineralMotor.setTargetPosition(target);
        robot.mineralMotor.setPower(0.5);
        while (robot.liftMotor.isBusy()) {
            Thread.yield();
            telemetry.addData("lift clicks", robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.liftMotor.setPower(0);
    }
}
