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
        int target = -150;
        robot.mineralMotor.setTargetPosition(target);
        robot.mineralMotor.setPower(0.5);
    }

    public void setDown() {
        int target = 0;
        robot.mineralMotor.setTargetPosition(target);
        robot.mineralMotor.setPower(0.5);
        while (robot.mineralMotor.isBusy()) {
            Thread.yield();
            telemetry.addData("mineral lift clicks", robot.mineralMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.mineralMotor.setPower(0);
    }
}
