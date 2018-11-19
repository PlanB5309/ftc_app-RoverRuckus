package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Teleop", group="Robot")
public class Teleop extends OpMode {

    RobotHardware robot = new RobotHardware(telemetry);


    public void init () {
        robot.init(hardwareMap);
    }
    public void loop () {
        telemetry.addData("lifter motor: ", robot.liftMotor.getCurrentPosition());
        telemetry.addData("bucket motor: ", robot.bucketMotor.getCurrentPosition());
        telemetry.addData("arm motor: ", robot.armMotor.getCurrentPosition());
        telemetry.update();


        //Set driving controls on gamepad 1
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

        //Opening and Closing lifter claws with left bumper+trigger
        if (gamepad2.left_bumper) {
//            robot.leftClaw.setPosition(robot.LEFT_CLAW_OPEN);
            robot.rightClaw.setPosition(robot.RIGHT_CLAW_OPEN);
        }
        if (gamepad2.left_trigger > 0.5) {
//            robot.leftClaw.setPosition(robot.LEFT_CLAW_CLOSED);
            robot.rightClaw.setPosition(robot.RIGHT_CLAW_CLOSED);
        }

        //Setting lift motor power to arrow pad
        if (gamepad2.dpad_up) {
            robot.liftMotor.setPower(1);
        }
        else if (gamepad2.dpad_down) {
            robot.liftMotor.setPower(-1);
        }
        else {
            robot.liftMotor.setPower(0);
        }

        //Setting the mineral sweeper motor settings
        if (gamepad2.a) {
            robot.sweeperMotor.setPower(1);
        }
        else if (gamepad2.y) {
            robot.sweeperMotor.setPower(-1);
        }
        else {
            robot.sweeperMotor.setPower(0);
        }

        //Mineral arm controls
        if (Math.abs(gamepad2.right_stick_y) > robot.JOYSTICK_BLANK_VALUE) {
                robot.armMotor.setPower(gamepad2.right_stick_y);
        }
        else {
            robot.armMotor.setPower(0);
        }

        if (Math.abs(gamepad2.left_stick_y) > robot.JOYSTICK_BLANK_VALUE) {
//            robot.bucketMotor.setPower(gamepad2.left_stick_y);
            if (gamepad2.left_stick_y > 0) {
                robot.bucketMotor.setPower(0.1);
            }
            else {
                robot.bucketMotor.setPower(-0.1);
            }
        }
        else {
            robot.bucketMotor.setPower(0);
        }

        if (gamepad2.left_stick_button && !robot.bucketMotor.isBusy()) {
            int bucketPosition = robot.bucketMotor.getCurrentPosition();
            int targetPosition  = bucketPosition + robot.BUCKET_TURN_VALUE;
            robot.bucketMotor.setTargetPosition(targetPosition);
            robot.bucketMotor.setPower(-0.5);
        }

    }
}
