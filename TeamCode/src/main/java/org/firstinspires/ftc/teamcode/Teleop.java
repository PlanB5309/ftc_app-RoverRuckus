package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Teleop", group="Robot")
public class Teleop extends OpMode {

    RobotHardware robot = new RobotHardware(telemetry);

    private double leftClawOpen = 0.2;
    private double leftClawClosed = 0.7;
    private double rightClawOpen = 1.0;
    private double rightClawClosed = 0.65;

    public void init () {
        robot.init(hardwareMap);
    }
    public void loop () {

        //Set driving controls on gamepad 1
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

        //Opening and Closing lifter claws with left bumper+trigger
        if (gamepad2.left_bumper) {
            robot.leftClaw.setPosition(leftClawOpen);
            robot.rightClaw.setPosition(rightClawOpen);
        }
        if (gamepad2.left_trigger > 0.5) {
            robot.leftClaw.setPosition(leftClawClosed);
            robot.rightClaw.setPosition(rightClawClosed);
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

    }
}
