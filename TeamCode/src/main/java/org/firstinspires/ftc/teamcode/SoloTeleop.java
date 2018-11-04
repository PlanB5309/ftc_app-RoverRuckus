package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Solo Teleop", group="Robot")
public class SoloTeleop extends OpMode{

    RobotHardware robot = new RobotHardware(telemetry);
    boolean sweeperStatus = false;
    boolean direction = true;


    public void init () {
        robot.init(hardwareMap);
    }
    public void loop() {

        //Set driving controls on gamepad 1

        robot.leftDrive.setPower(gamepad1.left_stick_y);
        robot.rightDrive.setPower(gamepad1.right_stick_y);

        //Opening and Closing lifter claws with left bumper+trigger
        if (gamepad1.b) {
            robot.leftClaw.setPosition(robot.LEFT_CLAW_OPEN);
            robot.rightClaw.setPosition(robot.RIGHT_CLAW_OPEN);
        }
        if (gamepad1.y) {
            robot.leftClaw.setPosition(robot.LEFT_CLAW_CLOSED);
            robot.rightClaw.setPosition(robot.RIGHT_CLAW_CLOSED);
        }

        //Setting lift motor power to arrow pad
        if (gamepad1.dpad_up) {
            robot.liftMotor.setPower(1);
        }
        else if (gamepad1.dpad_down) {
            robot.liftMotor.setPower(-1);
        }
        else {
            robot.liftMotor.setPower(0);
        }

        //Setting the mineral sweeper motor settings

        if(gamepad1.start){
            sweeperStatus = !sweeperStatus;
        }
        if(gamepad1.a)
            direction = false;
        if(gamepad1.x)
            direction = true;
        if(sweeperStatus){
            if(direction){
                robot.sweeperMotor.setPower(1);
            }
            else{
                robot.sweeperMotor.setPower(-1);
            }
        }
        else
            robot.sweeperMotor.setPower(0);

    }
}
