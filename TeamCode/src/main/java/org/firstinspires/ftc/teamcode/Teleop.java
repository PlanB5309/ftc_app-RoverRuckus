package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@TeleOp(name="Teleop", group="Robot")
public class Teleop extends OpMode {

    RobotHardware robot = new RobotHardware(telemetry);
    BNO055IMU imu;

    public void init () {
        robot.init(hardwareMap);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //When play is hit
    public void start () {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void loop () {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData("Lift Motor: ", robot.liftMotor.getCurrentPosition());
        telemetry.addData("Bucket Motor: ", robot.bucketMotor.getCurrentPosition());
        telemetry.addData("Arm Motor: ", robot.armMotor.getCurrentPosition());
        telemetry.addData("Left Drive", robot.leftDrive.getCurrentPosition());
        telemetry.addData("Right Drive", robot.rightDrive.getCurrentPosition());
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

        //Mineral arm lift controls
        if (gamepad2.right_trigger > 0.5 || gamepad2.right_bumper) {
            if (gamepad2.right_trigger > 0.5) {
                robot.mineralMotor.setPower(0.5);
            } else if (gamepad2.right_bumper) {
                robot.mineralMotor.setPower(-0.5);
            }
        }
        else {
            robot.mineralMotor.setPower(0);
        }

        //Mineral arm motor
        if (Math.abs(gamepad2.right_stick_y) > robot.JOYSTICK_BLANK_VALUE) {
                robot.armMotor.setPower(gamepad2.right_stick_y);
        }
        else {
            robot.armMotor.setPower(0);
        }

        //Bucket motor controls
        if (Math.abs(gamepad2.left_stick_y) > robot.JOYSTICK_BLANK_VALUE) {
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
