package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Teleop", group="Robot")
public class Teleop extends OpMode {
    private double armPower = 0;

    RobotHardware robot = new RobotHardware(telemetry);

    public void init () {
        robot.initTeleop(hardwareMap);
    }

    //When play is hit
    public void start () {
        robot.rakeServo.setPosition(robot.RAKE_TELEOP);
        robot.markerServo.setPosition(robot.MARKER_IN);
    }

    float getStickValue(float joy){
        if(-joy < -robot.DEADZONE){
            return ((-joy+robot.DEADZONE)/(1-robot.DEADZONE));
        }
        else if (-joy > robot.DEADZONE){
            return ((-joy-robot.DEADZONE)/(1-robot.DEADZONE));
        }
        else {
            return(0);
        }
    }

    public void loop () {

        telemetry.addData("Lift Motor: ", robot.liftMotor.getCurrentPosition());
        telemetry.addData("Bucket Servo: ", robot.bucketServo.getPosition());
        telemetry.addData("Arm Motor: ", robot.armMotor.getCurrentPosition());
        telemetry.addData("Left Drive", robot.leftDrive.getCurrentPosition());
        telemetry.addData("Right Drive", robot.rightDrive.getCurrentPosition());
        telemetry.addData("Mineral Lift:", robot.mineralMotor.getCurrentPosition());
        telemetry.update();


        //Set driving controls on gamepad 1
//        double drive = -gamepad1.left_stick_y;
//        double turn  =  gamepad1.right_stick_x;
//        double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
//        double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
//        robot.leftDrive.setPower(leftPower);
//        robot.rightDrive.setPower(rightPower);
        //Primary drive method.

        float ly = getStickValue(gamepad1.left_stick_y);
        float rx = getStickValue(gamepad1.right_stick_x);
        robot.leftDrive.setPower(ly - rx);
        robot.rightDrive.setPower(ly + rx);


        //Marker dump controls
        if (gamepad1.b) { //Dump
            robot.markerServo.setPosition(robot.MARKER_OUT);
        }
        else if (gamepad1.x) {
            robot.markerServo.setPosition(robot.MARKER_IN);
        }

        //Extender motor controls
        if (gamepad1.y) {
            robot.extenderMotor.setPower(0.75);
        }
        else if (gamepad1.a && !gamepad1.start) {
            robot.extenderMotor.setPower(-0.75);
        }
        else {
            robot.extenderMotor.setPower(0);
        }

        //Crater rake controls
        if (gamepad1.left_bumper){
            robot.rakeServo.setPosition(Range.clip(robot.rakeServo.getPosition()-0.015, robot.RAKE_DOWN, robot.RAKE_TELEOP));
        }
        else if(gamepad1.left_trigger > 0.5){
            robot.rakeServo.setPosition(Range.clip(robot.rakeServo.getPosition()+0.015, robot.RAKE_DOWN, robot.RAKE_TELEOP));
        }
        //Opening and Closing lifter claws with left bumper+trigger
        if (gamepad2.left_bumper) {
            robot.hookServo.setPosition(robot.RIGHT_CLAW_OPEN);
        }
        if (gamepad2.left_trigger > 0.5) {
            robot.hookServo.setPosition(robot.RIGHT_CLAW_CLOSED);
        }

        //Setting lift motor power to arrow pad
        if (gamepad2.dpad_up) {
            robot.liftMotor.setPower(1);
            robot.rakeServo.setPosition(robot.RAKE_TELEOP);
        }
        else if (gamepad2.dpad_down) {
            robot.liftMotor.setPower(-1);
        }
        else {
            robot.liftMotor.setPower(0);
        }

        //Setting the sweeper motor settings
        if (gamepad2.a) {
            robot.sweeperMotor.setPower(1);
        }
        else if (gamepad2.y) {
            robot.sweeperMotor.setPower(-1);
        }
        else {
            robot.sweeperMotor.setPower(0);
        }

        //Mineral lift motor controls
        if (gamepad2.right_trigger > 0.5 || gamepad2.right_bumper) { //Bumper - going up
            if (gamepad2.right_trigger > 0.5) {
                robot.mineralMotor.setPower(1);
            } else if (gamepad2.right_bumper) {
                robot.mineralMotor.setPower(-1);
            }
        }
        else {
            robot.mineralMotor.setPower(0);
        }

        //Mineral arm motor
        if (Math.abs(gamepad2.right_stick_y) > robot.JOYSTICK_BLANK_VALUE) {
                armPower = gamepad2.right_stick_y*gamepad2.right_stick_y;
                if (gamepad2.right_stick_y > 0) {
                    armPower *= -1;
                }
                robot.armMotor.setPower(armPower);
                robot.bucketServo.setPosition(robot.armMotor.getCurrentPosition() * 0.0002433);
        }
        else {
            robot.armMotor.setPower(0);
        }
        if (gamepad2.right_stick_button) {
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //Bucket servo controls
        if (Math.abs(gamepad2.left_stick_y) > robot.JOYSTICK_BLANK_VALUE) {
            robot.bucketServo.setPosition(robot.bucketServo.getPosition() + (robot.BUCKET_TURN_VALUE * gamepad2.left_stick_y));
//            if (gamepad2.left_stick_y > 0) {
//                robot.bucketServo.setPosition(robot.bucketServo.getPosition() - robot.BUCKET_TURN_VALUE);
//            }
//            else {
//                robot.bucketServo.setPosition(robot.bucketServo.getPosition() + robot.BUCKET_TURN_VALUE);
//            }
        }
        else if (gamepad2.x) {
            robot.bucketServo.setPosition(robot.BUCKET_CARRY_POSITION);
        }
        else if (gamepad2.b) {
            robot.bucketServo.setPosition(robot.BUCKET_SCOOP_POSITION);
        }
        else if (gamepad2.left_stick_button /*&& !robot.bucketServo.isBusy()*/) {
            robot.bucketServo.setPosition(robot.BUCKET_DUMP_POSITION);
        }

    }
}
