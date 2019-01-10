package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
@Disabled
@Autonomous(name="ForwardTest", group="Auto")



public class ForwardTest extends LinearOpMode {
    RobotHardware         robot   = new RobotHardware(telemetry);
//    LowerRobot lowerRobot = new LowerRobot(robot, telemetry);
//    OpenHooks openHooks = new OpenHooks(robot, telemetry);
//    PushGoldBlock pushGoldBlock = new PushGoldBlock(robot, telemetry);
//    DropTeamMarker dropTeamMarker = new DropTeamMarker(robot, telemetry);
    Drive drive = new Drive(robot,telemetry);
    GyroTurn gyroTurn = new GyroTurn(robot,telemetry);
    BNO055IMU imu;


    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        gyroTurn.right(90);
        sleep(2000);
        drive.forward(.5,12);
        sleep(2000);
        gyroTurn.left(90);
    }
}
