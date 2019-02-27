package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class RobotHardware
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor extenderMotor = null;
    public Servo markerServo = null;
    public Servo hookServo = null;
    public Servo bucketServo = null;
    public DcMotor liftMotor = null;
    public DcMotor sweeperMotor = null;
    public DcMotor armMotor = null;
    public DcMotor bucketMotor = null;
    BNO055IMU imu;
    public DcMotor mineralMotor = null;
    public Servo rakeServo = null;
    public final int RIGHT = 112;
    public final int LEFT = 211;
    public final int CENTER = 121;
    public final double HIGH_TURN_POWER = 0.3;
    public final double LOW_TURN_POWER = 0.07;
    public final double TENSORFLOW_SENSETIVITY = 0.9;
    public final double RIGHT_CLAW_OPEN = 1.0;
    public final double BUCKET_ARM_RATIO = -0.037;
    public final double RIGHT_CLAW_CLOSED = 0.7;
    public final double JOYSTICK_BLANK_VALUE = 0.05;
    public final double BUCKET_TURN_VALUE = 0.05;
    public final double BUCKET_SCOOP_POSITION = 0.24;
    public final double BUCKET_CARRY_POSITION = 0.07;
    public final double BUCKET_DUMP_POSITION = 0;
    public final float DEADZONE = .15f;
    public final double RAKE_INIT = 0.68;
    public final double RAKE_TELEOP = 0.55 ;
    public final double RAKE_DOWN = 0;
    public final double MARKER_IN = 0.07;
    public final double MARKER_OUT = 1;
    static final double     COUNTS_PER_MOTOR_REV    = 1180 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.54 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    protected static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    protected static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    protected static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    protected static final String VUFORIA_KEY = "AYboCe//////AAAAGZ74ine8W01csxwSLrlrFU8SCehSLWIwpwUNici0XIz9RK5ZR9rkALDwHUE/860ACZBO2RMrFbWIFC27Ri6dW0fsZH24ckZfYxsXdmBYmch/6XuhsWx75wLnb7+4ZthJQLMZ0wfORFa+6bCn6xDvRGIM+0wqrKZOEv1J+F16mj3MlG4Mx65/DBFc2t8ag9LoVoE2gCcKX+HmeA1aXfAWJVFfp1nhXXUMMyftFe1zaexsXoVbvuCcpyz8aqqZpXBBBuBoRGsxfzuQ6Tq2UlvzzFazzDdFliwmYFdmAQyfae7HORmTHTZs31eVlOB+cJPzciASiUP+KQz7lzRidcSruR3U06PYPv9P1uM9vf4KYwdY";
    protected VuforiaLocalizer vuforia;
    protected TFObjectDetector tfod;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    Telemetry telemetry;
    public RobotHardware(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        extenderMotor = hwMap.get(DcMotor.class, "extenderMotor");
        leftDrive  = hwMap.get(DcMotor.class, "leftDrive");
        rightDrive = hwMap.get(DcMotor.class, "rightDrive");
        liftMotor    = hwMap.get(DcMotor.class, "liftMotor");
        sweeperMotor    = hwMap.get(DcMotor.class, "sweeperMotor");
        armMotor = hwMap.get(DcMotor.class, "armMotor");
        mineralMotor = hwMap.get(DcMotor.class, "mineralMotor");
        bucketMotor = hwMap.get(DcMotor.class, "bucketMotor");
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        liftMotor.setPower(0);
        sweeperMotor.setPower(0);
        extenderMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bucketMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mineralMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        markerServo = hwMap.get(Servo.class, "markerServo");
        bucketServo = hwMap.get(Servo.class, "bucketServo");
        hookServo = hwMap.get(Servo.class, "hookServo");
        rakeServo = hwMap.get(Servo.class, "rakeServo");
        rakeServo.setPosition(RAKE_INIT);
        markerServo.setPosition(MARKER_IN);
        hookServo.setPosition(RIGHT_CLAW_CLOSED);
        bucketServo.setPosition(BUCKET_SCOOP_POSITION);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters();

        param.vuforiaLicenseKey = VUFORIA_KEY;
        param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(param);
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfodParameters.minimumConfidence = TENSORFLOW_SENSETIVITY;
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        tfod.activate();
        telemetry.addData("Encoder Value: ", mineralMotor.getCurrentPosition());
        telemetry.addData("Initialization Complete: ", "Yay");
        telemetry.update();
    }
    public void initTeleop(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "leftDrive");
        rightDrive = hwMap.get(DcMotor.class, "rightDrive");
        liftMotor    = hwMap.get(DcMotor.class, "liftMotor");
        sweeperMotor    = hwMap.get(DcMotor.class, "sweeperMotor");
        armMotor = hwMap.get(DcMotor.class, "armMotor");
        extenderMotor = hwMap.get(DcMotor.class, "extenderMotor");
        mineralMotor = hwMap.get(DcMotor.class, "mineralMotor");
        bucketMotor = hwMap.get(DcMotor.class, "bucketMotor");
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        liftMotor.setPower(0);
        sweeperMotor.setPower(0);
        extenderMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bucketMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mineralMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize ALL installed servos.
        markerServo = hwMap.get(Servo.class, "markerServo");
        bucketServo = hwMap.get(Servo.class, "bucketServo");
        hookServo = hwMap.get(Servo.class, "hookServo");
        rakeServo = hwMap.get(Servo.class, "rakeServo");
        hookServo.setPosition(RIGHT_CLAW_CLOSED);
        bucketServo.setPosition(BUCKET_SCOOP_POSITION);
        telemetry.addData("Initialization Complete: ", ":)");
        telemetry.update();
    }

}

