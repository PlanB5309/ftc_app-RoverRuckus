package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift {
    Telemetry telemetry;
    RobotHardware robot = new RobotHardware(telemetry);
    public Lift(){}
    public void power(double pow){
        robot.liftMotor.setPower(pow);
    }
    public void target(double clicks){
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        int target = (int) (clicks);
        robot.liftMotor.setTargetPosition(target);
        robot.liftMotor.setPower(1);

        while (robot.liftMotor.isBusy()) {
            Thread.yield();
            robot.liftMotor.setPower(1);
            
        }
        robot.liftMotor.setPower(0);
    }
    public void off(){
        robot.liftMotor.setPower(0);
    }
}
