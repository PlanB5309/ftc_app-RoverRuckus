package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.*;

public class FindGold {
    RobotHardware robot;
    Telemetry telemetry;

    public FindGold(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public int run() {
        List<Recognition> initialRecognitions = robot.tfod.getUpdatedRecognitions();
        List<Recognition> bottomRecognitions;
        int goldPosition = robot.CENTER;
        int mineralsFound = 0;
        long time = System.currentTimeMillis();
        while (System.currentTimeMillis() < time + 5000 && mineralsFound < 2) {
            mineralsFound = initialRecognitions.size();
            telemetry.addData("# Object Detected", mineralsFound);
            if (mineralsFound > 2) {
                Collections.sort(initialRecognitions, new BottomSort());
                bottomRecognitions = initialRecognitions.subList(0, 2);
                Collections.sort(bottomRecognitions, new LeftSort());
                if (bottomRecognitions.get(0).getLabel().equals(robot.LABEL_GOLD_MINERAL)) {
                    goldPosition = robot.LEFT;
                } else if (bottomRecognitions.get(1).getLabel().equals(robot.LABEL_GOLD_MINERAL)) {
                    goldPosition = robot.CENTER;
                } else {
                    goldPosition = robot.RIGHT;
                }
                telemetry.addData("Array Position 0 Left: ", bottomRecognitions.get(0).getLeft());
                telemetry.addData("Array Position 1 Left: ", bottomRecognitions.get(1).getLeft());
                telemetry.addData("Array Position 0 Bottom: ", bottomRecognitions.get(0).getBottom());
                telemetry.addData("Array Position 1 Bottom: ", bottomRecognitions.get(1).getBottom());
                telemetry.addData("Gold Position: ", goldPosition);
                telemetry.update();
            }
            if (mineralsFound == 2) {
                Collections.sort(initialRecognitions, new LeftSort());
                if (initialRecognitions.get(0).getLabel().equals(robot.LABEL_GOLD_MINERAL)) {
                    goldPosition = robot.LEFT;

                } else if (initialRecognitions.get(1).getLabel().equals(robot.LABEL_GOLD_MINERAL)) {
                    goldPosition = robot.CENTER;
                } else {
                    goldPosition = robot.RIGHT;
                }
                telemetry.addData("Array Position 0 Left: ", initialRecognitions.get(0).getLeft());
                telemetry.addData("Array Position 1 Left: ", initialRecognitions.get(1).getLeft());
                telemetry.addData("Gold Position: ", goldPosition);
                telemetry.update();
            }
        }
        return goldPosition;
    }
}
