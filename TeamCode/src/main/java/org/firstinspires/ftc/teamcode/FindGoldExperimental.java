package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.Collections;
import java.util.List;

public class FindGoldExperimental {
    RobotHardware robot;
    Telemetry telemetry;

    public FindGoldExperimental(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public int run() {
        List<Recognition> initialRecognitions = robot.tfod.getUpdatedRecognitions();
        List<Recognition> bottomRecognitions;
        int goldPosition = robot.CENTER;
        int mineralsFound = 0;
        long time = System.currentTimeMillis();
        int[] results = new int[20];
        while (System.currentTimeMillis() < time + 5000) {
            for(int i=0;i<20;i++) {
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
                    results[i] = goldPosition;
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
                    results[i] = goldPosition;
                }
                else {
                    i--;
                }
            }
        }
        int right =0;
        int left =0;
        int center =0;
        for(int i =0;i<results.length;i++){
            if(results[i] == robot.RIGHT)
                right++;
            else if(results[i] == robot.LEFT)
                left++;
            else
                center++;
        }
        if(right > left && right > center)
            return robot.RIGHT;
        else if(left>right && left>center)
            return robot.LEFT;
        else
            return robot.CENTER;
    }
}
