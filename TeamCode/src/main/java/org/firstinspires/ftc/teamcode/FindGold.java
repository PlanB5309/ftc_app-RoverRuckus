/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

public class FindGold {
    RobotHardware robot;
    Telemetry telemetry;

    public FindGold(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public int run() {
        boolean notFound = true;
        while (notFound) {
            List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
            }

//            Recognition recOne;
//            Recognition recTwo;
//            if(updatedRecognitions.get(1).getBottom() > updatedRecognitions.get(0).getBottom()) {
//                recOne = updatedRecognitions.get(0);
//                recTwo = updatedRecognitions.get(1);
//            }
//            else {
//                recOne = updatedRecognitions.get(1);
//                recTwo = updatedRecognitions.get(0);
//            }
            Recognition lowest = updatedRecognitions.get(0);
            for (Recognition recognition : updatedRecognitions) {
                if(lowest.equals(recognition)){
                }
                else if(recognition.getBottom() < lowest.getBottom())
                    lowest = recognition;
            }
            updatedRecognitions.remove(lowest);
            Recognition secondLowest = updatedRecognitions.get(0);
            for (Recognition recognition : updatedRecognitions) {
                if(secondLowest.equals(recognition)){
                }
                else if(recognition.getBottom() < secondLowest.getBottom())
                    secondLowest = recognition;
            }

            if(lowest.getLeft() < secondLowest.getLeft()){
                if(lowest.getLabel().equals(robot.LABEL_GOLD_MINERAL)){
                    telemetry.addData("Gold Mineral Position", "Left");
                    return robot.LEFT;
                }
                else if(secondLowest.getLabel().equals(robot.LABEL_GOLD_MINERAL)){
                    telemetry.addData("Gold Mineral Position", "Center");
                    return robot.CENTER;
                }
                else{
                    telemetry.addData("Gold Mineral Position", "Right");
                    return robot.RIGHT;
                }
            }
            else{
                if(secondLowest.getLabel().equals(robot.LABEL_GOLD_MINERAL)){
                    telemetry.addData("Gold Mineral Position", "Left");
                    return robot.LEFT;
                }
                else if(lowest.getLabel().equals(robot.LABEL_GOLD_MINERAL)){
                    telemetry.addData("Gold Mineral Position", "Center");
                    return robot.CENTER;
                }
                else{
                    telemetry.addData("Gold Mineral Position", "Right");
                    return robot.RIGHT;
                }
            }

//            if (recognition.getConfidence() > robot.TENSORFLOW_SENSETIVITY) {
//                if (recognition.getLabel().equals(robot.LABEL_GOLD_MINERAL)) {
//                    goldMineralX = (int) recognition.getLeft();
//                } else if (silverMineral1X == -1) {
//                    silverMineral1X = (int) recognition.getLeft();
//                }
//            }

//            if (goldMineralX != -1) {
//                if (silverMineral1X < goldMineralX) {
//                    telemetry.addData("Gold Mineral Position", "Center");
//                    return robot.CENTER;
//                } else {
//                    telemetry.addData("Gold Mineral Position", "Left");
//                    return robot.LEFT;
//                }
//            } else {
//                telemetry.addData("Gold Mineral Position", "Right");
//                return robot.RIGHT;
//            }



//                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                            telemetry.addData("Gold Mineral Position", "Left");
//                            return robot.LEFT;
//                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                            telemetry.addData("Gold Mineral Position", "Right");
//                            return robot.RIGHT;
//                        } else {
//                            telemetry.addData("Gold Mineral Position", "Center");
//                            return robot.CENTER;
//                        }
//                    }
        }
        telemetry.update();
        return robot.CENTER;
    }
}
