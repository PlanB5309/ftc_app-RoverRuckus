///* Copyright (c) 2018 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//
//import java.util.List;
//
//public class FindCraterGold {
//    RobotHardware robot;
//    Telemetry telemetry;
//
//    public FindCraterGold(RobotHardware robot, Telemetry telemetry) {
//        this.robot = robot;
//        this.telemetry = telemetry;
//    }
//
//    public int run() {
//        boolean notFound = true;
//        while (notFound) {
//            List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
//            if (updatedRecognitions != null) {
//                telemetry.addData("# Object Detected", updatedRecognitions.size());
//                if (updatedRecognitions.size() == 2) {
//                    int goldMineralX = -1;
//                    int silverMineral1X = -1;
//                    for (Recognition recognition : updatedRecognitions) {
//                        if(recognition.getConfidence() > robot.TENSORFLOW_SENSETIVITY) {
//                            if (recognition.getLabel().equals(robot.LABEL_GOLD_MINERAL)) {
//                                goldMineralX = (int) recognition.getLeft();
//                            } else if (silverMineral1X == -1) {
//                                silverMineral1X = (int) recognition.getLeft();
//                            }
//                        }
//                    }
//                    if(goldMineralX != -1){
//                        if(silverMineral1X > goldMineralX){
//                            telemetry.addData("Gold Mineral Position", "Center");
//                            return robot.CENTER;
//                        }
//                        else{
//                            telemetry.addData("Gold Mineral Position", "Left");
//                            return robot.LEFT;
//                        }
//                    }
//                    else{
//                        telemetry.addData("Gold Mineral Position", "Right");
//                        return robot.RIGHT;
//                    }
//
////                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
////                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
////                            telemetry.addData("Gold Mineral Position", "Left");
////                            return robot.LEFT;
////                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
////                            telemetry.addData("Gold Mineral Position", "Right");
////                            return robot.RIGHT;
////                        } else {
////                            telemetry.addData("Gold Mineral Position", "Center");
////                            return robot.CENTER;
////                        }
////                    }
//                }
//                telemetry.update();
//            }
//        }
//        return robot.CENTER;
//    }
//}
