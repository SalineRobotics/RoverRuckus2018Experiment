package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Pushbot: Teleop Mecanum", group="Pushbot")
public class PushbotTeleopMecanum extends LinearOpMode {

    /* Declare OpMode members. */
    SmsRCHardwarePushbotMecanum robot = new SmsRCHardwarePushbotMecanum();   // Use a Pushbot's hardware

    float[] hsvValues = new float[3];
    final float values[] = hsvValues;

    @Override
    public void runOpMode() {
        float powerAdjuster = 0.5f;

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)


        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1RightY = -gamepad1.right_stick_y;
            float gamepad1RightX = gamepad1.right_stick_x;

           float gamepad2LeftY = -gamepad2.left_stick_y;
           float gamepad2RightY = -gamepad2.right_stick_y;

           float gamepad2RightTrigger = gamepad2.right_trigger;
           float gamepad2LeftTrigger = gamepad2.left_trigger;


            if (Math.abs(gamepad1LeftY) < 0.2) {
                gamepad1LeftY = 0;
            }
            if (Math.abs(gamepad1LeftX) < 0.2) {
                gamepad1LeftX = 0;
            }
            if (Math.abs(gamepad1RightX) < 0.2) {
                gamepad1RightX = 0;
            }
            if (Math.abs(gamepad1RightY) < 0.2) {
                gamepad1RightY = 0;
            }

            float leftFront = 0;
            float leftBack = 0;
            float rightFront = 0;
            float rightBack = 0;

            // y-axis motion
            if (Math.abs(gamepad1LeftY) > Math.abs(gamepad1RightX) && Math.abs(gamepad1LeftY) > Math.abs(gamepad1LeftX)) {//Activates if y is largest {
                leftFront = (gamepad1LeftY);
                rightFront = (gamepad1LeftY);
                leftBack = (gamepad1LeftY);
                rightBack = (gamepad1LeftY);
            }
            // x-axis motion
            else if (Math.abs(gamepad1RightX) > Math.abs(gamepad1LeftY) && Math.abs(gamepad1RightX) > Math.abs(gamepad1LeftX)) {//Activates if x is largest {
                leftFront = (gamepad1RightX);
                rightFront = (gamepad1RightX * -1);
                leftBack = (gamepad1RightX * -1);
                rightBack = (gamepad1RightX);
            }

            // gamepad1LeftX-axis motion
            else if (Math.abs(gamepad1LeftX) > Math.abs(gamepad1LeftY) && Math.abs(gamepad1LeftX) > Math.abs(gamepad1RightX)) {
                leftFront = (gamepad1LeftX);
                rightFront = (gamepad1LeftX * -1);
                leftBack = (gamepad1LeftX);
                rightBack = (gamepad1LeftX * -1);
            }
            // Otherwise sticks are not pushed
            else {
                leftFront = (0);
                leftBack = (0);
                rightFront = (0);
                rightBack = (0);
            }

            // clip the right/left values so that the values never exceed +/- 1
            rightFront = Range.clip(rightFront, -1, 1);
            leftFront = Range.clip(leftFront, -1, 1);
            leftBack = Range.clip(leftBack, -1, 1);
            rightBack = Range.clip(rightBack, -1, 1);
          float armMove = Range.clip(gamepad2LeftY,-1,1);
          float armEx = Range.clip(gamepad2RightY,-1,1);

            // write the values to the motors
            if (robot.frontRightDrive != null) robot.frontRightDrive.setPower(rightFront);
            if (robot.frontLeftDrive != null) robot.frontLeftDrive.setPower(leftFront);
            if (robot.rearRightDrive != null) robot.rearRightDrive.setPower(rightBack);
            if (robot.rearLeftDrive != null) robot.rearLeftDrive.setPower(leftBack);
            if (robot.armExtend != null) robot.armExtend.setPower(armEx);
            if (robot.armMove != null) robot.armMove.setPower(armMove);
            //robot.armMove.setPower(armUpDown);
            //robot.armExtend.setPower(armEx);

            if (robot.collector != null) {
                if (gamepad2LeftTrigger > 0f) {
                    robot.collector.setPower(1);
                } else if (gamepad2RightTrigger > 0f) {
                    robot.collector.setPower(-1);
                } else {
                    robot.collector.setPower(0);
                }
            }

            telemetry.update();

        }
    }
}