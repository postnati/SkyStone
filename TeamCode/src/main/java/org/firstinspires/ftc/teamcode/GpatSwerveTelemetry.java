/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bothardware.SwerveBotHardware;

import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.Collections;

/**
 * {@link GpatSwerveTelemetry} illustrates various ways in which telemetry can be
 * transmitted from the robot controller to the driver station. The sample illustrates
 * numeric and text data, formatted output, and optimized evaluation of expensive-to-acquire
 * information. The telemetry {@link Telemetry#log() log} is illustrated by scrolling a poem
 * to the driver station.
 *
 * @see Telemetry
 */
@TeleOp(name = "Gpat Swerve Telemetry", group = "Gpat")
//@Disabled
public class GpatSwerveTelemetry extends LinearOpMode {
    private DecimalFormat df = new DecimalFormat("#.##");
    private SwerveBotHardware swerveBotHardware;

    @Override
    public void runOpMode() {
        swerveBotHardware = new SwerveBotHardware(df);
        swerveBotHardware.init(hardwareMap);

        //----------------------------------------------------------------------------------------------
        // Telemetry
        //----------------------------------------------------------------------------------------------
        /* we keep track of how long it's been since the OpMode was started, just
         * to have some interesting data to show */
        ElapsedTime opmodeRunTime = new ElapsedTime();
        // We show the log in oldest-to-newest order, as that's better for poetry
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        // We can control the number of lines shown in the log
        telemetry.log().setCapacity(6);

        /* Wait until we've been given the ok to go. For something to do, we emit the
         * elapsed time as we sit here and wait. If we didn't want to do anything while
         * we waited, we would just call {@link #waitForStart()}.
         */
        while (!isStarted()) {
            telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
            telemetry.update();
            idle();
        }

        // Ok, we've been given the ok to go

        // Reset to keep some timing stats for the post-'start' part of the opmode
        opmodeRunTime.reset();
        runOpModeLoop(opmodeRunTime);
    }

    private void runOpModeLoop(ElapsedTime opmodeRunTime) {
        int loopCount = 1;

        //----------------------------------------------------------------------------------------------
        // MAIN LOOP: Go go gadget robot!
        //----------------------------------------------------------------------------------------------
        while (opModeIsActive()) {
            // Loop Timing
            telemetry.addLine("Status | ")
                    .addData("loop count", loopCount)
                    .addData("ms/loop", df.format(opmodeRunTime.milliseconds() / loopCount));

            //----------------------------------------------------------------------------------------------
            // Joysticks
            //----------------------------------------------------------------------------------------------
            // Show joystick information as some other illustrative data
            float leftStickX = gamepad1.left_stick_x;
            float leftStickY = -gamepad1.left_stick_y;
            float rightStickX = gamepad1.right_stick_x;
            float rightStickY = -gamepad1.right_stick_y;
            double heading = swerveBotHardware.getHeading();

            //----------------------------------------------------------------------------------------------
            // Swerve
            //----------------------------------------------------------------------------------------------
            calculateSwerve(leftStickX, leftStickY, rightStickX, rightStickY, heading);


            swerveBotHardware.addServoTelemetry(telemetry);

            //Transmit the telemetry to the driver station, subject to throttling.
            telemetry.update();

            // Update loop info
            loopCount++;
        }
    }

    private void calculateSwerve(double leftStickX, double leftStickY, double RCW, double rightStickY, double heading) {
        double BASE_LENGTH = 30;
        double BASE_WIDTH = 24;
        double baseRadius = Math.hypot(BASE_LENGTH, BASE_WIDTH);

        // No field orientation adjustment
        // Y  FWD
        // X  STR
        // X of Rotational Joystick 2 // RCW
        // =SQRT(_L^2+_W^2)  // R

        // Adjust robot to field rotation to ensure movement is always from the drivers perpective
        double FWD = leftStickY * Math.cos(heading) + leftStickX * Math.sin(heading);
        double STR = -leftStickY * Math.sin(heading) + leftStickX * Math.cos(heading);

        double aCoefficient = STR - RCW * (BASE_LENGTH / baseRadius);
        double bCoefficient = STR + RCW * (BASE_LENGTH / baseRadius);
        double cCoefficient = FWD - RCW * (BASE_WIDTH / baseRadius);
        double dCoefficient = FWD + RCW * (BASE_WIDTH / baseRadius);

        double rfWheelSpeed = Math.hypot(bCoefficient, cCoefficient);
        double lfWheelSpeed = Math.hypot(bCoefficient, dCoefficient);
        double rrWheelSpeed = Math.hypot(aCoefficient, dCoefficient);
        double lrWheelSpeed = Math.hypot(aCoefficient, cCoefficient);

        double maxWheelSpeed = Collections.max(Arrays.asList(rfWheelSpeed, lfWheelSpeed, rrWheelSpeed, lrWheelSpeed));

        if (maxWheelSpeed > 0) {
            rfWheelSpeed = rfWheelSpeed / maxWheelSpeed;
            lfWheelSpeed = lfWheelSpeed / maxWheelSpeed;
            rrWheelSpeed = rrWheelSpeed / maxWheelSpeed;
            lrWheelSpeed = lrWheelSpeed / maxWheelSpeed;
        }

        double rfWheelAngle = 0;
        double lfWheelAngle = 0;
        double rrWheelAngle = 0;
        double lrWheelAngle = 0;

        if (bCoefficient != 0 || cCoefficient != 0) {
            rfWheelAngle = Math.atan2(bCoefficient, cCoefficient) * 180 / Math.PI;
        }

        if (bCoefficient != 0 || dCoefficient != 0) {
            lfWheelAngle = Math.atan2(bCoefficient, dCoefficient) * 180 / Math.PI;
        }

        if (aCoefficient != 0 || dCoefficient != 0) {
            rrWheelAngle = Math.atan2(aCoefficient, dCoefficient) * 180 / Math.PI;
        }

        if (aCoefficient != 0 || cCoefficient != 0) {
            lrWheelAngle = Math.atan2(aCoefficient, cCoefficient) * 180 / Math.PI;
        }

        telemetry.addLine("heading | ").addData("d", df.format(heading));
        telemetry.addLine("left joystick | ")
                .addData("x", df.format(leftStickX))
                .addData("y", df.format(leftStickY));
        telemetry.addLine("right joystick | ")
                .addData("x", df.format(RCW))
                .addData("y", df.format(rightStickY));
        telemetry.addLine("field | ")
                .addData("FWD", df.format(FWD))
                .addData("STR", df.format(STR))
                .addData("RCW", df.format(RCW));
        telemetry.addLine("Co | ")
                .addData("r", df.format(baseRadius))
                .addData("a", df.format(aCoefficient))
                .addData("b", df.format(bCoefficient))
                .addData("c", df.format(cCoefficient))
                .addData("d", df.format(dCoefficient));
        telemetry.addLine("RF | ").addData("ws", df.format(rfWheelSpeed)).addData("wa", df.format(rfWheelAngle));
        telemetry.addLine("LF | ").addData("ws", df.format(lfWheelSpeed)).addData("wa", df.format(lfWheelAngle));
        telemetry.addLine("RR | ").addData("ws", df.format(rrWheelSpeed)).addData("wa", df.format(rrWheelAngle));
        telemetry.addLine("LR | ").addData("ws", df.format(lrWheelSpeed)).addData("wa", df.format(lrWheelAngle));


        /*
        30  // L
        24  // W

        Y  FWD
        X  STR
        X of Rotational Joystick 2 // RCW

        =SQRT(_L^2+_W^2)  // R

        9  =STR-RCW*(_L/_R) // A
        10 =STR+RCW*(_L/_R) // B
        11  =FWD-RCW*(_W/_R) // C
        12  =FWD+RCW*(_W/_R)  // D

        13  =SQRT(_B^2+_C^2)  // WS1
        14  =SQRT(_B^2+_D^2)  // WS2
        15  =SQRT(_A^2+_D^2)  // WS3
        16  =SQRT(_A^2+_C^2)  // WS4
        17  =MAX(A13:A16)


        =IF(_max>1,A13/_max,A13)
        =IF(_max>1,A14/_max,A14)
        =IF(_max>1,A15/_max,A15)
        =IF(_max>1,A16/_max,A16)

        =IF(AND(_C=0,_B=0),0,ATAN2(_C,_B)*180/PI())
        =IF(AND(_D=0,_B=0),0,ATAN2(_D,_B)*180/PI())
        =IF(AND(_D=0,_A=0),0,ATAN2(_D,_A)*180/PI())
        =IF(AND(_C=0,_A=0),0,ATAN2(_C,_A)*180/PI())


        V1x = Vx + (ωr)x = Vx + ωL/2
        V1y = Vy + (ωr)y = Vy – ωW/2
        V2x = Vx + (ωr)x = Vx + ωL/2
        V2y = Vy + (ωr)y = Vy + ωW/2

        V3x = Vx + (ωr)x = Vx – ωL/2
        V3y = Vy + (ωr)y = Vy + ωW/2
        V4x = Vx + (ωr)x = Vx – ωL/2
        V4y = Vy + (ωr)y = Vy – ωW/2


        Convert the joystick cartesian coordinates (x,y) into polar coordinates (r,θ)
        Subtract the gyro angle from the joystick angle
        Convert back to cartesian coordinates

        */
    }
}
