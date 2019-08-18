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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
    public DecimalFormat df = new DecimalFormat("#.##");

    //----------------------------------------------------------------------------------------------
    // DC Drive Motors
    //----------------------------------------------------------------------------------------------
    public DcMotor lfDriveMotor = null;
    public DcMotor rfDriveMotor = null;
    public DcMotor lrDriveMotor = null;
    public DcMotor rrDriveMotor = null;

    //----------------------------------------------------------------------------------------------
    // Servo Motors
    //----------------------------------------------------------------------------------------------
    public Servo lfDriveServo = null;
    public Servo rfDriveServo = null;
    public Servo lrDriveServo = null;
    public Servo rrDriveServo = null;

    //----------------------------------------------------------------------------------------------
    // Gryo
    //----------------------------------------------------------------------------------------------
    BNO055IMU imu; // The IMU sensor object
    Orientation angles; // State used for updating telemetry

    @Override
    public void runOpMode() {
        //----------------------------------------------------------------------------------------------
        // DC Drive Motors
        //----------------------------------------------------------------------------------------------
        // Define and Initialize Motors
        lfDriveMotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        rfDriveMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        lrDriveMotor = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rrDriveMotor = hardwareMap.get(DcMotor.class, "right_rear_drive");
        // Set motor direction
        lfDriveMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rfDriveMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        lrDriveMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rrDriveMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // Set all motors to run without encoders.
        lfDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lrDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rrDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Set all motors to zero power
        lfDriveMotor.setPower(0);
        rfDriveMotor.setPower(0);
        lrDriveMotor.setPower(0);
        rrDriveMotor.setPower(0);

        //----------------------------------------------------------------------------------------------
        // Servo Motors
        //----------------------------------------------------------------------------------------------
        // Define and initialize ALL installed servos.
        lfDriveServo = hardwareMap.get(Servo.class, "left_front_servo");
        rfDriveServo = hardwareMap.get(Servo.class, "right_front_servo");
        lrDriveServo = hardwareMap.get(Servo.class, "left_rear_servo");
        rrDriveServo = hardwareMap.get(Servo.class, "right_rear_servo");
        // Set to initial position
        lfDriveServo.setPosition(0.5);
        rfDriveServo.setPosition(0.5);
        lrDriveServo.setPosition(0.5);
        rrDriveServo.setPosition(0.5);

        //----------------------------------------------------------------------------------------------
        // Gyro
        //----------------------------------------------------------------------------------------------
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            float leftStickX = gamepad1.left_stick_x;
            float leftStickY = -gamepad1.left_stick_y;
            float rightStickX = gamepad1.right_stick_x;
            float rightStickY = -gamepad1.right_stick_y;
            float heading = angles.firstAngle;

            //----------------------------------------------------------------------------------------------
            // Swerve
            //----------------------------------------------------------------------------------------------
            calculateSwerve(leftStickX, leftStickY, rightStickX, rightStickY, heading);

            //----------------------------------------------------------------------------------------------
            // Servo Motors
            //----------------------------------------------------------------------------------------------
            // Display the current position of all the servos
            telemetry.addLine("Servos | ")
                    .addData("lf", df.format(lfDriveServo.getPosition()))
                    .addData("rf", df.format(rfDriveServo.getPosition()))
                    .addData("lr", df.format(lrDriveServo.getPosition()))
                    .addData("rr", df.format(rrDriveServo.getPosition()));

            //Transmit the telemetry to the driver station, subject to throttling.
            telemetry.update();

            // Update loop info
            loopCount++;
        }
    }

    private void calculateSwerve(double leftStickX, double leftStickY, double rightStickX, double rightStickY, double heading) {
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
        double RCW = rightStickX;

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
                .addData("x", df.format(rightStickX))
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
