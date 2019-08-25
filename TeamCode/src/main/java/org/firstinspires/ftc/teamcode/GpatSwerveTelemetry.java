package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bothardware.SwerveBotHardware;

import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.Collections;

@TeleOp(name = "Gpat Swerve", group = "Gpat")
public class GpatSwerveTelemetry extends LinearOpMode {
    public double LF_ENCODER_OFFSET = 0.238;
    public double RF_ENCODER_OFFSET = 0.0371;
    public double LR_ENCODER_OFFSET = 1.052;
    public double RR_ENCODER_OFFSET = 0.241;
    double lfEncoderOffset = LF_ENCODER_OFFSET;
    double rfEncoderOffset = LF_ENCODER_OFFSET;
    double lrEncoderOffset = LF_ENCODER_OFFSET;
    double rrEncoderOffset = LF_ENCODER_OFFSET;
    private DecimalFormat df = new DecimalFormat("#.##");
    private SwerveBotHardware swerveBotHardware;

    @Override
    public void runOpMode() {
        swerveBotHardware = new SwerveBotHardware(df, hardwareMap, telemetry);
        swerveBotHardware.init();


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

        lfEncoderOffset = swerveBotHardware.lfEncoder.getVoltage();
        rfEncoderOffset = swerveBotHardware.rfEncoder.getVoltage();
        lrEncoderOffset = swerveBotHardware.lrEncoder.getVoltage();
        rrEncoderOffset = swerveBotHardware.rrEncoder.getVoltage();

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

            swerveBotHardware.addAllServoTelemetry();
            swerveBotHardware.addAllEncoderTelemetry();

            //Transmit the telemetry to the driver station, subject to throttling.
            telemetry.update();

            // Update loop info
            loopCount++;
        }
    }

    private void calculateSwerve(double leftStickX, double leftStickY, double RCW, double rightStickY, double heading) {
        double BASE_LENGTH = 15.5;
        double BASE_WIDTH = 14.5;
        double baseRadius = Math.hypot(BASE_LENGTH, BASE_WIDTH);

        heading = 360 - ((heading + 360) % 360);
        double headingRadians = Math.toRadians(heading);
        // Adjust robot to field rotation to ensure movement is always from the drivers perpective
        double FWD = (leftStickY * Math.cos(headingRadians)) + (leftStickX * Math.sin(headingRadians));
        double STR = (-leftStickY * Math.sin(headingRadians)) + (leftStickX * Math.cos(headingRadians));

        double aCoefficient = STR - (RCW * (BASE_LENGTH / baseRadius));
        double bCoefficient = STR + (RCW * (BASE_LENGTH / baseRadius));
        double cCoefficient = FWD - (RCW * (BASE_WIDTH / baseRadius));
        double dCoefficient = FWD + (RCW * (BASE_WIDTH / baseRadius));

        double lfWheelSpeed = Math.hypot(bCoefficient, dCoefficient);
        double rfWheelSpeed = Math.hypot(bCoefficient, cCoefficient);
        double lrWheelSpeed = Math.hypot(aCoefficient, dCoefficient);
        double rrWheelSpeed = Math.hypot(aCoefficient, cCoefficient);

        double maxWheelSpeed = Collections.max(Arrays.asList(lfWheelSpeed, rfWheelSpeed, lrWheelSpeed, rrWheelSpeed));

        if (maxWheelSpeed > 1) {
            lfWheelSpeed = lfWheelSpeed / maxWheelSpeed;
            rfWheelSpeed = rfWheelSpeed / maxWheelSpeed;
            lrWheelSpeed = lrWheelSpeed / maxWheelSpeed;
            rrWheelSpeed = rrWheelSpeed / maxWheelSpeed;
        }

        double rfWheelAngle = -1;
        double lfWheelAngle = -1;
        double rrWheelAngle = -1;
        double lrWheelAngle = -1;

        if (bCoefficient != 0 || cCoefficient != 0) {
            double rfRad = Math.atan2(bCoefficient, cCoefficient);
            double rfDeg = rfRad * 180 / Math.PI;
            double rawAbs = (rfDeg + 360) % 360;
            rfWheelAngle = rawAbs;
        }

        if (bCoefficient != 0 || dCoefficient != 0) {
            lfWheelAngle = ((Math.atan2(bCoefficient, dCoefficient) * 180 / Math.PI) + 360) % 360;
        }


        if (aCoefficient != 0 || dCoefficient != 0) {
            lrWheelAngle = ((Math.atan2(aCoefficient, dCoefficient) * 180 / Math.PI) + 360) % 360;
        }

        if (aCoefficient != 0 || cCoefficient != 0) {
            rrWheelAngle = ((Math.atan2(aCoefficient, cCoefficient) * 180 / Math.PI) + 360) % 360;
        }

        boolean lfWheelInPosition = updateServo(swerveBotHardware.lfDriveMotor, swerveBotHardware.lfDriveServo, swerveBotHardware.lfEncoder, lfWheelAngle, lfEncoderOffset);
        boolean rfWheelInPosition = updateServo(swerveBotHardware.rfDriveMotor, swerveBotHardware.rfDriveServo, swerveBotHardware.rfEncoder, rfWheelAngle, rfEncoderOffset);
        boolean lrWheelInPosition = updateServo(swerveBotHardware.lrDriveMotor, swerveBotHardware.lrDriveServo, swerveBotHardware.lrEncoder, lrWheelAngle, lrEncoderOffset);
        boolean rrWheelInPosition = updateServo(swerveBotHardware.rrDriveMotor, swerveBotHardware.rrDriveServo, swerveBotHardware.rrEncoder, rrWheelAngle, rrEncoderOffset);

        if (gamepad1.right_trigger > 0.2) {
            swerveBotHardware.lfDriveMotor.setPower(lfWheelSpeed);
            swerveBotHardware.rfDriveMotor.setPower(rfWheelSpeed);
            swerveBotHardware.lrDriveMotor.setPower(lrWheelSpeed);
            swerveBotHardware.rrDriveMotor.setPower(rrWheelSpeed);
        } else {
            swerveBotHardware.lfDriveMotor.setPower(lfWheelSpeed / 3);
            swerveBotHardware.rfDriveMotor.setPower(rfWheelSpeed / 3);
            swerveBotHardware.lrDriveMotor.setPower(lrWheelSpeed / 3);
            swerveBotHardware.rrDriveMotor.setPower(rrWheelSpeed / 3);
        }

//        if(lfWheelInPosition && rfWheelInPosition && lrWheelInPosition && rrWheelInPosition) {
//        }

        telemetry.addLine("heading | ").addData("d", df.format(heading));
        telemetry.addLine("Trig | ")
                .addData("rad", df.format(headingRadians))
                .addData("cos", df.format(Math.cos(headingRadians)))
                .addData("sin", df.format(Math.sin(headingRadians)));
        telemetry.addLine("left joystick | ")
                .addData("x", df.format(leftStickX))
                .addData("y", df.format(leftStickY));
        telemetry.addLine("right joystick | ")
                .addData("x", df.format(RCW))
                .addData("y", df.format(rightStickY));
        telemetry.addLine("gc | ")
                .addData("rt", df.format(gamepad1.right_trigger))
                .addData("rb", gamepad1.right_bumper)
                .addData("x", gamepad1.x)
                .addData("y", gamepad1.y)
                .addData("a", gamepad1.a)
                .addData("b", gamepad1.b);
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
        telemetry.addLine("LF | ")
                .addData("ws", df.format(lfWheelSpeed))
                .addData("wa", df.format(lfWheelAngle));
        telemetry.addLine("RF | ")
                .addData("ws", df.format(rfWheelSpeed))
                .addData("wa", df.format(rfWheelAngle));
        telemetry.addLine("LR | ")
                .addData("ws", df.format(lrWheelSpeed))
                .addData("wa", df.format(lrWheelAngle));
        telemetry.addLine("RR | ")
                .addData("ws", df.format(rrWheelSpeed))
                .addData("wa", df.format(rrWheelAngle));
        telemetry.addLine("LfEn | ")
                .addData("off", df.format(lfEncoderOffset))
                .addData("ep", df.format(getAngleFromEncoder(swerveBotHardware.lfEncoder, lfEncoderOffset)))
                .addData("ev", df.format(swerveBotHardware.lfEncoder.getVoltage()))
                .addData("mev", df.format(swerveBotHardware.lfEncoder.getMaxVoltage()))
                .addData("inP", lfWheelInPosition);
        telemetry.addLine("RfEn | ")
                .addData("off", df.format(rfEncoderOffset))
                .addData("ep", df.format(getAngleFromEncoder(swerveBotHardware.rfEncoder, rfEncoderOffset)))
                .addData("ev", df.format(swerveBotHardware.rfEncoder.getVoltage()))
                .addData("mev", df.format(swerveBotHardware.rfEncoder.getMaxVoltage()))
                .addData("inP", rfWheelInPosition);
        telemetry.addLine("LrEn | ")
                .addData("off", df.format(lrEncoderOffset))
                .addData("ep", df.format(getAngleFromEncoder(swerveBotHardware.lrEncoder, lrEncoderOffset)))
                .addData("ev", df.format(swerveBotHardware.lrEncoder.getVoltage()))
                .addData("mev", df.format(swerveBotHardware.lrEncoder.getMaxVoltage()))
                .addData("inP", lrWheelInPosition);
        telemetry.addLine("RrEn | ")
                .addData("off", df.format(rrEncoderOffset))
                .addData("ep", df.format(getAngleFromEncoder(swerveBotHardware.rrEncoder, rrEncoderOffset)))
                .addData("ev", df.format(swerveBotHardware.rrEncoder.getVoltage()))
                .addData("mev", df.format(swerveBotHardware.rrEncoder.getMaxVoltage()))
                .addData("inP", rrWheelInPosition);
    }

    private boolean updateServo(DcMotor motor, CRServo servo, AnalogInput encoder, double wheelAngle, double encoderOffset) {
        double encoderAngle = getAngleFromEncoder(encoder, encoderOffset);
        boolean wheelInPosition = false;

        if (wheelAngle < 0) {
            wheelAngle = encoderAngle;
        }

        double rawAngularTravel = wheelAngle - encoderAngle;
        double angularTravel = (rawAngularTravel + 360) % 360;

        if (angularTravel <= 90) {
            // Clockwise to target
            servo.setDirection(DcMotor.Direction.FORWARD);
            motor.setDirection(DcMotor.Direction.FORWARD);
        } else if (angularTravel <= 180) {
            angularTravel = 180 - angularTravel;
            servo.setDirection(DcMotor.Direction.REVERSE);
            motor.setDirection(DcMotor.Direction.REVERSE);
        } else if (angularTravel <= 270) {
            angularTravel = angularTravel - 180;
            servo.setDirection(DcMotor.Direction.FORWARD);
            motor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            angularTravel = 360 - angularTravel;
            servo.setDirection(DcMotor.Direction.REVERSE);
            motor.setDirection(DcMotor.Direction.FORWARD);
        }

        if (angularTravel > 10) {
            if (angularTravel <= 20) {
                servo.setPower(angularTravel / 40);
            } else {
                servo.setPower(1);
            }
        } else {
            servo.setPower(0);
            wheelInPosition = true;
        }

        return wheelInPosition;
    }

    private double getAngleFromEncoder(AnalogInput encoder, double voltageOffset) {
        double rawVoltage = encoder.getVoltage();
        double maxVoltage = encoder.getMaxVoltage();
        double voltage = ((rawVoltage - voltageOffset) + maxVoltage) % maxVoltage;

        double initEncoderAngle = ((voltage * 360) / maxVoltage);
        double encoderAngle = 360 - initEncoderAngle;
        return encoderAngle;
    }
}