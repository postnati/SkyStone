package org.firstinspires.ftc.teamcode.bothardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.text.DecimalFormat;

public class SwerveBotHardware {
    private final DecimalFormat df;
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
    HardwareMap hwMap = null;
    //----------------------------------------------------------------------------------------------
    // Gryo
    //----------------------------------------------------------------------------------------------
    BNO055IMU imu; // The IMU sensor object
    Orientation angles; // State used for updating telemetry

    /* Constructor */
    public SwerveBotHardware(DecimalFormat df) {
        this.df = df;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //----------------------------------------------------------------------------------------------
        // DC Drive Motors
        //----------------------------------------------------------------------------------------------
        // Define and Initialize Motors
        lfDriveMotor = hwMap.get(DcMotor.class, "left_front_drive");
        rfDriveMotor = hwMap.get(DcMotor.class, "right_front_drive");
        lrDriveMotor = hwMap.get(DcMotor.class, "left_rear_drive");
        rrDriveMotor = hwMap.get(DcMotor.class, "right_rear_drive");
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
        lfDriveServo = hwMap.get(Servo.class, "left_front_servo");
        rfDriveServo = hwMap.get(Servo.class, "right_front_servo");
        lrDriveServo = hwMap.get(Servo.class, "left_rear_servo");
        rrDriveServo = hwMap.get(Servo.class, "right_rear_servo");
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
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void addServoTelemetry(Telemetry telemetry) {
        //----------------------------------------------------------------------------------------------
        // Servo Motors
        //----------------------------------------------------------------------------------------------
        // Display the current position of all the servos
        telemetry.addLine("Servos | ")
                .addData("lf", df.format(lfDriveServo.getPosition()))
                .addData("rf", df.format(rfDriveServo.getPosition()))
                .addData("lr", df.format(lrDriveServo.getPosition()))
                .addData("rr", df.format(rrDriveServo.getPosition()));

    }
}
