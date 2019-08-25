package org.firstinspires.ftc.teamcode.bothardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.text.DecimalFormat;

public class SwerveBotHardware {
    private final DecimalFormat df;
    private final Telemetry telemetry;
    private final HardwareMap hwMap;

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
    public CRServo lfDriveServo = null;
    public CRServo rfDriveServo = null;
    public CRServo lrDriveServo = null;
    public CRServo rrDriveServo = null;
    //----------------------------------------------------------------------------------------------
    // Servo Motors
    //----------------------------------------------------------------------------------------------
    public AnalogInput lfEncoder = null;
    public AnalogInput rfEncoder = null;
    public AnalogInput lrEncoder = null;
    public AnalogInput rrEncoder = null;
    //----------------------------------------------------------------------------------------------
    // Servo Controller
    //----------------------------------------------------------------------------------------------
    public ServoController servoController = null;

    //----------------------------------------------------------------------------------------------
    // Gryo
    //----------------------------------------------------------------------------------------------
    BNO055IMU imu; // The IMU sensor object
    Orientation angles; // State used for updating telemetry

    /* Constructor */
    public SwerveBotHardware(DecimalFormat df, HardwareMap hardwareMap, Telemetry telemetry) {
        this.df = df;
        this.hwMap = hardwareMap;
        this.telemetry = telemetry;
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
        //----------------------------------------------------------------------------------------------
        // DC Drive Motors
        //----------------------------------------------------------------------------------------------
        // Define and Initialize Motors
        lfDriveMotor = initMotor("left_front_drive");
        rfDriveMotor = initMotor("right_front_drive");
        lrDriveMotor = initMotor("left_rear_drive");
        rrDriveMotor = initMotor("right_rear_drive");

        //----------------------------------------------------------------------------------------------
        // Servo Motors
        //----------------------------------------------------------------------------------------------
        // Define and initialize ALL installed servos.
        lfDriveServo = initServo("left_front_servo");
        rfDriveServo = initServo("right_front_servo");
        lrDriveServo = initServo("left_rear_servo");
        rrDriveServo = initServo("right_rear_servo");
        // Get Servo Controller
        servoController = lfDriveServo.getController();

        //----------------------------------------------------------------------------------------------
        // Servo Encoders
        //----------------------------------------------------------------------------------------------
        // Define and initialize ALL servo encoders.
        lfEncoder = hwMap.get(AnalogInput.class, "left_front_encoder");
        rfEncoder = hwMap.get(AnalogInput.class, "right_front_encoder");
        lrEncoder = hwMap.get(AnalogInput.class, "left_rear_encoder");
        rrEncoder = hwMap.get(AnalogInput.class, "right_rear_encoder");

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

    private CRServo initServo(String name) {
        CRServo servo = hwMap.get(CRServo.class, name);
        servo.setPower(0.0);
        servo.setDirection(DcMotor.Direction.FORWARD);

        return servo;
    }

    private DcMotor initMotor(String name) {
        DcMotor motor = hwMap.get(DcMotor.class, name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);

        return motor;
    }

    public void addAllServoTelemetry() {
        addServoTelemetry(lfDriveServo);
        addServoTelemetry(rfDriveServo);
        addServoTelemetry(lrDriveServo);
        addServoTelemetry(rrDriveServo);
    }

    private void addServoTelemetry(CRServo servo) {
        telemetry.addLine(servo.getDeviceName() + " | ")
                .addData("pwr", df.format(servo.getPower()))
                .addData("dir", servo.getDirection().toString())
                .addData("pos", df.format(servoController.getServoPosition(servo.getPortNumber())));

    }

    public void addAllEncoderTelemetry() {
        addServoTelemetry(lfDriveServo);
        addServoTelemetry(rfDriveServo);
        addServoTelemetry(lrDriveServo);
        addServoTelemetry(rrDriveServo);
    }

    private void addAnalogInputTelemetry(AnalogInput device) {
        telemetry.addLine(device.getDeviceName() + " | ")
                .addData("volt", df.format(device.getVoltage()))
                .addData("max volt", df.format(device.getMaxVoltage()));

    }
}
