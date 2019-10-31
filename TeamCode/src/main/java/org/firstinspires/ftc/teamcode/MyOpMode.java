package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressWarnings("WeakerAccess")
abstract class MyOpMode extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    Orientation angles;

    DcMotor leftRear, rightRear, leftFront, rightFront, actuatorMotor, lifterLeft, lifterRight;
    Servo foundationGrabber, clamp, spin;
    BNO055IMU imu;
    ColorSensor colorSensor;
    ColorSensor colorSensor2;

    @SuppressWarnings("unused")
    private static final double NEVEREST_COUNTS_PER_MOTOR_REV = 537.6;
    @SuppressWarnings("unused")
    private static final double GOBILDA_COUNTS_PER_MOTOR_REV = 36.4 * 5.2;

    @SuppressWarnings("unused")
    private static final double ACTUATOR_GEAR_REDUCTION = 3/12.7;

    private static final double COUNTS_PER_MOTOR_REV = GOBILDA_COUNTS_PER_MOTOR_REV;    // Neverest 20: 537.6,    Torquenado: 1440,
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference       3.93701 = mechanum wheels
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    protected void initialize() {
        colorSensor2 =  hardwareMap.get(ColorSensor.class, "colorSensor2");
        // The IMU sensor object
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // Define and Initialize Motors
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        actuatorMotor = hardwareMap.get(DcMotor.class, "actuatorMotor");
        lifterLeft = hardwareMap.get(DcMotor.class, "lifterLeft");
        lifterRight = hardwareMap.get(DcMotor.class, "lifterRight");
        leftRear.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        lifterLeft.setDirection(DcMotor.Direction.REVERSE);

        //Set to brake mode
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        actuatorMotor.setPower(0);

        foundationGrabber = hardwareMap.get(Servo.class, "foundationGrabber");
        foundationGrabber.setPosition(0.0);
        clamp = hardwareMap.get(Servo.class, "clamp");
        clamp.setPosition(1.0);
        spin = hardwareMap.get(Servo.class, "spin");
        spin.setPosition(0.3);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    private double adjust = 4;

    protected void gyroTurn(double targetAngle) { //positive angle turns to the left
        double error = 2;
        double currentSpeed;
        double headingAngle = normalizeAngle();
        adjust += Math.abs(targetAngle - headingAngle) * .012;
        if (shortestDirection(targetAngle)) {
            targetAngle -= adjust;
            if (targetAngle < -180) targetAngle += 360;
            while (headingAngle > targetAngle + error / 2 || headingAngle < targetAngle - error / 2 && opModeIsActive()) {
                if (Math.abs(targetAngle - headingAngle) < 30) {
                    currentSpeed = .2;
                } else {
                    currentSpeed = 1;
                }
                leftFront.setPower(currentSpeed);
                rightFront.setPower(-currentSpeed);
                leftRear.setPower(currentSpeed);
                rightRear.setPower(-currentSpeed);
                headingAngle = normalizeAngle();

            }
        } else {
            targetAngle += adjust;
            if (targetAngle > 180) targetAngle -= 360;
            while (headingAngle > targetAngle + error / 2 || headingAngle < targetAngle - error / 2 && opModeIsActive()) {
                if (Math.abs(targetAngle - headingAngle) < 30) {
                    currentSpeed = .2;
                } else {
                    currentSpeed = 1;
                }
                leftFront.setPower(-currentSpeed);
                rightFront.setPower(currentSpeed);
                leftRear.setPower(-currentSpeed);
                rightRear.setPower(currentSpeed);

                headingAngle = normalizeAngle();

            }
        }
        brake();
        sleep(200);
        normalizeAngle();
    }

    protected void encoderDrive(double speed, double inches, double timeout) {
        int target;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        target = leftRear.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setTargetPosition(target);
        leftRear.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        rightRear.setTargetPosition(target);

        runtime.reset();

        leftRear.setPower(speed);
        leftFront.setPower(speed);
        rightRear.setPower(speed);
        rightFront.setPower(speed);

        while ((opModeIsActive() && (runtime.seconds() < timeout)) && leftRear.isBusy()) {
            telemetry.addData("Left Rear Current Position", leftRear.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        brake();
        sleep(1000);


        // Turn off RUN_TO_POSITION
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void sideDrive(double speed, double inches, double timeout) { //positive inches to go left
        int target;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        target = leftRear.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setTargetPosition(-target); //negative because we want side drive
        leftRear.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        rightRear.setTargetPosition(-target);  //negative because we want side drive

        runtime.reset();

        leftRear.setPower(speed);
        leftFront.setPower(speed);
        rightRear.setPower(speed);
        rightFront.setPower(speed);

        while ((opModeIsActive() && (runtime.seconds() < timeout)) && leftRear.isBusy()) {
            telemetry.addData("Left Rear Current Position", leftRear.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        brake();
        sleep(1000);


        // Turn off RUN_TO_POSITION
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void actuatorMove(double speed, double inches, double timeout) {
        actuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target = actuatorMotor.getCurrentPosition() - (int) (inches * (COUNTS_PER_MOTOR_REV * ACTUATOR_GEAR_REDUCTION));

        actuatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        actuatorMotor.setTargetPosition(target);

        runtime.reset();

        actuatorMotor.setPower(speed);

        while ((opModeIsActive() && (runtime.seconds() < timeout)) && actuatorMotor.isBusy()) {
            telemetry.addData("Actuator Position", leftRear.getCurrentPosition());
            telemetry.update();
        }

        brake();
        sleep(500);

//        actuatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private static boolean inRange(int lower, int higher, int val) {
        return val > lower && val < higher;
    }

    protected void gyroDrive(double speed, double inches) {

        double encoderCount = inches * COUNTS_PER_INCH;
        double startPosition = leftRear.getCurrentPosition();

        runtime.reset();

        if (encoderCount > 0) {

            while (leftRear.getCurrentPosition() < (encoderCount + startPosition)) {
                if (!opModeIsActive()) {
                    return;
                }

                leftFront.setPower(-speed); //negative because random problem with TMK chassis
                rightFront.setPower(-speed); //negative because random problem with TMK chassis
                leftRear.setPower(speed);
                rightRear.setPower(speed);

                telemetry.addData("encoder value", leftRear.getCurrentPosition());
                telemetry.addData("inches travelled", Math.round((leftRear.getCurrentPosition() - startPosition) / COUNTS_PER_INCH));
                telemetry.update();
                sleep(50);
            }
        } else if (encoderCount < 0) {

            while (leftRear.getCurrentPosition() > (encoderCount + startPosition)) {
                if (!opModeIsActive()) {
                    return;
                }

                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftRear.setPower(-speed);
                rightRear.setPower(-speed);

                telemetry.addData("encoder value", leftRear.getCurrentPosition());
                telemetry.update();
                sleep(50);
            }
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);


        sleep(50);
    }

    private double normalizeAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double headingAngle = angles.firstAngle;
        telemetry.update();
        return headingAngle;
    }

    private boolean shortestDirection(double angle) {
        return normalizeAngle() < angle;
    }

    protected void brake() {
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }
}
