package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressWarnings("WeakerAccess")
abstract class MechanumOpMode extends LinearOpMode {
    private Orientation lastAngles = new Orientation();
    private double globalAngle;
    private double rotation;
    private PIDController pidRotate = new PIDController(.007, .00005, 0);

    @SuppressWarnings("FieldCanBeLocal")
    private final boolean calibrateIMU = false;

    ElapsedTime runtime = new ElapsedTime();

    Orientation angles;

    DcMotor leftRear, rightRear, leftFront, rightFront;
    BNO055IMU imu;
    PIDController pidDrive;

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

    protected void initializeMechanum() {
        // The IMU sensor object
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (calibrateIMU && !isStopRequested() && !imu.isGyroCalibrated()) {
            telemetry.addData("Mode", "Calibrating imu...");
            telemetry.update();
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "Initializing hardware devices");
        telemetry.update();


        // PID Configuration
        pidDrive = new PIDController(.03, 0, 0);

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);


        // Define and Initialize Motors
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");



        //Set to brake mode
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Mode", "Setting motors power");
        telemetry.update();

        // Set all motors to zero power
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    private double adjust = 4;
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("1 imu heading", angles.firstAngle);
        telemetry.update();
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    protected void PIDEncoderDrive(double inches, double powerF, double timeout, String direction){
        // Set up parameters for driving in a straight line.
        if(direction.equals("right")){
            rightRear.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            leftFront.setDirection(DcMotor.Direction.FORWARD);
        }
        if(direction.equals("left")){
            rightRear.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            leftRear.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
        }
        if(direction.equals("backward")){
            rightRear.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
        }
        if(direction.equals("forward")){
            rightRear.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            leftRear.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.FORWARD);
        }

//        rightRear.setDirection(DcMotor.Direction.FORWARD);
//        rightFront.setDirection(DcMotor.Direction.FORWARD);
//        leftRear.setDirection(DcMotor.Direction.REVERSE);
//        leftFront.setDirection(DcMotor.Direction.REVERSE);
        int currentPos = leftRear.getCurrentPosition();
        int targetPos = currentPos - (int)(inches * COUNTS_PER_INCH);

        pidDrive.setSetpoint(0);
        double power = powerF;
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-180, 180);
        pidDrive.enable();
        runtime.reset();

        telemetry.addData("Mode", "Initalized Motors and PID");
        while (opModeIsActive() && (leftRear.getCurrentPosition() >= targetPos) && (runtime.seconds() < timeout)) {
            // Use PID with imu input to drive in a straight line.
            double correction = pidDrive.performPID(getAngle());

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 turn rotation", rotation);
            telemetry.addData("Current Position: " , leftRear.getCurrentPosition());
            telemetry.addData("Target Position: " , targetPos);
            telemetry.update();

            telemetry.update();

            // set power levels.
            if(direction.equals("right")){
                leftRear.setPower(power - correction);
                leftFront.setPower(power - correction);
                rightRear.setPower(power + correction);
                rightFront.setPower(power + correction);
            }
            if(direction.equals("left")){
                leftRear.setPower(power + correction);
                leftFront.setPower(power + correction);
                rightRear.setPower(power - correction);
                rightFront.setPower(power - correction);
            }
            if(direction.equals("forward")){
                leftRear.setPower(power - correction);
                leftFront.setPower(power + correction);
                rightRear.setPower(power + correction);
                rightFront.setPower(power - correction);
            }
            if(direction.equals("backward")){
                leftRear.setPower(power - correction);
                leftFront.setPower(power + correction);
                rightRear.setPower(power + correction);
                rightFront.setPower(power - correction);
            }

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.
        }

        // turn the motors off.
        leftRear.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }
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
