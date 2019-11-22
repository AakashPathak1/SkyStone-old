package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressWarnings("WeakerAccess")
abstract class MyOpMode extends LinearOpMode {

    @SuppressWarnings("FieldCanBeLocal")
    private final boolean calibrateIMU = true;

    ElapsedTime runtime = new ElapsedTime();

    Orientation angles;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    PIDController pidRotate = new PIDController(0.0075, .00003, 0);

    DcMotor leftRear, rightRear, leftFront, rightFront, actuatorMotor, lifterLeft, lifterRight;
    Servo foundationGrabber, clamp, spin;
    BNO055IMU imu;
    ColorSensor colorSensor1, colorSensor2;

    DigitalChannel downStop;  // Hardware Device Object
    DigitalChannel upStop;  // Hardware Device Object

    /** PID stuff **/

    private Orientation lastAngles = new Orientation();
    private double globalAngle;
    private double rotation;


    @SuppressWarnings("unused")
    private static final double NEVEREST_COUNTS_PER_MOTOR_REV = 537.6;
    @SuppressWarnings("unused")
    private static final double GOBILDA_COUNTS_PER_MOTOR_REV = 36.4 * 5.2;

    @SuppressWarnings("unused")
    private static final double ACTUATOR_GEAR_REDUCTION = 3 / 12.7;

    private static final double COUNTS_PER_MOTOR_REV = GOBILDA_COUNTS_PER_MOTOR_REV;    // Neverest 20: 537.6,    Torquenado: 1440,
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference       3.93701 = mechanum wheels
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    protected void initialize() {
        // The IMU sensor object
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "IMUCalibration.json";
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


        colorSensor1 = hardwareMap.get(ColorSensor.class, "sensor_color_distance1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "sensor_color_distance2");
        downStop = hardwareMap.get(DigitalChannel.class, "downStop");
        upStop = hardwareMap.get(DigitalChannel.class, "upStop");


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
        lifterRight.setDirection(DcMotor.Direction.FORWARD);

        downStop.setMode(DigitalChannel.Mode.INPUT);
        upStop.setMode(DigitalChannel.Mode.INPUT);


        //Set to brake mode
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Mode", "Setting motors power");
        telemetry.update();

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
        spin.setPosition(0.45);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("Mode", "Done initializing");
        telemetry.update();
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
     void pidRotate(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(0.5);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                leftFront.setPower(power);
                leftRear.setPower(power);
                rightFront.setPower(-power);
                rightRear.setPower(-power);

                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                leftRear.setPower(-power);
                leftFront.setPower(-power);
                rightRear.setPower(power);
                rightFront.setPower(power);


            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                leftRear.setPower(-power);
                leftFront.setPower(-power);
                rightRear.setPower(power);
                rightFront.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        leftRear.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
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

    private double adjust = 4;

    protected void gyroTurn(double targetAngle, double speed) { //positive angle turns to the left
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
                    currentSpeed = speed;
                }
                leftFront.setPower(currentSpeed);
                rightFront.setPower(-currentSpeed);
                leftRear.setPower(currentSpeed);
                rightRear.setPower(-currentSpeed);
                headingAngle = normalizeAngle();
                telemetry.addData("heading angle: ", headingAngle);
                telemetry.update();
            }
        } else {
            targetAngle += adjust;
            if (targetAngle > 180) targetAngle -= 360;
            while (headingAngle > targetAngle + error / 2 || headingAngle < targetAngle - error / 2 && opModeIsActive()) {
                if (Math.abs(targetAngle - headingAngle) < 30) {
                    currentSpeed = .2;
                } else {
                    currentSpeed = speed;
                }
                leftFront.setPower(-currentSpeed);
                rightFront.setPower(currentSpeed);
                leftRear.setPower(-currentSpeed);
                rightRear.setPower(currentSpeed);

                headingAngle = normalizeAngle();
                telemetry.addData("heading angle: ", headingAngle);
                telemetry.update();
            }
        }
        brake();
        sleep(200);
//        telemetry.log().setCapacity(30);
//        telemetry.addLine().addData("Final value: " + normalizeAngle(), "q");
        telemetry.log().add("Final value: " + normalizeAngle());
        telemetry.update();
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

    protected void actuatorMove(double speed, int ticks, double timeout) {
        actuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target = actuatorMotor.getCurrentPosition() - ticks;

        actuatorMotor.setTargetPosition(target);


        actuatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        runtime.reset();

        actuatorMotor.setPower(speed);

        while ((opModeIsActive() && (runtime.seconds() < timeout)) && actuatorMotor.isBusy()) {
            telemetry.addData("Actuator Position", leftRear.getCurrentPosition());
            telemetry.update();
        }

        actuatorMotor.setPower(0);
        brake();
        sleep(500);

        actuatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    protected void lifterMove(double speed, int ticks, double timeout) {
        lifterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double countsPerMotorRev = 356.3;


        int targetLeft = lifterLeft.getCurrentPosition() - ticks;
        int targetRight = lifterRight.getCurrentPosition() - ticks;


        lifterLeft.setTargetPosition(-targetLeft);
        lifterRight.setTargetPosition(-targetRight);


        lifterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        runtime.reset();

        lifterLeft.setPower(speed);
        lifterRight.setPower(speed);

        while ((opModeIsActive() && (runtime.seconds() < timeout)) && lifterLeft.isBusy() && lifterRight.isBusy()) {
            telemetry.addData("Left Lifter Position", lifterLeft.getCurrentPosition());
            telemetry.addData("Right Lifter Position", lifterRight.getCurrentPosition());

            telemetry.update();
        }

        lifterLeft.setPower(0);
        lifterRight.setPower(0);

        brake();
        sleep(500);

        lifterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        double headingAngle = angles.thirdAngle;
        telemetry.update();
        return headingAngle;
    }

    private boolean shortestDirection(double angle)
    {
        return normalizeAngle() < angle;
    }

    protected void brake() {
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    public void gyroTurn(double speed, double targetAngle, double startSpeedCorrection, double timeout) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentSpeed;
        double headingAngle = angles.firstAngle;
        double error = Math.abs(targetAngle - headingAngle);
        //int counter = 0;
        double startJerk = 1;
        currentSpeed = Math.abs((error / targetAngle) * speed);

        double threshold = 0.5;

        runtime.reset();

        if (targetAngle < 0) {
            while (headingAngle >= (targetAngle + threshold) || headingAngle <= (targetAngle - threshold)) {
                if (!opModeIsActive()) {
                    return;
                }
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                headingAngle = angles.firstAngle;
                error = Math.abs(targetAngle - headingAngle);

                if (error < startSpeedCorrection) {
                    currentSpeed = Math.abs((error / targetAngle) * speed);
                }

                if (currentSpeed < 0.2) {
                    currentSpeed = 0.2;
                }

                if (headingAngle >= (targetAngle + threshold)) {
                    leftFront.setPower(-currentSpeed);
                    rightFront.setPower(currentSpeed);
                    leftRear.setPower(-currentSpeed);
                    rightRear.setPower(currentSpeed);

                }

                if (headingAngle <= (targetAngle - threshold)) {
                    leftFront.setPower(currentSpeed);
                    rightFront.setPower(-currentSpeed);
                    leftRear.setPower(currentSpeed);
                    rightRear.setPower(-currentSpeed);
                }

                if (error < startJerk) {

                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftRear.setPower(0);
                    rightRear.setPower(0);
                }

                //sleep(10);

                headingAngle = angles.firstAngle;
                //if (counter % 5 == 0){
                telemetry.addData("Heading ", headingAngle);
                //telemetry.addData("counter: ", counter);
                telemetry.update();
                //}
                //counter++;
            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            // Update telemetry & Allow time for other processes to run.
            //if(counter % 1 == 0){
            telemetry.addData("angles:", angles.firstAngle);
            telemetry.update();
            //}

        }

        if (targetAngle > 0) {

            while (headingAngle <= (targetAngle - threshold) || headingAngle >= (targetAngle + threshold)) {
                if (!opModeIsActive()) {
                    return;
                }
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                headingAngle = angles.firstAngle;
                error = Math.abs(targetAngle - headingAngle);

                if (error < startSpeedCorrection) {
                    currentSpeed = Math.abs((error / targetAngle) * speed);
                }


                if (currentSpeed < 0.1) {
                    currentSpeed = 0.1;
                }
                if (headingAngle <= -90) {
                    headingAngle += 360;
                }

                if (headingAngle <= (targetAngle - threshold)) {
                    leftFront.setPower(currentSpeed);
                    rightFront.setPower(-currentSpeed);
                    leftRear.setPower(currentSpeed);
                    rightRear.setPower(-currentSpeed);

                }

                if (headingAngle >= (targetAngle + threshold)) {
                    leftFront.setPower(-currentSpeed);
                    rightFront.setPower(currentSpeed);
                    leftRear.setPower(-currentSpeed);
                    rightRear.setPower(currentSpeed);
                }


                // sleep(50);

                if (error < startJerk) {
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftRear.setPower(0);
                    rightRear.setPower(0);
                }

                //sleep(10);


                //if (counter % 1 == 0){
                telemetry.addData("Heading ", headingAngle);
                //    telemetry.addData("counter: ", counter);
                telemetry.update();
                //}
                //counter++;

            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);


            // Update telemetry & Allow time for other processes to run.
            //if(counter % 5 == 0){
            telemetry.update();
            //}
        }

        if (targetAngle == 0.0) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            headingAngle = angles.firstAngle;
            while (headingAngle <= (targetAngle - threshold) || headingAngle >= (targetAngle + threshold)) {
                if (!opModeIsActive()) {
                    return;
                }
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                headingAngle = angles.firstAngle;
                error = targetAngle - headingAngle;

                if (error < 0) {
                    leftFront.setPower(-speed);
                    rightFront.setPower(speed);
                    leftRear.setPower(-speed);
                    rightRear.setPower(speed);
                } else if (error > 0) {
                    leftFront.setPower(speed);
                    rightFront.setPower(-speed);
                    leftRear.setPower(speed);
                    rightRear.setPower(-speed);
                }

                //if (counter % 1 == 0){
                telemetry.addData("Heading ", headingAngle);
                //telemetry.addData("counter: ", counter);
                telemetry.update();
                //}
                //counter++;
            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            telemetry.update();
        }
    }

    private void forwardDrive(double inches) {
        int targetPosition = (int) (inches * 45);
        int currentPosition = leftRear.getCurrentPosition();
        if (inches > 0) {
            rightRear.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
        } else {
            rightRear.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            leftRear.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.FORWARD);
        }
        PIDController pidDrive = new PIDController(.02, .001, 0);


        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        double power = 0.3;
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        while (opModeIsActive() && (inches < 0 ? (leftRear.getCurrentPosition() < currentPosition - targetPosition) : (leftRear.getCurrentPosition() < -currentPosition + targetPosition))) {
            // Use PID with imu input to drive in a straight line.

            double correction = pidDrive.performPID(inches < 0 ? getAngle() : -getAngle());

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 turn rotation", rotation);
            telemetry.addData("Current Position", leftRear.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition + currentPosition);


            telemetry.update();

            // set power levels.
            leftRear.setPower(power - correction);
            leftFront.setPower(power - correction);
            rightRear.setPower(power + correction);
            rightFront.setPower(power + correction);


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

    @SuppressWarnings("unused")
    private void pidSideDrive(double inches) {
        int targetPosition = (int) (inches * 45);
        int currentPosition = leftRear.getCurrentPosition();
        if (inches > 0) {
            rightRear.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            leftRear.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
        } else {
            rightRear.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            leftFront.setDirection(DcMotor.Direction.FORWARD);
        }
        PIDController pidDrive = new PIDController(.1, .002, 0);


        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        double power = 0.3;
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        while (opModeIsActive() && (inches > 0 ? (leftRear.getCurrentPosition() < currentPosition + targetPosition) : (leftRear.getCurrentPosition() < -currentPosition + targetPosition))) {
            // Use PID with imu input to drive in a straight line.

            double correction = pidDrive.performPID(inches < 0 ? getAngle() : -getAngle());

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 turn rotation", rotation);

            telemetry.update();

            // set power levels.
            leftRear.setPower(power - correction);
            leftFront.setPower(power - correction);
            rightRear.setPower(power + correction);
            rightFront.setPower(power + correction);
        }

        brake();
    }
}
