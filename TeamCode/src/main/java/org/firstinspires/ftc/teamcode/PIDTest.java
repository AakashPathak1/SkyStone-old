// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses PID controller to drive in a straight line when not
// avoiding an obstacle.
//
// Use PID controller to manage motor power during 90 degree turn to reduce
// overshoot.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "PIDTest", group = "Exercises")
//@Disabled
public class PIDTest extends MyOpMode {
    private Orientation lastAngles = new Orientation();
    private double globalAngle;
    private double rotation;
    private PIDController pidRotate = new PIDController(0.0075, .00003, 0);

    @Override
    public void runOpMode() {
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to REV Touch sensor.

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.


        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);
        forwardDrive(50);


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

}