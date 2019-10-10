package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "MechanumDrive")
public class MechanumDrive extends MyOpMode {

    private boolean slowMode = false;
    private double acceleratePower = 0.0;

    @Override
    public void runOpMode() {
        initialize();
        // Send telemetry message to signify robot waiting;
        boolean accelerating = false;
        composeTelemetry();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double r = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if (gamepad1.y) {
                accelerating = false;
                acceleratePower = 0.0;
            }

            if (slowMode && !accelerating) {
                leftFront.setPower(v1 / 2);
                rightFront.setPower(v2 / 2);
                leftRear.setPower(v3 / 2);
                rightRear.setPower(v4 / 2);
            } else if (!accelerating) {
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftRear.setPower(v3);
                rightRear.setPower(v4);
            }

            if (gamepad1.x) {
                accelerating = true;
                if (acceleratePower < 0.5) {
                    acceleratePower += 0.001;
                }
                rightFront.setPower(-acceleratePower);
                rightRear.setPower(-acceleratePower);
                leftRear.setPower(-acceleratePower);
                leftFront.setPower(-acceleratePower);
            }

            if (acceleratePower > 0 && !gamepad1.x) {
                acceleratePower -= 0.001;
                if (acceleratePower < 0) {
                    acceleratePower = 0;
                    accelerating = false;
                }
                rightFront.setPower(-acceleratePower);
                rightRear.setPower(-acceleratePower);
                leftRear.setPower(-acceleratePower);
                leftFront.setPower(-acceleratePower);
            }

            slowMode = gamepad1.a || slowMode; // enable slow mode
            slowMode = !gamepad1.b && slowMode; // disable slow mode "1_1/2"
        }
    }

    private void composeTelemetry() {
        telemetry.addLine().addData("Slow Mode: ", () -> slowMode ? "true" : "false");
        telemetry.addLine().addData("Acceleration/Deceleration Power: ", () -> Double.toString(acceleratePower));
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
    }


}