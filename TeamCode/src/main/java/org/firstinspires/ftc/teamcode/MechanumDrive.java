package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        boolean lastBState = false;
        boolean lastAState = false;
        boolean lastBState2 = false;
        boolean lastAState2 = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double r = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            double lifterPower = -gamepad2.right_stick_y;
            final double v1 = (r * Math.cos(robotAngle) + rightX) * Math.sqrt(2);
            final double v2 = (r * Math.sin(robotAngle) - rightX) * Math.sqrt(2);
            final double v3 = (r * Math.sin(robotAngle) + rightX) * Math.sqrt(2);
            final double v4 = (r * Math.cos(robotAngle) - rightX) * Math.sqrt(2);


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
                if (acceleratePower <= 0) {
                    acceleratePower = 0;
                    accelerating = false;
                }
                rightFront.setPower(-acceleratePower);
                rightRear.setPower(-acceleratePower);
                leftRear.setPower(-acceleratePower);
                leftFront.setPower(-acceleratePower);
            }

            if (gamepad1.a && !lastAState) {
                slowMode = !slowMode;
            }
            if (gamepad2.a && !lastAState2) {
                clamp.setPosition(clamp.getPosition() != 0.0 ? 0.0 : 1.0);

            }
            lastAState = gamepad1.a;
            lastAState2 = gamepad2.a;



            if (gamepad1.b && !lastBState) {
                // if foundationGrabber != 0.0, set it to 1.0, else set it to 0.0
                foundationGrabber.setPosition(foundationGrabber.getPosition() != 0.0 ? 0.0 : 1.0);
            }
            if (gamepad2.b && !lastBState2) {
                // if foundationGrabber != 0.0, set it to 1.0, else set it to 0.0
                spin.setPosition(spin.getPosition() != 0.3 ? 0.3 : 0.6);
            }
            lastBState = gamepad1.b;
            lastBState2 = gamepad2.b;

            actuatorMotor.setPower(gamepad2.left_stick_y);
            telemetry.addData("actuator motor", gamepad2.left_stick_y);

            lifterRight.setPower(lifterPower);
            lifterLeft.setPower(lifterPower);

            telemetry.addData("lifter", lifterPower);
            telemetry.update();
        }
    }

    private void composeTelemetry() {
        telemetry.addLine().addData("Slow Mode: ", () -> slowMode);
        telemetry.addLine().addData("Foundation Grabber Position: ", () -> foundationGrabber.getPosition());
        telemetry.addLine().addData("Acceleration/Deceleration Power: ", () -> Double.toString(acceleratePower));
        telemetry.addLine().addData("Clamp Servo: ", () -> Double.toString(clamp.getPosition()));
        telemetry.addLine().addData("Spin Servo: ", () -> Double.toString(spin.getPosition()));
        telemetry.addLine().addData("ActuatorMotor Encoder Value: ", () -> actuatorMotor.getCurrentPosition());
        telemetry.addLine().addData("DrawerSlides Encoder Value: ", () -> lifterLeft.getCurrentPosition());

        telemetry.update();
    }



}