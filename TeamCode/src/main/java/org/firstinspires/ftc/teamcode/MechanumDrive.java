package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MechanumDrive")
public class MechanumDrive extends MyOpMode {

    private boolean slowMode = false;
    private boolean lifterActuatorSlowMode = false;
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
        boolean lastXState2 = false;
        boolean lastRightBumperState2 = false;


        // run until the end of the match (driver presses STOP)
        long currentNanos2 = 0;
        while (opModeIsActive()) {
            telemetry.addData("time it took", (currentNanos2 - System.nanoTime()) / 1_000_000);
            telemetry.update();
            long currentNanos = System.nanoTime();
            double r = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x / 1.5; //divide by a number to make it slower
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
                    acceleratePower += 0.005;
                }
                rightFront.setPower(-acceleratePower);
                rightRear.setPower(-acceleratePower);
                leftRear.setPower(-acceleratePower);
                leftFront.setPower(-acceleratePower);
            }

            if (acceleratePower > 0 && !gamepad1.x) {
                acceleratePower -= 0.005;
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

            if (gamepad2.x && !lastXState2) {
                lifterActuatorSlowMode = !lifterActuatorSlowMode;
            }
            lastXState2 = gamepad2.x;


            if (gamepad1.b && !lastBState) {
                // if foundationGrabber != 0.0, set it to 1.0, else set it to 0.0
                foundationGrabber.setPosition(foundationGrabber.getPosition() != 0.0 ? 0.0 : 1.0);
            }
            if (gamepad2.b && !lastBState2) {
                // if foundationGrabber != 0.0, set it to 1.0, else set it to 0.0
                spin.setPosition((spin.getPosition() > 0.63) || (spin.getPosition() < 0.61) ? 0.62 : 0.94);
            }
            lastBState = gamepad1.b;
            lastBState2 = gamepad2.b;


            if (lifterActuatorSlowMode) {
                actuatorMotor.setPower(gamepad2.left_stick_y / 2); //actuator motor
            } else {
                actuatorMotor.setPower(gamepad2.left_stick_y); //actuator motor
            }

//            if (gamepad2.right_bumper && !lastRightBumperState2) { // doesn't work jey
//                lifterMove(1.0, 100, 5);
//                telemetry.addLine("right bumper ran");
//            }
//            lastRightBumperState2 = gamepad2.right_bumper;

            if (downStop.getState() && upStop.getState()) { //is not pressed
                if (lifterPower == 0) {
                    lifterPower = 0.15;
                }
                if (lifterActuatorSlowMode) {
                    lifterRight.setPower(lifterPower / 2);
                    lifterLeft.setPower(lifterPower / 2);
                } else {
                    lifterRight.setPower(lifterPower);
                    lifterLeft.setPower(lifterPower);
                }
            } else if (!downStop.getState()) { //is pressed
                lifterPower = lifterPower < 0 ? 0 : lifterPower; // if lifterPower < 0 set it to 0

                if (lifterActuatorSlowMode) {
                    lifterRight.setPower(lifterPower / 2);
                    lifterLeft.setPower(lifterPower / 2);
                } else {
                    lifterRight.setPower(lifterPower);
                    lifterLeft.setPower(lifterPower);
                }
            }
//            else if (!upStop.getState()) {
//                lifterPower = lifterPower >= 0 ? 0.2 : lifterPower; // if lifterPower >= 0 set it to 0
//
//                lifterRight.setPower(lifterPower);
//                lifterLeft.setPower(lifterPower);
//            }


            if (gamepad2.y) {
                lifterLeft.setPower(-0.8);
                lifterRight.setPower(-0.8);
                while (downStop.getState() && opModeIsActive()) { //is not pressed
                }
                lifterLeft.setPower(0);
                lifterRight.setPower(0);
            }

            telemetry.addData("lifter power: ", lifterPower);
            telemetry.addData("time it took", (currentNanos - System.nanoTime()) / 1_000_000);
            telemetry.update();
            currentNanos2 = System.nanoTime();
        }
    }

    private void composeTelemetry() {
        telemetry.addLine().addData("Slow Mode: ", () -> slowMode);
        telemetry.addLine().addData("Second Slow Mode: ", () -> lifterActuatorSlowMode);
        telemetry.addLine().addData("Foundation Grabber Position: ", () -> foundationGrabber.getPosition());
        telemetry.addLine().addData("Acceleration/Deceleration Power: ", () -> Double.toString(acceleratePower));
        telemetry.addLine().addData("Clamp Servo: ", () -> Double.toString(clamp.getPosition()));
        telemetry.addLine().addData("Spin Servo: ", () -> Double.toString(spin.getPosition()));
        telemetry.addLine().addData("ActuatorMotor Encoder Value: ", () -> actuatorMotor.getCurrentPosition());
        telemetry.addLine().addData("DrawerSlides Encoder Value: ", () -> lifterLeft.getCurrentPosition());
        telemetry.addLine().addData("Down Touch Sensor pressed: ", () -> !downStop.getState());
        telemetry.addLine().addData("Up Touch Sensor pressed: ", () -> !upStop.getState());
        telemetry.addLine().addData("Actuator motor power", () -> gamepad2.left_stick_y);


        telemetry.update();
    }

}
