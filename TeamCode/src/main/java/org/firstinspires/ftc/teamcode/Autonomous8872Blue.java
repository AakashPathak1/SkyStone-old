package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

@Autonomous(name = "Autonomous 8872")
public class Autonomous8872Blue extends MyOpMode {

    @Override
    public void runOpMode() {
        initialize(true);
        colorTelemetry();
        telemetry.addData("Staus", "waiting for start");
        telemetry.update();
        waitForStart();
        telemetry.addData("Staus", "running");
        telemetry.update();

        spin.setPosition(0.62); //spin outside
        // sleep(500);
        clamp.setPosition(0.0); //clamp open
        // sleep(500);

        encoderDrive(0.3, 34.5, 10);
        // sleep(500);

        int skystone = goToSkyStone();
        pickUpStone();

        forwardDrive(-7); //back up from stones
        // sleep(200);

        int sideDriveAmt = 90;
        if (skystone == 4) {
            sideDriveAmt += 30;
        } else if (skystone == 6) {
            sideDriveAmt -= 10;
        }


        sideDrive(1.0, sideDriveAmt, 10); //side drive to building zone

        lifterMove(0.6, 130, 5); //move lifter up
        lifterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifterRight.setPower(0.16);
        lifterLeft.setPower(0.16);
        sleep(200);

        sideDrive(0.18, +100, 10); //side drive slower to not lose block

        sideDrive(0.3, -7, 10); //move in front of foundation

        encoderDrive(0.2, 12, 10); //drive toward foundation to place
        // sleep(500);

        clamp.setPosition(0.0); //open clamp
        sleep(1000);

        forwardDrive(-5);// back up from foundation
        // sleep(200);

        pidRotate(90, 1); //turn around
        pidRotate(90, 1);
        sleep(200);


        forwardDrive(-9); //drive to foundation
        sleep(200);

        foundationGrabber.setPosition(1.0);
        sleep(1000);

        leftRear.setPower(-0.1);
        leftFront.setPower(-0.1);
        rightRear.setPower(-1.0);
        rightFront.setPower(-1.0);

        sleep(2000); //change time here

        leftRear.setPower(0);
        leftFront.setPower(0);

        rightRear.setPower(0);
        rightFront.setPower(0);
        sleep(1000);

        leftRear.setPower(0.8);
        leftFront.setPower(0.8);

        rightRear.setPower(0.3);
        rightFront.setPower(0.3);

        sleep(1500); //change time here

        leftRear.setPower(0);
        leftFront.setPower(0);

        rightRear.setPower(0);
        rightFront.setPower(0);
        sleep(1000);

        foundationGrabber.setPosition(0.0);


        dropLifter();

        clamp.setPosition(1.0);//these are initialize positions
        spin.setPosition(0.45);

        actuatorMove(1, -454, 5); // bring actuator back

        sideDrive(0.4, 14, 10); // side drive away from wall


        forwardDrive(90); //forward drive to loading zone

        pidRotate(90, 1); // turn to face stones

        sideDrive(0.4, 50, 10); // side drive into wall

        encoderDrive(0.4, -20, 10); // drive into wall

        spin.setPosition(0.62); //spin outside
        // sleep(500);
        clamp.setPosition(0.0); //clamp open
        // sleep(500);

        encoderDrive(0.3, 34.5, 10);
        // sleep(500);

        skystone = goToSkyStone();
        pickUpStone();

        forwardDrive(-7); //back up from stones
        // sleep(200);

        sideDriveAmt = +150; //this is 2nd one
        if (skystone == 4) {
            sideDriveAmt += 30;
        } else if (skystone == 6) {
            sideDriveAmt -= 10;
        }

        pidRotate(-90, 1);

        encoderDrive(0.4, sideDriveAmt, 10);

        lifterMove(0.6, 130, 5); //move lifter up
        lifterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifterRight.setPower(0.16);
        lifterLeft.setPower(0.16);
        sleep(200);

        encoderDrive(0.18, -40, 10); //drive slower to not lose block





        clamp.setPosition(0.0); //open clamp
        sleep(1000);

        encoderDrive(0.4, 10, 10);// back up from foundation
        // sleep(200);


        dropLifter();

        encoderDrive(0.4, 50, 10); //drive onto tape








        telemetry.addData("Staus", "done");
        telemetry.update();
    }

    private int goToSkyStone() {
        double value1 = shaunakvalue1();
        double value2 = shaunakvalue2();
        // if(Math.abs(value1 - value2) < .06){ //both stones are the same
        //     //skystone = 6th
        //     telemetry.addData("position", 6);
        //     telemetry.update();
        //     sideDrive(0.1, 15.5, 5);
        // } else if(value1 < value2) {
        // //skystone = 4th
        //     telemetry.addData("position", 4);
        //     telemetry.update();
        //     sideDrive(0.1, -13.7, 5);
        // } else {
        //     // skystone = 5th
        //     telemetry.addData("position", 5);
        //     telemetry.update();
        //     sideDrive(0.1, 1, 5);
        // }

        int skystonePosition;
        if (shaunakvalue2() < 0.68) {
            skystonePosition = 4;
            telemetry.addData("position", 4);
            telemetry.update();
            sideDrive(0.1, 13.5, 5);
        } else if (shaunakvalue1() < 0.68) {
            skystonePosition = 5;
            telemetry.addData("position", 5);
            telemetry.update();
            sideDrive(0.1, -1.3, 5);
        } else {
            skystonePosition = 6;
            telemetry.addData("position", 6);
            telemetry.update();
            sideDrive(0.1, -13.75, 5);
        }
        return skystonePosition;
    }

    private void pickUpStone() {
        actuatorMove(1, 457, 5); //move actuator to stones
        sleep(200);

        clamp.setPosition(1.0); //close clamp
        sleep(1000);

        // make strings taut
        while (!downStop.getState() && opModeIsActive()) {
            lifterLeft.setPower(0.5);
            lifterRight.setPower(0.5);
        }


        lifterLeft.setPower(0.16);
        lifterRight.setPower(0.16);


        lifterMove(0.6, 30, 5); //move lifter up
        lifterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifterLeft.setPower(0.16);
        lifterRight.setPower(0.16);
    }

    private double shaunakvalue1() {
        return (1.0d * colorSensor1.red() + colorSensor1.green()) / (1.0d * colorSensor1.red() + colorSensor1.green() + colorSensor1.blue());
    }

    private double shaunakvalue2() {
        return (1.0d * colorSensor2.red() + colorSensor2.green()) / (1.0d * colorSensor2.red() + colorSensor2.green() + colorSensor2.blue());
    }

    private void colorTelemetry() {
        telemetry
                .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addData("Alpha ", colorSensor1.alpha());
        telemetry.addData("Red  ", colorSensor1.red());
        telemetry.addData("Green ", colorSensor1.green());
        telemetry.addData("Blue ", colorSensor1.blue());
        telemetry.addData("Shaunak value ", 1.0 * (colorSensor1.red() + colorSensor1.green()) / (colorSensor1.red() + colorSensor1.green() + colorSensor1.blue()));

        telemetry.update();
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


}

