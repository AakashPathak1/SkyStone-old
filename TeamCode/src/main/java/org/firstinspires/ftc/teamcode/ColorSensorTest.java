package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


@TeleOp(name = "Color Sensor Test", group = "Sensor")
public class ColorSensorTest extends LinearOpMode {

    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        waitForStart();
        composeTelemetry();

        while (opModeIsActive()) {
            telemetry.update();
        }

    }

    private void composeTelemetry() {
        telemetry.addLine().addData("Distance (cm)", () -> String.format(Locale.US, "%.02f", distanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addLine().addData("Alpha ", () -> colorSensor.alpha());
        telemetry.addLine().addData("Red ", () -> colorSensor.red());
        telemetry.addLine().addData("Green ", () -> colorSensor.green());
        telemetry.addLine().addData("Blue ", () -> colorSensor.blue());
        telemetry.addLine().addData("Shaunak value ", () -> 1.0 * (colorSensor.red() + colorSensor.green()) / (colorSensor.red() + colorSensor.green() + colorSensor.blue()));

        telemetry.update();
    }
}
