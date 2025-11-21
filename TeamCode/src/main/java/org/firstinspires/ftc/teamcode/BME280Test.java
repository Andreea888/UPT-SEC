package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.FileWriter;

@TeleOp(name="BME280 Test", group="Sensor")
public class BME280Test extends LinearOpMode {

    private DcMotorEx frontRight = null;
    private DcMotorEx frontLeft = null;
    private Servo gripper = null;
    private Servo brat = null;

    private float pFrontRight, pFrontLeft;

    @Override
    public void runOpMode() {

        // Get the sensor from hardwareMap
        BME280 bme280 = hardwareMap.get(BME280.class, "UPTSensor");

        // Initialize it
        bme280.initialize();

        getHardware();
        waitForStart();

        while (opModeIsActive()) {

            power();
            frontLeft.setPower(pFrontLeft);
            frontRight.setPower(pFrontRight);

            // Read values
            double temperature = bme280.readTemperatureC();
            double pressure = bme280.readPressureHPa();
            double humidity = bme280.readHumidityPercent();

            // Display on telemetry
            telemetry.addData("Temperature (°C)", temperature);
            telemetry.addData("Pressure (hPa)", pressure);
            telemetry.addData("Humidity (%)", humidity);
            telemetry.addData("gripper poz", gripper.getPosition());
            telemetry.addData("brat poz", brat.getPosition());

            telemetry.update();

            try {
                FileWriter fw = new FileWriter("/sdcard/sensor_data.txt");
                fw.write(String.format("temperature=%.2f\nhumidity=%.2f\n", temperature, humidity));
                fw.close();
            } catch (Exception e) {
                telemetry.addData("File Error", e.getMessage());
                telemetry.update();
            }

            // Wait a short time to avoid spamming
            sleep(500);
        }
    }

    private void power() {

        //forward-backward
        pFrontLeft = gamepad1.left_trigger - gamepad1.right_trigger;
        pFrontRight = gamepad1.left_trigger - gamepad1.right_trigger;


        //steer
        pFrontRight += gamepad1.left_stick_x;
        pFrontLeft -= gamepad1.left_stick_x;


        float maxim = 0;
        if (Math.abs(pFrontRight) > maxim)
            maxim = (Math.abs(pFrontRight));
        if (Math.abs(pFrontLeft) > maxim)
            maxim = (Math.abs(pFrontLeft));


        if (maxim > 1.0 && maxim != 0) {
            pFrontRight /= maxim;
            pFrontLeft /= maxim;

        }
        if (gamepad1.right_bumper) {
            pFrontRight /= 2;
            pFrontLeft /= 2;

        }
    }

    public void getHardware() {


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        frontLeft = hardwareMap.get(DcMotorEx.class, "LeftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "RightFront");

        gripper = hardwareMap.get(Servo.class, "Gripper");
        brat = hardwareMap.get(Servo.class, "Brat");

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }
}