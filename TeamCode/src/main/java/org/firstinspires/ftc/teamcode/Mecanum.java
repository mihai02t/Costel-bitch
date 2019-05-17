package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class Mecanum extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor up;
    private DcMotor brat;
    private DcMotor collector;
    private DcMotor culisanta;
    private DigitalChannel endup;
    private DigitalChannel endbrat;
    private DigitalChannel endculisanta;

    private double motorPower = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {

        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        up = hardwareMap.dcMotor.get("up");
        brat = hardwareMap.dcMotor.get("brat");
        collector = hardwareMap.dcMotor.get("collector");
        culisanta = hardwareMap.dcMotor.get("culisanta");
        endup = hardwareMap.get(DigitalChannel.class, "btt");
        endbrat = hardwareMap.get(DigitalChannel.class, "endbrat");
        endculisanta = hardwareMap.get(DigitalChannel.class, "endculisanta");
        up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        culisanta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//      brat.setDirection(DcMotor.Direction.REVERSE);
//      culisanta.setDirection(DcMotor.Direction.REVERSE);
//        up.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Init >", "Done");
        telemetry.update();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            double G1LeftStickY = -gamepad1.left_stick_y;
            double G1LeftStickX = gamepad1.left_stick_x;
            double G1RightStickX = gamepad1.right_stick_x;
            boolean G1RightBumper = gamepad1.right_bumper;
            boolean G1LeftBumper = gamepad1.left_bumper;
            boolean G1RightTrigger = (gamepad1.right_trigger > 0.05);
            boolean G1LeftTrigger = (gamepad1.left_trigger > 0.05);

            boolean G2RightBumper = gamepad2.right_bumper;
            boolean G2LeftBumper = gamepad2.left_bumper;
            boolean G2RightTrigger = (gamepad2.right_trigger > 0.05);
            boolean G2LeftTrigger = (gamepad2.left_trigger > 0.05);

            move(G1LeftStickX, G1LeftStickY, G1RightStickX);

            if(G1RightTrigger) up.setPower(-0.8);
            else if(G1LeftTrigger && !endup.getState()) up.setPower(0.8);
            else up.setPower(0.0);

            if(G1RightBumper) collector.setPower(1.0);
            else if(G1LeftBumper) collector.setPower(-1.0);
            else collector.setPower(0.0);

            if(G2RightTrigger) culisanta.setPower(1.0);
            else if(G2LeftTrigger && !endculisanta.getState()) culisanta.setPower(-1.0);
            else culisanta.setPower(0.0);

            if(G2RightBumper) brat.setPower(1.0);
            else if(G2LeftBumper && !endbrat.getState()) brat.setPower(-1.0);
            else brat.setPower(0.0);
//
//          culisanta.setPower(G2LeftStickY);
//          brat.setPower(G2RightStickY);

            failsafe();
        }
    }

    private void move(double x, double y, double rotation) {
        y = -y;
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;

        frontLeft.setPower (motorPower * (r * Math.cos(robotAngle) + rotation));
        frontRight.setPower(motorPower * (r * Math.sin(robotAngle) - rotation));
        backLeft.setPower  (motorPower * (r * Math.sin(robotAngle) + rotation));
        backRight.setPower (motorPower * (r * Math.cos(robotAngle) - rotation));
    }

    private void failsafe() {
        if(getBatteryVoltage() < 10) {
            frontRight.setPower(0.0);
            frontLeft.setPower(0.0);
            backRight.setPower(0.0);
            backLeft.setPower(0.0);
            up.setPower(0.0);
            brat.setPower(0.0);
            collector.setPower(0.0);
            culisanta.setPower(0.0);
        }
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}