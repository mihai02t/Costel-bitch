package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

@Autonomous(name = "MecanumDemoAuto", group = "LinearOpMode")
@Disabled
public class MecanumDemoAuto extends LinearOpMode {


    private static final String VUFORIA_KEY = "ATD0J3D/////AAABmd39V35PGUhYl+CL7UZFkI95JbUeEL99dL/kF1Zn7cyhPoSCvATY6s1es0/AgsJo7nt818iId/Ugcs8cMHP7kP+LG7ixm7asuVSJqdkcOcJjnBMnIiTRLpMXKLOgbDLFHeCfDUt/C5kVVmFWyK3s8Uch5AnctvE4Z8GJfZlL0um5gQU4jw9Yok4K/+T7gMEc+3yfDqCyH9wzjddLZvt8IOyW9mBu63IKFFFL/wlmZjyFYF+KFqcgoUMEaY9uiC+tBW+P7ihKTyC8vdLBrVXIcigjEp6umkDHnWwnHiVDElwCaolWVc8NEAtOZA3jgCXHuXheh4L1slHm+jJohD1y7CJ0+8N/CXYXnIMCmykZoxiv";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String TAG = "VuforiaTFOD.Test";

    private VuforiaLocalizer localizer;
    private TFObjectDetector detector;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor up;
    private DcMotor brat;
    private DcMotor collector;
    private Servo servo;
    private BNO055IMU gyro;
    private DistanceSensor range;
    private DigitalChannel endup;

    DcMotor[] motors;
    Orientation angles = new Orientation();
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

    private double motorPower = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        range = hardwareMap.get(DistanceSensor.class, "range");
        servo = hardwareMap.servo.get("tm");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        up = hardwareMap.dcMotor.get("up");
//      brat = hardwareMap.dcMotor.get("brat");
//      collector = hardwareMap.dcMotor.get("collector");

        endup = hardwareMap.get(DigitalChannel.class, "btt");

        motors = new DcMotor[] { frontLeft, frontRight, backLeft, backRight };

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        servo.setPosition(0.7);
//      brat.setDirection(DcMotor.Direction.REVERSE);
//        up.setDirection(DcMotor.Direction.REVERSE);
        up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.mode                = BNO055IMU.SensorMode.IMU;
        param.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.loggingEnabled      = false;
        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        for(DcMotor m: motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Thread.sleep(250);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();

        gyro.initialize(param);
        while (!isStopRequested() && !gyro.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.");
        telemetry.update();

        // ---- vuforia initialization
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.localizer = ClassFactory.getInstance().createVuforia(parameters);

        // ---- tfod initialization
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        this.detector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, this.localizer);
        this.detector.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

//        while(!gamepad1.x) {
//            if(gamepad1.x) break;
//
//            telemetry.addData("Heading >", getHeading());
//            telemetry.update();
//
//            idle();
//        }

        telemetry.addData(">", "Init done.");
        telemetry.update();

        waitForStart();

        CameraDevice.getInstance().setFlashTorchMode(true);

        if(this.detector != null) {
            this.detector.activate();
        }

        while (opModeIsActive() && !isStopRequested()) {
            double G1LeftStickY = -gamepad1.left_stick_y;
            double G1LeftStickX = gamepad1.left_stick_x;
            double G1RightStickX = gamepad1.right_stick_x;
            boolean G1RightBumper = gamepad1.right_bumper;
            boolean G1LeftBumper = gamepad1.left_bumper;

            double G2LeftStickY = -gamepad2.left_stick_y;
            double G2RightStickY = -gamepad2.right_stick_y;

            move(G1LeftStickX, G1LeftStickY, G1RightStickX);
            if(G1RightBumper) up.setPower(-0.8);
            else if(G1LeftBumper && !endup.getState()) up.setPower(0.8);
            else up.setPower(0.0);
//
            for(DcMotor m: motors) {
                int pos = m.getCurrentPosition();
                telemetry.addData("motorPos", pos);
            }
            int heading = getHeading();
            telemetry.addData("heading", heading);
//            telemetry.addData("target", this.target);
//            telemetry.addData("willTurn", this.getTurnDirection(heading, this.target).toString());
            telemetry.update();

            if(gamepad1.a) {
                while(gamepad1.a) { idle(); }
                up.setPower(0.8);
                while(opModeIsActive() && !endup.getState()) {
                    idle();
                }
                up.setPower(0.0);
                this.moveStraightToTarget(-300, 0.4, motors);
                Thread.sleep(500);
                this.strafeToTarget(1500, 0.4, motors);
                Thread.sleep(500);
                this.moveStraightToTarget(-550, 0.4, motors);
            }

            if(gamepad1.b) {
                while(gamepad1.b) { idle(); }
                this.move(-0.5, 0.0, 0.0, 1000);
            }


            if(gamepad1.y) {
                while (gamepad1.y) idle();

                up.setPower(1.0);
                while(opModeIsActive() && !endup.getState()) {
                    idle();
                }
                up.setPower(0.0);
                this.moveStraightToTarget(-300, 0.4, motors);
                Thread.sleep(100);
                this.strafeToTarget(1500, 0.5, motors);
                Thread.sleep(100);
                this.moveStraightToTarget(-550, 0.5, motors);

//                turnToTarget(45, 0.3);
//                Thread.sleep(100);
////                this.move(-0.5, 0.0, 0.0, 1000);
//                this.strafeToTarget(-800, 0.4, motors);
////                Thread.sleep(500);
//                moveStraightToTarget(-400, 0.3, motors);
//                Thread.sleep(300);

                // do the detection here

                List<Recognition> recognitions = null;
                GoldMineralPosition position = GoldMineralPosition.NONE;
                while(opModeIsActive()) {
                    if(this.detector != null) {
                        recognitions = this.detector.getUpdatedRecognitions();
                        if(recognitions != null && recognitions.size() == 2) {
                            Recognition mineral1 = recognitions.get(0);
                            Recognition mineral2 = recognitions.get(1);

                            if(mineral1.getLeft() < mineral2.getLeft()) {
                                if(mineral1.getLabel() == LABEL_GOLD_MINERAL && mineral2.getLabel() == LABEL_SILVER_MINERAL) {
                                    position = GoldMineralPosition.CENTER;
                                }
                                else if(mineral1.getLabel() == LABEL_SILVER_MINERAL && mineral2.getLabel() == LABEL_GOLD_MINERAL) {
                                    position = GoldMineralPosition.RIGHT;
                                }
                                else if(mineral1.getLabel() == LABEL_SILVER_MINERAL && mineral2.getLabel() == LABEL_SILVER_MINERAL) {
                                    position = GoldMineralPosition.LEFT;
                                }
                            }
                            else if(mineral2.getLeft() < mineral1.getLeft()) {
                                if(mineral1.getLabel() == LABEL_GOLD_MINERAL && mineral2.getLabel() == LABEL_SILVER_MINERAL) {
                                    position = GoldMineralPosition.RIGHT;
                                }
                                else if(mineral1.getLabel() == LABEL_SILVER_MINERAL && mineral2.getLabel() == LABEL_GOLD_MINERAL) {
                                    position = GoldMineralPosition.CENTER;
                                }
                                else if(mineral1.getLabel() == LABEL_SILVER_MINERAL && mineral2.getLabel() == LABEL_SILVER_MINERAL) {
                                    position = GoldMineralPosition.LEFT;
                                }
                            }

                            if(position != GoldMineralPosition.NONE)
                                break;
                        }
                    }
                    idle();
                }

                telemetry.addData("Gold Position", position.toString());
                telemetry.update();

                if(position == GoldMineralPosition.LEFT) {
                    moveStraightToTarget(2100, 0.6, motors);
                }
                else if(position == GoldMineralPosition.CENTER) {
                    moveStraightToTarget(950, 0.6, motors);
                }
                else if(position == GoldMineralPosition.RIGHT) {
                    moveStraightToTarget(-500, 0.6, motors);
                }

                strafeToTarget(1600, 0.6, motors);
                strafeToTarget(-1100, 0.6, motors);

                Thread.sleep(100);
                turnToTarget(45, 0.3);
                Thread.sleep(100);

                move(0.0, 0.7, 0.0);
                while(opModeIsActive() && range.getDistance(DistanceUnit.CM) > 32.0) {
//                    telemetry.addData("Range: ", range.getDistance(DistanceUnit.CM));
//                    telemetry.update();
                    idle();
                }
                move(0.0, 0.0, 0.0);

//                if(true) continue;

                Thread.sleep(100);
                turnToTarget(85, 0.3);
                Thread.sleep(100);

//                move(0.5, 0.0, 0.0);
//                while(opModeIsActive() && range.getDistance(DistanceUnit.CM) > 18.0) {
//                    idle();
//                }
//                move(0.0, 0.0, 0.0);
//                Thread.sleep(300);
//                turnToTarget(87, 0.3);
//                Thread.sleep(300);

                move(0.0, 0.8, 0.0);
                while(opModeIsActive() && range.getDistance(DistanceUnit.CM) > 50.0) {
                    idle();
                }
                move(0.0, 0.0, 0.0);
                Thread.sleep(100);
                servo.setPosition(0.1);

//                Thread.sleep(100);
//                turnToTarget(88, 0.3);
//                Thread.sleep(100);

                moveStraightToTarget(-5000, 0.9, motors);
                servo.setPosition(0.7);


                Thread.sleep(6000);
                // while(opModeIsActive());
            }

            if(gamepad1.x) {
                while (gamepad1.x) idle();

                up.setPower(1.0);
                while(opModeIsActive() && !endup.getState()) {
                    idle();
                }
                up.setPower(0.0);
                this.moveStraightToTarget(-300, 0.4, motors);
                Thread.sleep(100);
                this.strafeToTarget(1500, 0.5, motors);
                Thread.sleep(100);
                this.moveStraightToTarget(-550, 0.5, motors);

//                turnToTarget(45, 0.3);
//                Thread.sleep(100);
////                this.move(-0.5, 0.0, 0.0, 1000);
//                this.strafeToTarget(-800, 0.4, motors);
////                Thread.sleep(500);
//                moveStraightToTarget(-400, 0.3, motors);
//                Thread.sleep(300);

                // do the detection here

                List<Recognition> recognitions = null;
                GoldMineralPosition position = GoldMineralPosition.NONE;
                while(opModeIsActive()) {
                    if(this.detector != null) {
                        recognitions = this.detector.getUpdatedRecognitions();
                        if(recognitions != null && recognitions.size() == 2) {
                            Recognition mineral1 = recognitions.get(0);
                            Recognition mineral2 = recognitions.get(1);

                            if(mineral1.getLeft() < mineral2.getLeft()) {
                                if(mineral1.getLabel() == LABEL_GOLD_MINERAL && mineral2.getLabel() == LABEL_SILVER_MINERAL) {
                                    position = GoldMineralPosition.CENTER;
                                }
                                else if(mineral1.getLabel() == LABEL_SILVER_MINERAL && mineral2.getLabel() == LABEL_GOLD_MINERAL) {
                                    position = GoldMineralPosition.RIGHT;
                                }
                                else if(mineral1.getLabel() == LABEL_SILVER_MINERAL && mineral2.getLabel() == LABEL_SILVER_MINERAL) {
                                    position = GoldMineralPosition.LEFT;
                                }
                            }
                            else if(mineral2.getLeft() < mineral1.getLeft()) {
                                if(mineral1.getLabel() == LABEL_GOLD_MINERAL && mineral2.getLabel() == LABEL_SILVER_MINERAL) {
                                    position = GoldMineralPosition.RIGHT;
                                }
                                else if(mineral1.getLabel() == LABEL_SILVER_MINERAL && mineral2.getLabel() == LABEL_GOLD_MINERAL) {
                                    position = GoldMineralPosition.CENTER;
                                }
                                else if(mineral1.getLabel() == LABEL_SILVER_MINERAL && mineral2.getLabel() == LABEL_SILVER_MINERAL) {
                                    position = GoldMineralPosition.LEFT;
                                }
                            }

                            if(position != GoldMineralPosition.NONE)
                                break;
                        }
                    }
                    idle();
                }

                telemetry.addData("Gold Position", position.toString());
                telemetry.update();

                if(position == GoldMineralPosition.LEFT) {
                    moveStraightToTarget(2100, 0.6, motors);
                }
                else if(position == GoldMineralPosition.CENTER) {
                    moveStraightToTarget(950, 0.6, motors);
                }
                else if(position == GoldMineralPosition.RIGHT) {
                    moveStraightToTarget(-500, 0.6, motors);
                }

                strafeToTarget(1600, 0.6, motors);
                strafeToTarget(-1100, 0.6, motors);

                Thread.sleep(100);
                turnToTarget(135, 0.5);
                Thread.sleep(100);

                move(0.0, 0.7, 0.0);
                while(opModeIsActive() && range.getDistance(DistanceUnit.CM) > 35.0) {
//                    telemetry.addData("Range: ", range.getDistance(DistanceUnit.CM));
//                    telemetry.update();
                    idle();
                }
                move(0.0, 0.0, 0.0);

//                if(true) continue;

                Thread.sleep(100);
                turnToTarget(0, 0.5);
                Thread.sleep(100);

//                move(0.5, 0.0, 0.0);
//                while(opModeIsActive() && range.getDistance(DistanceUnit.CM) > 18.0) {
//                    idle();
//                }
//                move(0.0, 0.0, 0.0);
//                Thread.sleep(300);
//                turnToTarget(87, 0.3);
//                Thread.sleep(300);

                move(0.0, 0.8, 0.0);
                while(opModeIsActive() && range.getDistance(DistanceUnit.CM) > 50.0) {
                    idle();
                }
                move(0.0, 0.0, 0.0);
                Thread.sleep(100);
                servo.setPosition(0.1);

//                Thread.sleep(100);
//                turnToTarget(88, 0.3);
//                Thread.sleep(100);

                moveStraightToTarget(-5000, 0.9, motors);
                servo.setPosition(0.7);


                Thread.sleep(6000);
                // while(opModeIsActive());
            }

//            if(G1RightBumper) collector.setPower(-0.5);
//            else if(G1LeftBumper) collector.setPower(1.0);
//            else collector.setPower(0.0);
//
//            culisanta.setPower(G2LeftStickY);
//            brat.setPower(G2RightStickY);

            // failsafe();
        }

        CameraDevice.getInstance().setFlashTorchMode(false);
    }

    private enum ETurnDirection {
        LEFT,
        RIGHT
    }

    private enum GoldMineralPosition {
        NONE,
        LEFT,
        CENTER,
        RIGHT
    }

    private ETurnDirection getTurnDirection(int current, int target) {
        return current >= target ?
                (Math.abs(current - target) > 180 ? ETurnDirection.LEFT : ETurnDirection.RIGHT) :
                (Math.abs(current - target) > 180 ? ETurnDirection.RIGHT : ETurnDirection.LEFT);
    }

    private int opposite(int angle) {
        return (angle + 180) % 360;
    }

    private void turnToTarget(int target, double power) throws InterruptedException {
        if(getTurnDirection(getHeading(), target) == ETurnDirection.RIGHT) {
            move(0.0, 0.0, power);
        }
        else {
            move(0.0, 0.0, -power);
        }
        int lower = Math.max(target - 2, 0);
        int higher = Math.min(target + 2, 359);

        int heading = getHeading();
        while(opModeIsActive()) { idle(); /*telemetry.addData("heading", heading); telemetry.update(); */ heading = getHeading(); if(heading >= lower && heading <= higher) break; }

        move(0.0, 0.0, 0.0);
        Thread.sleep(100);
    }

    private void moveStraightToTarget(int target, double speed, DcMotor[] motors) throws InterruptedException {
        target = -target;
        for(DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Thread.sleep(20);
        }

        for(DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(target);
            motor.setPower(speed);
        }

        while (motors[0].isBusy() && opModeIsActive()) {
            idle();
            telemetry.addData("motor", motors[0].getCurrentPosition());
            telemetry.update();
        }

        for(DcMotor motor: motors) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void strafeToTarget(int target, double speed, DcMotor[] motors) throws InterruptedException {
        for(DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Thread.sleep(20);
        }

        frontLeft.setTargetPosition(target);
        backLeft.setTargetPosition(-target);
        frontRight.setTargetPosition(-target);
        backRight.setTargetPosition(target);

        for(DcMotor motor: motors) {
            motor.setPower(speed);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while (motors[0].isBusy() && opModeIsActive()) {
            idle();
            telemetry.addData("motor", motors[0].getCurrentPosition());
            telemetry.update();
        }

        for(DcMotor motor: motors) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void diagRightToTarget(int target, double speed, DcMotor[] motors) throws InterruptedException {
        for(DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Thread.sleep(20);
        }

        if(target > 0) {
            frontLeft.setTargetPosition(target);
            backRight.setTargetPosition(target);
        }
        else {
            frontRight.setTargetPosition(-target);
            backLeft.setTargetPosition(-target);
        }

        for(DcMotor motor: motors) {
            motor.setPower(speed);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while (motors[0].isBusy() && opModeIsActive()) {
            idle();
            telemetry.addData("motor", motors[0].getCurrentPosition());
            telemetry.update();
        }

        for(DcMotor motor: motors) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void diagLeftToTarget(int target, double speed, DcMotor[] motors) throws InterruptedException {
        for(DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Thread.sleep(20);
        }

        if(target > 0) {
            frontRight.setTargetPosition(target);
            backLeft.setTargetPosition(target);
        }
        else {
            frontLeft.setTargetPosition(-target);
            backRight.setTargetPosition(-target);
        }

        for(DcMotor motor: motors) {
            motor.setPower(speed);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while (motors[0].isBusy() && opModeIsActive()) {
            idle();
            telemetry.addData("motor", motors[0].getCurrentPosition());
            telemetry.update();
        }

        for(DcMotor motor: motors) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void move(double left, double right, double rotation, long time) throws InterruptedException {
        move(left, right, rotation);
        Thread.sleep(time);
        move(0.0, 0.0, 0.0);
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

    private int getHeading() {
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int val = (int)Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        if(val > 0)
            return val;

        val = -val;
        val = 360 - val;
        return val;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private void failsafe() {
        if(getBatteryVoltage() < 10.5) {
            frontRight.setPower(0.0);
            frontLeft.setPower(0.0);
            backRight.setPower(0.0);
            backLeft.setPower(0.0);
//            culisanta.setPower(0.0);
//            brat.setPower(0.0);
//            collector.setPower(0.0);
        }
    }

    public double getBatteryVoltage() {
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