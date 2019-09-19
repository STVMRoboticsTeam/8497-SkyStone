package net.camfeezel.robotics;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(group = "Sensor Tests", name = "01-Distance")
public class SensorTest1 extends LinearOpMode {

    private Rev2mDistanceSensor sensorDistance00;
    private Rev2mDistanceSensor sensorDistance18;
    private Rev2mDistanceSensor sensorDistance27;

    private ColorSensor sensorColorDown;

    private ModernRoboticsI2cGyro sensorGyro;

    private DcMotor motorFL0;
    private DcMotor motorFR1;
    private DcMotor motorBL2;
    private DcMotor motorBR3;

    private ElapsedTime timer1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        sensorDistance00 = hardwareMap.get(Rev2mDistanceSensor.class, "dist00");
        sensorDistance18 = hardwareMap.get(Rev2mDistanceSensor.class, "dist18");
        sensorDistance27 = hardwareMap.get(Rev2mDistanceSensor.class, "dist27");

        sensorColorDown = hardwareMap.colorSensor.get("colorDown");

        motorFL0 = hardwareMap.dcMotor.get("0");
        motorFR1 = hardwareMap.dcMotor.get("1");
        motorBL2 = hardwareMap.dcMotor.get("2");
        motorBR3 = hardwareMap.dcMotor.get("3");

        // GYRO Init and Calibration
        sensorGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        telemetry.log().add("DO NOT MOVE ROBOT! Gyro Calibrating...");
        sensorGyro.calibrate();
        timer1.reset();
        while (!isStopRequested() && sensorGyro.isCalibrating())  {
            telemetry.addData("Gyro Calibration", "%s", Math.round(timer1.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated.");
        telemetry.clear(); telemetry.update();

        waitForStart();
        telemetry.log().clear();
        double initialDist00 = sensorDistance00.getDistance(DistanceUnit.CM);
        double initialDist18 = sensorDistance18.getDistance(DistanceUnit.CM);
        double initialDist27 = sensorDistance27.getDistance(DistanceUnit.CM);
        double initialHeading = sensorGyro.getHeading();
        while(opModeIsActive()) {
            float zAngle = sensorGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("Heading", "%3d", sensorGyro.getHeading());
            telemetry.addData("Integ. Z", "%3d", sensorGyro.getIntegratedZValue());
            telemetry.addData("Angle", "%3d", String.format("%.3f", zAngle));

            float cd00 = (float) sensorDistance00.getDistance(DistanceUnit.CM);
            float cd18 = (float) sensorDistance18.getDistance(DistanceUnit.CM);
            float cd27 = (float) sensorDistance27.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance-000", "%d cm", cd00);
            telemetry.addData("Distance-180", "%d cm", cd18);
            telemetry.addData("Distance-270", "%d cm", cd27);

            telemetry.addLine("Color")
                    .addData("R", sensorColorDown.red())
                    .addData("G", sensorColorDown.green())
                    .addData("B", sensorColorDown.blue());
            telemetry.update();

            float x;
            float y;

            if(cd00 > 10.5) {
                y = Range.clip((cd00 - 8) / 10, 0, 1);
            } else if(cd00 < 9.5) {
                y = Range.clip((cd00 - 12) / 10, -1, 0);
            } else {
                y = 0;
            }

            if(cd27 > 10.5) {
                x = Range.clip((cd27 - 8) / 10, 0, 1);
            } else if(cd27 < 9.5) {
                x = Range.clip((cd27 - 12) / 10, -1, 0);
            } else {
                x = 0;
            }

            telemetry.addLine("Velocity").addData("X", x).addData("Y", y);
            setVelocity(x, y, 0);
        }
    }

    /**
     *
     * @param x speed in the 90/270 degrees direction. -1 to 1
     * @param y speed in the 0/180 degrees direction. -1 to 1
     * @param rot rotational speed, positive means positive degrees.
     */
    private void setVelocity(float x, float y, float rot) {
        float frFin = 0f;
        float flFin = 0f;
        float brFin = 0f;
        float blFin = 0f;

        x = Range.clip(x, -1, 1);
        y = Range.clip(y, -1, 1);
        rot = Range.clip(rot / 360, -1, 1);

        /*
         * fl = x + y + rot
         * fr = x - y - rot
         * bl = x - y + rot
         * br = x + y - rot
         *
         * final values need scaled // not clipped
         * scale rotation less than position
         */
        float curScale = 1f;
        float scale = 1f;

        // FL x + y + rot
        if(x + y + rot > 1f) {
            curScale = (1 - rot) / (x + y);
        } else if(x + y + rot < -1f) {
            curScale = (rot - 1) / (x + y);
        }
        if(curScale < scale) scale = curScale;

        // FR x - y - rot
        if(x - y - rot > 1f) {
            curScale = (1 + rot) / (x - y);
        } else if(x - y - rot < -1f) {
            curScale = (-rot - 1) / (x - y);
        }
        if(curScale < scale) scale = curScale;

        // BL x - y + rot
        if(x - y + rot > 1f) {
            curScale = (1 - rot) / (x - y);
        } else if(x - y + rot < -1f) {
            curScale = (rot - 1) / (x - y);
        }
        if(curScale < scale) scale = curScale;

        // BR x + y - rot
        if(x + y - rot > 1f) {
            curScale = (1 + rot) / (x + y);
        } else if(x + y - rot < -1f) {
            curScale = (-rot - 1) / (x + y);
        }
        if(curScale < scale) scale = curScale;

        flFin = (x + y) * scale + rot;
        frFin = (x - y) * scale - rot;
        blFin = (x - y) * scale + rot;
        brFin = (x + y) * scale - rot;

        telemetry.addLine("Motors")
                .addData("FL", flFin)
                .addData("FR", frFin)
                .addData("BL", blFin)
                .addData("BR", brFin);
        telemetry.update();

        motorFL0.setPower(flFin);
        motorFR1.setPower(-frFin);
        motorBL2.setPower(blFin);
        motorBR3.setPower(-brFin);
    }

}
