package net.camfeezel.robotics;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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

    private RevColorSensorV3 sensorColorDown;

    private ModernRoboticsI2cGyro sensorGyro;

    private DcMotor motorFL0;
    private DcMotor motorFR1;
    private DcMotor motorBL2;
    private DcMotor motorBR3;
    private MecanumControl mec;

    private ElapsedTime timer1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        sensorDistance00 = hardwareMap.get(Rev2mDistanceSensor.class, "dist00");
        sensorDistance18 = hardwareMap.get(Rev2mDistanceSensor.class, "dist18");
        sensorDistance27 = hardwareMap.get(Rev2mDistanceSensor.class, "dist27");

        sensorColorDown = hardwareMap.get(RevColorSensorV3.class, "colorDown");

        motorFL0 = hardwareMap.dcMotor.get("0");
        motorFR1 = hardwareMap.dcMotor.get("1");
        motorBL2 = hardwareMap.dcMotor.get("2");
        motorBR3 = hardwareMap.dcMotor.get("3");
        mec = new MecanumControl(motorFL0, motorFR1, motorBL2, motorBR3, telemetry);


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
            telemetry.addData("Angle", "%s", String.format("%.3f", zAngle));

            float cd00 = (float) sensorDistance00.getDistance(DistanceUnit.CM);
            float cd18 = (float) sensorDistance18.getDistance(DistanceUnit.CM);
            float cd27 = (float) sensorDistance27.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance-000", "%f cm", cd00);
            telemetry.addData("Distance-180", "%f cm", cd18);
            telemetry.addData("Distance-270", "%f cm", cd27);

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
            mec.setVelocity(x, y, 0);
        }
    }

}
