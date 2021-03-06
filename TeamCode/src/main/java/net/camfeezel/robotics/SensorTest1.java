package net.camfeezel.robotics;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
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
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous(group = "Sensor Tests", name = "01-Distance")
public class SensorTest1 extends LinearOpMode {

    private Rev2mDistanceSensor sensorDistance00;
    private Rev2mDistanceSensor sensorDistance18;
    private Rev2mDistanceSensor sensorDistance27;

    private RevColorSensorV3 sensorColor00;

    private BNO055IMU sensorGyro;
	private Orientation angles = new Orientation();

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

        sensorColor00 = hardwareMap.get(RevColorSensorV3.class, "color00");

        motorFL0 = hardwareMap.dcMotor.get("0");
        motorFR1 = hardwareMap.dcMotor.get("1");
        motorBL2 = hardwareMap.dcMotor.get("2");
        motorBR3 = hardwareMap.dcMotor.get("3");
        mec = new MecanumControl(motorFL0, motorFR1, motorBL2, motorBR3, telemetry);


        // GYRO Init and Calibration
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.mode                = BNO055IMU.SensorMode.IMU;
		parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.loggingEnabled      = false;

        sensorGyro = hardwareMap.get(BNO055IMU.class, "imu");
        telemetry.log().add("DO NOT MOVE ROBOT! Gyro Calibrating...");
        sensorGyro.initialize(parameters);
        timer1.reset();
        while (!isStopRequested() && !sensorGyro.isGyroCalibrated())  {
            sleep(50);
            idle();
        }
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibration status (" + timer1.seconds() + "s) " + sensorGyro.getCalibrationStatus().toString());
        telemetry.clear(); telemetry.update();
		float initialHeading = angles.firstAngle;
        waitForStart();
        telemetry.log().clear();
		Acceleration gravity = new Acceleration();
		sensorGyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);
		double initialDist00 = sensorDistance00.getDistance(DistanceUnit.CM);
        double initialDist18 = sensorDistance18.getDistance(DistanceUnit.CM);
        double initialDist27 = sensorDistance27.getDistance(DistanceUnit.CM);
		angles = sensorGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

		boolean searching = false;
		boolean yOnPoint = false;
		int curBlock = 0;
		int skyBlock = 0;
		int cmTarget = 94;

		while(opModeIsActive()) {
            angles = sensorGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			telemetry.addData("Heading", formatAngle(angles.angleUnit, angles.firstAngle));


			float x = 0;
			float y = 0;

            float cd00 = (float) sensorDistance00.getDistance(DistanceUnit.CM);
            float cd18 = (float) sensorDistance18.getDistance(DistanceUnit.CM);
            float cd27 = (float) sensorDistance27.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance-000", "%f cm", cd00);
            telemetry.addData("Distance-180", "%f cm", cd18);
            telemetry.addData("Distance-270", "%f cm", cd27);




            if(cd00 > 8) {
            	yOnPoint = false;
                y = Range.clip((cd00 - 6) / 25, 0, 1);
            } else if(cd00 < 4) {
				yOnPoint = false;
                y = Range.clip((cd00 - 6) / 25, -1, 0);
            } else {
                y = 0;
                yOnPoint = true;
            }
			NormalizedRGBA colors = sensorColor00.getNormalizedColors();
            telemetry.addLine("Color")
					.addData("R", colors.red)
					.addData("G", colors.green)
					.addData("B", colors.blue);

			if (cd27 > cmTarget+2) {
				x = Range.clip((cd27 - cmTarget) / 25, 0, 1);
			} else if (cd27 < cmTarget-2) {
				x = Range.clip((cd27 - cmTarget) / 25, -1, 0);
			} else {
				searching = true;
				x = 0;
			}
			if(searching && yOnPoint) {
				if(colors.red > 0.3f && colors.green > 0.3f) {
					cmTarget -= (++curBlock)*20;
					searching = false;
				} else {
					skyBlock = 6-curBlock;
				}
			}
			if(curBlock == 6) {
				telemetry.clear();
				telemetry.addLine("No SkyStones");
				telemetry.update();
				mec.setVelocity(0,0,0);
				while(opModeIsActive());
			}
			if(skyBlock != 0) {
				telemetry.clear();
				telemetry.addData("SkyStone Found", skyBlock);
				telemetry.update();
				mec.setVelocity(0,0,0);
				while(opModeIsActive());			}


            float rot = 0;
            float ang = angles.firstAngle;
            rot = (ang - initialHeading);
            telemetry.addLine("Velocity").addData("X", x).addData("Y", y).addData("ROT", rot);
            mec.setVelocity(x, y, rot);
			telemetry.update();
        }
    }

	String formatAngle(AngleUnit angleUnit, double angle) {
		return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
	}

	String formatDegrees(double degrees){
		return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
	}

}
