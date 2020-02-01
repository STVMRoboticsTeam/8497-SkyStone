package net.camfeezel.robotics;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import static java.lang.Math.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.GREEN;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;

@TeleOp(name = "GetJFCanada")
public class GetJFCanada extends LinearOpMode {

	private static final float mmPerInch        = 25.4f;


	private DcMotor motorFL0;
	private DcMotor motorFR1;
	private DcMotor motorBL2;
	private DcMotor motorBR3;

	private DcMotor motorIntakeL;
	private DcMotor motorIntakeR;

	private RevBlinkinLedDriver leds;

	private BNO055IMU imu;

	// Negative: Out
	private DcMotor motorSlideLat;
	// Positive: Up
	private DcMotor motorSlideVert;

	private Servo servoBlock0;

	private final boolean blueTeam = true;

	private static final String VUFORIA_KEY = //region
			"ASSzGtr/////AAABmZbPFu6zT0zIvBJNI9BxnCh58m/JkUORiHZOzTsf6RujF/GjGAERY9IEnRhkjoOmbTSQVldTUmKZBk3qEzxlXmwISSg7cbapxP+1k7+0kY9g1itZHc1PxwjSC+nJuP3Ua3/qdtKfRYbgBeJOS4h55ajSCPEy9+7Y7fgRcKcVC/bvW+bPukTpVB7LWBCqmLs0giRUc6SXBTUgaMyBcgkZEYTauqo9lUkxpgQyLfZXz6Ozs3c6D0FNL3q8XTP2WptVWM16/8VsDTZePEH9GGHYG8XiFdGgB7cXePoqe3EJfXY0SRjRQYjzdn32FEwVvM3lEsr9S3vGW9vZ1l4DDJAvmDka3eioBD5nd9yLAyvVFenk";
	//endregion

	@Override
	public void runOpMode() throws InterruptedException {
		motorFL0 = hardwareMap.dcMotor.get("0");
		motorFR1 = hardwareMap.dcMotor.get("1");
		motorBL2 = hardwareMap.dcMotor.get("2");
		motorBR3 = hardwareMap.dcMotor.get("3");

		imu = hardwareMap.get(AdafruitBNO055IMU.class, "imu");

		motorIntakeL = hardwareMap.dcMotor.get("intakeL");
		motorIntakeR = hardwareMap.dcMotor.get("intakeR");

		leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");

		motorSlideLat = hardwareMap.dcMotor.get("slideLat");
		motorSlideVert = hardwareMap.dcMotor.get("slideVert");
		servoBlock0 = hardwareMap.servo.get("hook");

		MecanumControl mec = new MecanumControl(motorFL0, motorFR1, motorBL2, motorBR3, telemetry);

		final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
		final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
		final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

		VuforiaControl vuf = new VuforiaControl(telemetry, hardwareMap, VUFORIA_KEY, CAMERA_FORWARD_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, -90, -90, 0);

		boolean autoMode = false;
		int autoPhase = 0;
		ElapsedTime phaseTime = new ElapsedTime();

		telemetry.addLine("Blue Team Selected");
		telemetry.addLine("Calibrating Gyro...");
		telemetry.update();

		BNO055IMU.Parameters param = new BNO055IMU.Parameters();


		imu.initialize(param);

		while(!isStopRequested() && !imu.isGyroCalibrated()) {

		}

		telemetry.clear();
		telemetry.addLine("Blue Team Selected");
		telemetry.addLine("Initialization Done. Ready for Start");
		telemetry.update();

		Orientation startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		float startHeading = (startAngle.secondAngle + 360) % 360;

		waitForStart();

		while(opModeIsActive()) {
			float lx = gamepad1.left_stick_x;
			float ly = gamepad1.left_stick_y;
			float rx = gamepad1.right_stick_x;

			Orientation curAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

			float x = 0;
			float y = 0;
			float rot = 0;
			
			float intL = 0;
			float intR = 0;

			if(Math.abs(lx) > 0.05f) x = (float) Math.pow(lx, 3);
			if(Math.abs(ly) > 0.05f) y = (float) Math.pow(ly, 3);
			if(Math.abs(rx) > 0.05f) rot = rx*180;

			// FIELD CENTRIC CORRECTIONS
			float curHeading = (curAngle.secondAngle + 360) % 360;
			float angleDiff = curHeading - startHeading;
			angleDiff += 90;

			x = (float) cos(toRadians(angleDiff));
			y = (float) sin(toRadians(angleDiff));


			float lt = gamepad2.left_trigger;
			float rt = gamepad2.right_trigger;
			float ry2 = gamepad2.right_stick_y;

			if(gamepad2.y) motorSlideLat.setPower(0.3f);
			if(gamepad2.a) motorSlideLat.setPower(0.3f);
			if(ry2 > 0.05f) motorSlideVert.setPower(ry2);
			else if(ry2 < -0.05f) motorSlideVert.setPower(ry2);
			else motorSlideVert.setPower(0f);
			if(lt > 0.05f) servoBlock0.setPosition(1f);
			else if(rt > 0.05f) servoBlock0.setPosition(-1f);


			
			
			if(gamepad2.dpad_down) {
				intR = (1);
				intL = (-1);
			} else if(gamepad2.dpad_up) {
				intR = (-1);
				intL = (1);
			} else if(gamepad2.dpad_left || gamepad2.dpad_right) {
				intR = (-1);
				intL = (-1);
			} else {
				intR = (0);
				intL = (0);
			}

			if(gamepad1.x) autoMode = true;
			else {
				if(autoMode) {
					leds.setPattern(RAINBOW_RAINBOW_PALETTE);
				}
				autoMode = false;
				autoPhase = 0;
			}

			if(gamepad2.b) {
				leds.setPattern(blueTeam ? STROBE_BLUE : STROBE_RED);
			} else {
				leds.setPattern(RAINBOW_RAINBOW_PALETTE);
			}

			if(autoMode) {
				if(autoPhase == 0) {
					Recognition rec = vuf.findStone(false);
					if(rec == null) {
						telemetry.addLine("NO BLOCK FOUND");
						telemetry.update();
						autoPhase = -1;
						leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
						phaseTime.reset();
					} else {
						float angle = vuf.getAngle(rec);
						if(rec.getHeight() / rec.getImageHeight() > 0.6f)
							y = -0.4f;
						else y = -1f;
						x = 0;
						rot = angle*2;
						intL = (gamepad2.b ? 1 : -1);
						intR = (1);
						if(gamepad2.x) {
							autoPhase = 1;
							phaseTime.reset();
							leds.setPattern(GREEN);
						}
					}

				} else if(autoPhase == -1) {
					if(phaseTime.seconds() >= 1) {
						leds.setPattern(RAINBOW_RAINBOW_PALETTE);
					}
				} else if(autoPhase == 1) {
					if(phaseTime.seconds() >= 1) {
						leds.setPattern(RAINBOW_RAINBOW_PALETTE);
					}

				}
			}

			motorIntakeL.setPower(intL);
			motorIntakeR.setPower(intR);
			mec.setVelocity(x, y, rot);
		}
	}
}
