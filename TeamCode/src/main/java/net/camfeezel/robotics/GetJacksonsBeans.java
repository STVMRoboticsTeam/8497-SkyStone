package net.camfeezel.robotics;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BLUE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.GREEN;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.LIME;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;

@Autonomous(name = "GetJacksonsBeans")
public class GetJacksonsBeans extends LinearOpMode {

	private static final float mmPerInch        = 25.4f;


	private DcMotor motorFL0;
	private DcMotor motorFR1;
	private DcMotor motorBL2;
	private DcMotor motorBR3;

	private TouchSensor touchSensor;

	private DcMotor motorIntakeL;
	private DcMotor motorIntakeR;

	private RevBlinkinLedDriver leds;
	private RevColorSensorV3 colorDown;

	// Negative: Out
	private DcMotor motorSlideLat;
	// Positive: Up
	private DcMotor motorSlideVert;

	private Servo servoBlock0;

	private boolean blueTeam = true;

	private static final String VUFORIA_KEY = //region
			"ASSzGtr/////AAABmZbPFu6zT0zIvBJNI9BxnCh58m/JkUORiHZOzTsf6RujF/GjGAERY9IEnRhkjoOmbTSQVldTUmKZBk3qEzxlXmwISSg7cbapxP+1k7+0kY9g1itZHc1PxwjSC+nJuP3Ua3/qdtKfRYbgBeJOS4h55ajSCPEy9+7Y7fgRcKcVC/bvW+bPukTpVB7LWBCqmLs0giRUc6SXBTUgaMyBcgkZEYTauqo9lUkxpgQyLfZXz6Ozs3c6D0FNL3q8XTP2WptVWM16/8VsDTZePEH9GGHYG8XiFdGgB7cXePoqe3EJfXY0SRjRQYjzdn32FEwVvM3lEsr9S3vGW9vZ1l4DDJAvmDka3eioBD5nd9yLAyvVFenk";
	//endregion

	@Override
	public void runOpMode() throws InterruptedException {
		motorFL0 = hardwareMap.dcMotor.get("0");
		motorFR1 = hardwareMap.dcMotor.get("1");
		motorBL2 = hardwareMap.dcMotor.get("2");
		motorBR3 = hardwareMap.dcMotor.get("3");

		motorIntakeL = hardwareMap.dcMotor.get("intakeL");
		motorIntakeR = hardwareMap.dcMotor.get("intakeR");

		touchSensor = hardwareMap.touchSensor.get("touch");

		colorDown = hardwareMap.get(RevColorSensorV3.class, "colorDown");


		leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");

		motorSlideLat = hardwareMap.dcMotor.get("slideLat");
		motorSlideVert = hardwareMap.dcMotor.get("slideVert");
		servoBlock0 = hardwareMap.servo.get("hook");

		MecanumControl mec = new MecanumControl(motorFL0, motorFR1, motorBL2, motorBR3, telemetry);

		final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
		final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
		final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

		VuforiaControl vuf = new VuforiaControl(telemetry, hardwareMap, VUFORIA_KEY, CAMERA_FORWARD_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, -90, -90, 0);

		int autoPhase = 0;
		ElapsedTime phaseTime = new ElapsedTime();
		telemetry.clear();
		telemetry.addLine("Press Team Color on 1");
		telemetry.update();

		while(!(gamepad1.x || gamepad1.b) && !isStopRequested()) {

		}

		if(gamepad1.x) {
			blueTeam = true;
		} else if(gamepad1.b) {
			blueTeam = false;
		}

		telemetry.addLine("Team Selected: " + (blueTeam ? "Blue" : "Red"));
		telemetry.update();

		while(blueTeam ? gamepad1.x : gamepad1.b) {

		}

		telemetry.addLine("Line is on: (X)Left (B)Right");
		telemetry.update();

		while(!(gamepad1.x || gamepad1.b) && !isStopRequested()) {

		}

		boolean left = true;
		if(gamepad1.x) {
			left = true;
		} else if(gamepad1.b) {
			left = false;
		}

		telemetry.clear();
		telemetry.addLine("Line is on: " + (left ? "Left" : "Right"));
		telemetry.update();

		while(left ? gamepad1.x : gamepad1.b) {

		}

		telemetry.addLine("Which Lane: (A)Wall (Y)Center");
		telemetry.update();

		while(!(gamepad1.a || gamepad1.y) && !isStopRequested()) {

		}

		boolean wallLane = true;
		if(gamepad1.a) {
			wallLane = true;
		} else if(gamepad1.y) {
			wallLane = false;
		}

		telemetry.addLine("Lane: " + (wallLane ? "Wall" : "Center"));
		telemetry.update();

		while(wallLane ? gamepad1.a : gamepad1.y) {

		}

		telemetry.addLine("Park on Lane: (A)Wall (Y)Center");
		telemetry.update();

		boolean parkOnWall = true;
		while(!(gamepad1.a || gamepad1.y) && !isStopRequested()) {

		}

		if(gamepad1.a) {
			parkOnWall = true;
		} else if(gamepad1.y) {
			parkOnWall = false;
		}
		telemetry.clear();
		telemetry.addLine("Park on Lane " + (parkOnWall ? "Wall" : "Center"));
		telemetry.update();

		while(parkOnWall ? gamepad1.a : gamepad1.y) {

		}

		telemetry.addLine("Move Platform: (A)Yes (B)No");
		telemetry.update();

		boolean movePlatform = true;


		while(!(gamepad1.a || gamepad1.b) && !isStopRequested()) {

		}

		if(gamepad1.a) {
			movePlatform = true;
		} else if(gamepad1.b) {
			movePlatform = false;
		}

		telemetry.clear();
		telemetry.addLine((blueTeam ? "Blue" : "Red") + " Team");
		telemetry.addLine("Line on " + (left ? "Left" : "Right"));
		telemetry.addLine((wallLane ? "Wall" : "Center") + " Lane");
		telemetry.addLine("Park in " + (wallLane ? "Wall" : "Center") + " Lane");
		telemetry.addLine((movePlatform ? "WILL" : "NOT") + " Move Foundation");
		telemetry.addLine("Initialization Done. Ready for Start");
		telemetry.update();

		colorDown.enableLed(true);
		waitForStart();
		phaseTime.reset();
		while(opModeIsActive()) {

			float x = 0;
			float y = 0;
			float rot = 0;
			
			float intL = 0;
			float intR = 0;

				if(autoPhase == 0) {
					if(phaseTime.milliseconds() > 15000) {
						autoPhase = 7; // TODO phase that parks
					}
					Recognition rec = vuf.findStone();
					if(rec == null) {
						telemetry.addLine("NO BLOCK FOUND");
						telemetry.update();
						leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
					} else {
						float angle = vuf.getAngle(rec);
						if(rec.getHeight() / rec.getImageHeight() > 0.6f)
							y = -0.4f;
						else y = -1f;
						x = 0;
						rot = angle*2;
						// TODO reverse L using distance sensors
						intL = (1);
						intR = (1);
						if(true) {
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
				if(autoPhase == 0) {
					if(!wallLane) {
						if(left) x = -0.7f;
						else x = 0.7f;
					}
					y = 0;
					phaseTime.reset();
					autoPhase = 1;
					leds.setPattern(blueTeam ? BLUE : RED);
				} else if(autoPhase == 1) {
					if(phaseTime.milliseconds() > 2000) {
						x = 0;
						phaseTime.reset();
						autoPhase = 2;
					} else if(!wallLane) {
						if(left) x = -0.7f;
						else x = 0.7f;
					}
					else x = 0;

				} else if(autoPhase == 2) {
					y = 0.5f;

					if(phaseTime.milliseconds() > 2000) {
						autoPhase = 3;
						phaseTime.reset();
						y = 0;
						leds.setPattern(LIME);
					}
				}else if(autoPhase == 3) {
					if(wallLane != parkOnWall) {
						leds.setPattern(blueTeam ? STROBE_RED : STROBE_BLUE);
						if(parkOnWall) {
							if(left) x = 0.5f;
							else x = -0.5f;
							phaseTime.reset();
							autoPhase = 4;
						} else {
							if(left) x = -0.5f;
							else x = 0.5f;
							phaseTime.reset();
							autoPhase = 4;
						}
					} else autoPhase = 5;
				} else if(autoPhase == 4) {
					if(phaseTime.milliseconds() > 2000) {
						y = 0;
						x = 0;
						autoPhase = 5;
					} else {
						if(parkOnWall) {
							if(left) x = 0.5f;
							else x = -0.5f;
						} else {
							if(left) x = -0.5f;
							else x = 0.5f;
						}
					}
				} else if(autoPhase == 5) {
					leds.setPattern(STROBE_WHITE);
					phaseTime.reset();
					autoPhase = 6;
				} else if(autoPhase == 6) {
					if(phaseTime.milliseconds() > 3000) {
						autoPhase = 7;
						leds.setPattern(RAINBOW_RAINBOW_PALETTE);
					}
				}


			motorIntakeL.setPower(intL);
			motorIntakeR.setPower(intR);
			telemetry.addData("Color", colorDown.red() + " : " + colorDown.blue());
			telemetry.update();
			mec.setVelocity(x, y, rot);
		}
	}
}
