package net.camfeezel.robotics;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "GetOOFCheese")
public class GetOOFCheese extends LinearOpMode {

	private static final float mmPerInch        = 25.4f;


	private DcMotor motorFL0;
	private DcMotor motorFR1;
	private DcMotor motorBL2;
	private DcMotor motorBR3;

	private DcMotor motorIntakeL;
	private DcMotor motorIntakeR;

	private RevBlinkinLedDriver leds;

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

		leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");

		motorSlideLat = hardwareMap.dcMotor.get("slideLat");
		motorSlideVert = hardwareMap.dcMotor.get("slideVert");
		servoBlock0 = hardwareMap.servo.get("hook");

		motorSlideLat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorSlideVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		MecanumControl mec = new MecanumControl(motorFL0, motorFR1, motorBL2, motorBR3, telemetry);

		final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
		final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
		final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

		VuforiaControl vuf = new VuforiaControl(telemetry, hardwareMap, VUFORIA_KEY, CAMERA_FORWARD_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, -90, -90, 0);

		boolean autoMode = false;
		int autoPhase = 0;
		ElapsedTime phaseTime = new ElapsedTime();
		telemetry.clear();

		blueTeam = true;

		telemetry.addLine("Team Selected: " + (blueTeam ? "Blue" : "Red"));
		telemetry.addLine("Initialization Done. Ready for Start");
		telemetry.update();

		waitForStart();

		motorSlideVert.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motorSlideLat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		while(opModeIsActive()) {
			float x = 0;
			float y = 0;
			float rot = 0;

			if(autoPhase == 0) {
				autoPhase = 1;
				phaseTime.reset();
			}
			else if(autoPhase == 1) {
				if(phaseTime.milliseconds() < 1000) {
					y = 0.5f;
				} else {
					y = 0;
					autoPhase = 2;
					phaseTime.reset();
				}
			} else if(autoPhase == 2) {
				if(phaseTime.milliseconds() < 2000) {
					leds.setPattern(STROBE_GOLD);
				} else {
					leds.setPattern(RAINBOW_RAINBOW_PALETTE);
				}
			}

			telemetry.update();
			mec.setVelocity(x, y, rot);
		}
	}
}
