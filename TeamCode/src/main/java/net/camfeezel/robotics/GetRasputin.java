package net.camfeezel.robotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "GetRasputin")
public class GetRasputin extends LinearOpMode {

	private DcMotor motorFL0;
	private DcMotor motorFR1;
	private DcMotor motorBL2;
	private DcMotor motorBR3;

	// Negative: Out
	private DcMotor motorSlide0;
	// Positive: Up
	private DcMotor motorPivot2;

	private Servo servoBlock0;

	@Override
	public void runOpMode() throws InterruptedException {
		motorFL0 = hardwareMap.dcMotor.get("0");
		motorFR1 = hardwareMap.dcMotor.get("1");
		motorBL2 = hardwareMap.dcMotor.get("2");
		motorBR3 = hardwareMap.dcMotor.get("3");

		motorSlide0 = hardwareMap.dcMotor.get("slide");
		motorPivot2 = hardwareMap.dcMotor.get("pivot");
		servoBlock0 = hardwareMap.servo.get("hook");

		MecanumControl mec = new MecanumControl(motorFL0, motorFR1, motorBL2, motorBR3, telemetry);

		waitForStart();

		while(opModeIsActive()) {
			float lx = gamepad1.left_stick_x;
			float ly = gamepad1.left_stick_y;
			float rx = gamepad1.right_stick_x;

			float x = 0;
			float y = 0;
			float rot = 0;

			if(Math.abs(lx) > 0.05f) x = (float) Math.pow(lx, 3);
			if(Math.abs(ly) > 0.05f) y = (float) Math.pow(ly, 3);
			if(Math.abs(rx) > 0.05f) rot = rx*180;

			mec.setVelocity(x, y, rot);

			float lt = gamepad1.left_trigger;
			float rt = gamepad1.right_trigger;

			if(lt > 0.02f) motorPivot2.setPower(-lt);
			if(rt > 0.02f) motorPivot2.setPower(rt);
			if(gamepad1.a) motorSlide0.setPower(0.3f);
			else if(gamepad1.y) motorSlide0.setPower(-0.3f);
			if(gamepad1.x) servoBlock0.setPosition(1f);
			else if(gamepad1.b) servoBlock0.setPosition(-1f);

		}
	}
}
