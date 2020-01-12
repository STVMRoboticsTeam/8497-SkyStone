package net.camfeezel.robotics;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(group = "Webcam Tests", name = "01-Skystone")
public class VuforiaTest2 extends LinearOpMode {

	private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
	private static final boolean PHONE_IS_PORTRAIT = false;

	private static final String VUFORIA_KEY = //region
			"ASSzGtr/////AAABmZbPFu6zT0zIvBJNI9BxnCh58m/JkUORiHZOzTsf6RujF/GjGAERY9IEnRhkjoOmbTSQVldTUmKZBk3qEzxlXmwISSg7cbapxP+1k7+0kY9g1itZHc1PxwjSC+nJuP3Ua3/qdtKfRYbgBeJOS4h55ajSCPEy9+7Y7fgRcKcVC/bvW+bPukTpVB7LWBCqmLs0giRUc6SXBTUgaMyBcgkZEYTauqo9lUkxpgQyLfZXz6Ozs3c6D0FNL3q8XTP2WptVWM16/8VsDTZePEH9GGHYG8XiFdGgB7cXePoqe3EJfXY0SRjRQYjzdn32FEwVvM3lEsr9S3vGW9vZ1l4DDJAvmDka3eioBD5nd9yLAyvVFenk";
		//endregion


	WebcamName webcamName = null;
	private boolean targetVisible = false;
	private float phoneXRotate    = 0;
	private float phoneYRotate    = 0;
	private float phoneZRotate    = 0;
	private OpenGLMatrix lastLocation = null;


	private Rev2mDistanceSensor sensorDistance00;

    private RevColorSensorV3 sensorColorDown;

    private BNO055IMU sensorGyro;
	private Orientation angles = new Orientation();

    private DcMotor motorFL0;
    private DcMotor motorFR1;
    private DcMotor motorBL2;
    private DcMotor motorBR3;
    private MecanumControl mec;
    private VuforiaControl vuf;

    private ElapsedTime timer1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        sensorDistance00 = hardwareMap.get(Rev2mDistanceSensor.class, "dist00");

        sensorColorDown = hardwareMap.get(RevColorSensorV3.class, "colorDown");

        motorFL0 = hardwareMap.dcMotor.get("0");
        motorFR1 = hardwareMap.dcMotor.get("1");
        motorBL2 = hardwareMap.dcMotor.get("2");
        motorBR3 = hardwareMap.dcMotor.get("3");
        mec = new MecanumControl(motorFL0, motorFR1, motorBL2, motorBR3, telemetry);

		webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

		// We need to rotate the camera around it's long axis to bring the correct camera forward.
		if (CAMERA_CHOICE == BACK) {
			phoneYRotate = -90;
		} else {
			phoneYRotate = 90;
		}

		// Rotate the phone vertical about the X axis if it's in portrait mode
		if (PHONE_IS_PORTRAIT) {
			phoneXRotate = 90 ;
		}

		// Next, translate the camera lens to where it is on the robot.
		// In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
		float cameraForward = 4.0f ;   // eg: Camera is 4 Inches in front of robot-center
		float cameraVertical = 8.0f;   // eg: Camera is 8 Inches above ground
		float cameraLeft = 0;     // eg: Camera is ON the robot's center line

		vuf = new VuforiaControl(telemetry, webcamName, cameraMonitorViewId, VUFORIA_KEY,
				cameraForward, cameraVertical, cameraLeft, phoneXRotate, phoneYRotate, phoneZRotate);


		// GYRO Init and Calibration
		BNO055IMU.Parameters gyroparameters = new BNO055IMU.Parameters();
		gyroparameters.mode                = BNO055IMU.SensorMode.IMU;
		gyroparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
		gyroparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		gyroparameters.loggingEnabled      = false;

        sensorGyro = hardwareMap.get(BNO055IMU.class, "imu");
        telemetry.log().add("DO NOT MOVE ROBOT! Gyro Calibrating...");
        sensorGyro.initialize(gyroparameters);
        timer1.reset();
        while (!isStopRequested() && !sensorGyro.isGyroCalibrated())  {
            sleep(50);
            idle();
        }
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibration status (" + timer1.seconds() + "s) " + sensorGyro.getCalibrationStatus().toString());
        telemetry.clear(); telemetry.update();
        waitForStart();
        telemetry.log().clear();
		sensorGyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

		while (!isStopRequested()) {

			float x = 0;
			float y = 0;
			float rot = 0;

			// check all the trackable targets to see which one (if any) is visible.
			targetVisible = false;
			for(VuforiaTrackable trackable : vuf.getAllTrackables()) {
				if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
					telemetry.addData("Visible Target", trackable.getName());
					targetVisible = true;

					// getUpdatedRobotLocation() will return null if no new information is available since
					// the last time that call was made, or if the trackable is not currently visible.
					OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
					if (robotLocationTransform != null) {
						lastLocation = robotLocationTransform;
					}
					break;
				}
			}

			// Provide feedback as to where the robot is located (if we know).
			if (targetVisible) {
				// express position (translation) of robot in inches.
				VectorF translation = lastLocation.getTranslation();
				telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
						translation.get(0) / VuforiaControl.mmPerInch, translation.get(1) / VuforiaControl.mmPerInch, translation.get(2) / VuforiaControl.mmPerInch);

				// express the rotation of the robot in degrees.
				Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
				telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
			}
			else {
				telemetry.addData("Visible Target", "none");
			}

			if(lastLocation != null) {
				mec.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
				VectorF translation = lastLocation.getTranslation();
				float mx = translation.get(0);
				if(mx > 12 || mx < 8) {
					float dx = 10 - mx;
					if(dx > 40) y = 1f;
					else y = dx/40;
				}
				float mz = translation.get(2);
				if(mz > 12 || mz < 8) {
					float dz = 10 - mz;
					if(dz > 30) x = 1f;
					else x = dz/40;
				}

			}

			telemetry.addLine("Velocity").addData("X", x).addData("Y", y).addData("ROT", rot);
			mec.setVelocity(x, y, rot);
			telemetry.update();
		}

		// Disable Tracking when we are done;
		vuf.deactivate();
    }

	String formatAngle(AngleUnit angleUnit, double angle) {
		return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
	}

	String formatDegrees(double degrees){
		return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
	}

}
