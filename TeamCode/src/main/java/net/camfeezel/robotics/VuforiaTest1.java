package net.camfeezel.robotics;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(group = "Webcam Tests", name = "01-Skystone")
public class VuforiaTest1 extends LinearOpMode {

	private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
	private static final boolean PHONE_IS_PORTRAIT = false  ;

	private static final String VUFORIA_KEY =
			"ASSzGtr/////AAABmZbPFu6zT0zIvBJNI9BxnCh58m/JkUORiHZOzTsf6RujF/GjGAERY9IEnRhkjoOmbTSQVldTUmKZBk3qEzxlXmwISSg7cbapxP+1k7+0kY9g1itZHc1PxwjSC+nJuP3Ua3/qdtKfRYbgBeJOS4h55ajSCPEy9+7Y7fgRcKcVC/bvW+bPukTpVB7LWBCqmLs0giRUc6SXBTUgaMyBcgkZEYTauqo9lUkxpgQyLfZXz6Ozs3c6D0FNL3q8XTP2WptVWM16/8VsDTZePEH9GGHYG8XiFdGgB7cXePoqe3EJfXY0SRjRQYjzdn32FEwVvM3lEsr9S3vGW9vZ1l4DDJAvmDka3eioBD5nd9yLAyvVFenk";

	private static final float mmPerInch        = 25.4f;
	private static final float mmTargetHeight   = (6) * mmPerInch;
	private static final float stoneZ = 2.00f * mmPerInch;
	private static final float bridgeZ = 6.42f * mmPerInch;
	private static final float bridgeY = 23 * mmPerInch;
	private static final float bridgeX = 5.18f * mmPerInch;
	private static final float bridgeRotY = 59;
	private static final float bridgeRotZ = 180;
	private static final float halfField = 72 * mmPerInch;
	private static final float quadField  = 36 * mmPerInch;

	private OpenGLMatrix lastLocation = null;
	private VuforiaLocalizer vuforia = null;

	WebcamName webcamName = null;
	private boolean targetVisible = false;
	private float phoneXRotate    = 0;
	private float phoneYRotate    = 0;
	private float phoneZRotate    = 0;


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

		webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		parameters.cameraName = webcamName;

		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

		VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
		stoneTarget.setName("Stone Target");
		VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
		blueRearBridge.setName("Blue Rear Bridge");
		VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
		redRearBridge.setName("Red Rear Bridge");
		VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
		redFrontBridge.setName("Red Front Bridge");
		VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
		blueFrontBridge.setName("Blue Front Bridge");
		VuforiaTrackable red1 = targetsSkyStone.get(5);
		red1.setName("Red Perimeter 1");
		VuforiaTrackable red2 = targetsSkyStone.get(6);
		red2.setName("Red Perimeter 2");
		VuforiaTrackable front1 = targetsSkyStone.get(7);
		front1.setName("Front Perimeter 1");
		VuforiaTrackable front2 = targetsSkyStone.get(8);
		front2.setName("Front Perimeter 2");
		VuforiaTrackable blue1 = targetsSkyStone.get(9);
		blue1.setName("Blue Perimeter 1");
		VuforiaTrackable blue2 = targetsSkyStone.get(10);
		blue2.setName("Blue Perimeter 2");
		VuforiaTrackable rear1 = targetsSkyStone.get(11);
		rear1.setName("Rear Perimeter 1");
		VuforiaTrackable rear2 = targetsSkyStone.get(12);
		rear2.setName("Rear Perimeter 2");

		List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
		allTrackables.addAll(targetsSkyStone);

		// Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
		// Rotated it to to face forward, and raised it to sit on the ground correctly.
		// This can be used for generic target-centric approach algorithms
		stoneTarget.setLocation(OpenGLMatrix
				.translation(0, 0, stoneZ)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

		//Set the position of the bridge support targets with relation to origin (center of field)
		blueFrontBridge.setLocation(OpenGLMatrix
				.translation(-bridgeX, bridgeY, bridgeZ)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

		blueRearBridge.setLocation(OpenGLMatrix
				.translation(-bridgeX, bridgeY, bridgeZ)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

		redFrontBridge.setLocation(OpenGLMatrix
				.translation(-bridgeX, -bridgeY, bridgeZ)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

		redRearBridge.setLocation(OpenGLMatrix
				.translation(bridgeX, -bridgeY, bridgeZ)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

		//Set the position of the perimeter targets with relation to origin (center of field)
		red1.setLocation(OpenGLMatrix
				.translation(quadField, -halfField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

		red2.setLocation(OpenGLMatrix
				.translation(-quadField, -halfField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

		front1.setLocation(OpenGLMatrix
				.translation(-halfField, -quadField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

		front2.setLocation(OpenGLMatrix
				.translation(-halfField, quadField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

		blue1.setLocation(OpenGLMatrix
				.translation(-quadField, halfField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

		blue2.setLocation(OpenGLMatrix
				.translation(quadField, halfField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

		rear1.setLocation(OpenGLMatrix
				.translation(halfField, quadField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

		rear2.setLocation(OpenGLMatrix
				.translation(halfField, -quadField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

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
		final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
		final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
		final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

		OpenGLMatrix robotFromCamera = OpenGLMatrix
				.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

		/**  Let all the trackable listeners know where the phone is.  */
		for (VuforiaTrackable trackable : allTrackables) {
			((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
		}





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

		targetsSkyStone.activate();
		while (!isStopRequested()) {

			float x = 0;
			float y = 0;
			float rot = 0;

			// check all the trackable targets to see which one (if any) is visible.
			targetVisible = false;
			for (VuforiaTrackable trackable : allTrackables) {
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
						translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

				// express the rotation of the robot in degrees.
				Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
				telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
			}
			else {
				telemetry.addData("Visible Target", "none");
			}

			telemetry.addLine("Velocity").addData("X", x).addData("Y", y).addData("ROT", rot);
			mec.setVelocity(x, y, rot);
			telemetry.update();
		}

		// Disable Tracking when we are done;
		targetsSkyStone.deactivate();
    }

	String formatAngle(AngleUnit angleUnit, double angle) {
		return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
	}

	String formatDegrees(double degrees){
		return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
	}

}
