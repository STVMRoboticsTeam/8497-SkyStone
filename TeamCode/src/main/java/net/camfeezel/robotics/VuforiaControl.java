package net.camfeezel.robotics;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VuforiaControl {


	public static final float mmPerInch        = 25.4f;
	private static final float mmTargetHeight   = (6) * mmPerInch;
	private static final float stoneZ = 2.00f * mmPerInch;
	private static final float bridgeZ = 6.42f * mmPerInch;
	private static final float bridgeY = 23 * mmPerInch;
	private static final float bridgeX = 5.18f * mmPerInch;
	private static final float bridgeRotY = 59;
	private static final float bridgeRotZ = 180;
	private static final float halfField = 72 * mmPerInch;
	private static final float quadField  = 36 * mmPerInch;

	private VuforiaLocalizer vuforia = null;

	private List<VuforiaTrackable> allTrackables;
	private VuforiaTrackables targetsSkyStone;

	private Telemetry telemetry;

	public VuforiaControl(Telemetry telemetry, WebcamName webcamName,
						  int cameraMonitorViewId, final String VUFORIA_KEY,
						  final float CAMERA_FORWARD_DISPLACEMENT, final float CAMERA_VERTICAL_DISPLACEMENT,
						  final float CAMERA_LEFT_DISPLACEMENT, float phoneXRotate,
						  float phoneYRotate, float phoneZRotate) {
		this.telemetry = telemetry;
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		parameters.cameraName = webcamName;

		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

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

		allTrackables = new ArrayList<VuforiaTrackable>();
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

		OpenGLMatrix robotFromCamera = OpenGLMatrix
				.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

		/**  Let all the trackable listeners know where the phone is.  */
		for (VuforiaTrackable trackable : allTrackables) {
			((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
		}

		targetsSkyStone.activate();
	}

	/**
	 Only use if multiple re-activations
	 */
	public void activate() {
		targetsSkyStone.activate();
	}

	public void deactivate() {
		targetsSkyStone.deactivate();
	}

	public List<VuforiaTrackable> getAllTrackables() {
		return allTrackables;
	}


}
