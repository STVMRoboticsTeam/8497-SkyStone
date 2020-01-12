package net.camfeezel.robotics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.Pif;
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


	private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
	private static final String LABEL_FIRST_ELEMENT = "Stone";
	private static final String LABEL_SECOND_ELEMENT = "Skystone";

	private VuforiaLocalizer vuforia = null;

	private List<VuforiaTrackable> allTrackables;
	private VuforiaTrackables targetsSkyStone;

	private TFObjectDetector tfod;

	private Telemetry telemetry;
	private HardwareMap hardwareMap;

	public VuforiaControl(Telemetry telemetry, HardwareMap hardwareMap,
						  final String VUFORIA_KEY,
						  final float CAMERA_FORWARD_DISPLACEMENT, final float CAMERA_VERTICAL_DISPLACEMENT,
						  final float CAMERA_LEFT_DISPLACEMENT, float phoneXRotate,
						  float phoneYRotate, float phoneZRotate) {
		this.telemetry = telemetry;
		this.hardwareMap = hardwareMap;
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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

		if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
			initTfod();
		} else {
			telemetry.addData("Sorry!", "This device is not compatible with TFOD");
		}

		if(tfod != null) {
			tfod.activate();
		}
	}

	private Recognition curStone = null;

	public Recognition findStone() {
		if (tfod != null) {
			// getUpdatedRecognitions() will return null if no new information is available since
			// the last time that call was made.
			List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
			if (updatedRecognitions != null) {

				// step through the list of recognitions and display boundary info.
				int i = 0;
				for (Recognition recognition : updatedRecognitions) {
					curStone = recognition;
					return recognition;
				}
			} else {
				return curStone;
			}
		}
		return null;
	}

	public void clearCurrentStone() {
		curStone = null;
	}

	/**
	 * Positive is turn to the right.
	 * @param rec
	 * @return
	 */
	public float getAngle(Recognition rec) {
		float l = rec.getLeft();
		float r = rec.getRight();
		float hr = rec.getHeight() / rec.getImageHeight();
		if(hr > 0.8) {
			return 0;
		}
		float iw = rec.getImageWidth();
		float w = rec.getWidth();
		if(w / iw > 0.85) {
			return 0;
		}
		float rem = iw - w;
		float evenRatio = rem / 2.0f;
		if(l > evenRatio && (r-evenRatio / evenRatio > 0.05f)) {
			return ((l-evenRatio)/iw)*75f;
		} else if(r > evenRatio && (r-evenRatio / evenRatio > 0.05f)) {
			return ((r-evenRatio)/iw)*75f;
		} else {
			return 0;
		}
	}

	/**
	 Only use if multiple re-activations
	 */
	public void activate() {

		targetsSkyStone.activate();
		if(tfod != null) tfod.activate();
	}

	public void deactivate() {

		targetsSkyStone.deactivate();
		if(tfod != null) tfod.shutdown();
	}

	public List<VuforiaTrackable> getAllTrackables() {
		return allTrackables;
	}

	private void initTfod() {
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfodParameters.minimumConfidence = 0.8;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
	}

}
