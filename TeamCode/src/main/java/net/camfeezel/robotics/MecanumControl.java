package net.camfeezel.robotics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class MecanumControl {

    private DcMotor motorFL0;
    private DcMotor motorFR1;
    private DcMotor motorBL2;
    private DcMotor motorBR3;
    private DcMotor motorOdomX;
    private DcMotor motorOdomY;
    private Telemetry telemetry;

	public MecanumControl(DcMotor motorFL0, DcMotor motorFR1,
						  DcMotor motorBL2, DcMotor motorBR3,
						  DcMotor motorOdomX, DcMotor motorOdomY,
						  Telemetry telemetry) {
		this.motorFL0 = motorFL0;
		this.motorFR1 = motorFR1;
		this.motorBL2 = motorBL2;
		this.motorBR3 = motorBR3;
		this.motorOdomX = motorOdomX;
		this.motorOdomY = motorOdomY;
		this.telemetry = telemetry;

		this.motorOdomX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		this.motorOdomY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	}

	public MecanumControl(DcMotor motorFL0, DcMotor motorFR1,
						  DcMotor motorBL2, DcMotor motorBR3,
						  Telemetry telemetry) {
		this.motorFL0 = motorFL0;
		this.motorFR1 = motorFR1;
		this.motorBL2 = motorBL2;
		this.motorBR3 = motorBR3;
		this.telemetry = telemetry;
	}

    /**
     *
     * @param x speed in the 90/270 degrees direction. -1 to 1
     * @param y speed in the 0/180 degrees direction. -1 to 1
     * @param rot rotational speed -180 to 180, positive means positive degrees.
     */
    public void setVelocity(float x, float y, float rot) {
        float frFin = 0f;
        float flFin = 0f;
        float brFin = 0f;
        float blFin = 0f;

        x = Range.clip(x + (Math.signum(x) * 0.06f), -1f, 1f);
        y = Range.clip(y + (Math.signum(y) * 0.06f), -1f, 1f);
        rot = Range.clip((rot / 180) + (Math.signum(rot) * 0.06f), -1f, 1f);

        /*
         * fl = x + y + rot
         * fr = x - y - rot
         * bl = x - y + rot
         * br = x + y - rot
         *
         * final values need scaled // not clipped
         * scale rotation less than position
         */
        float curScale = 1f;
        float scale = 1f;

        // FL x - y + rot
        if(x - y + rot > 1f) {
            curScale = (1 - rot) / (x - y);
        } else if(x - y + rot < -1f) {
            curScale = (rot - 1) / (x - y);
        }
        if(curScale < scale) scale = curScale;

        // FR x + y - rot
        if(x + y - rot > 1f) {
            curScale = (1 + rot) / (x + y);
        } else if(x + y - rot < -1f) {
            curScale = (-rot - 1) / (x + y);
        }
        if(curScale < scale) scale = curScale;

        // BL x + y + rot
        if(x + y + rot > 1f) {
            curScale = (1 - rot) / (x + y);
        } else if(x + y + rot < -1f) {
            curScale = (rot - 1) / (x + y);
        }
        if(curScale < scale) scale = curScale;

        // BR x - y - rot
        if(x - y - rot > 1f) {
            curScale = (1 + rot) / (x - y);
        } else if(x - y - rot < -1f) {
            curScale = (-rot - 1) / (x - y);
        }
        if(curScale < scale) scale = curScale;

        flFin = (x - y) * scale + rot;
        frFin = (x + y) * scale + rot;
        blFin = (x + y) * scale - rot;
        brFin = (x - y) * scale - rot;

//        telemetry.addLine("Motors")
//                .addData("FL", flFin)
//                .addData("FR", -frFin)
//                .addData("BL", blFin)
//                .addData("BR", -brFin);

        motorFL0.setPower(flFin);
        motorFR1.setPower(frFin);
        motorBL2.setPower(-blFin);
        motorBR3.setPower(-brFin);
    }

    public void setDriveMode(DcMotor.RunMode driveMode) {
    	motorFL0.setMode(driveMode);
    	motorFR1.setMode(driveMode);
    	motorBL2.setMode(driveMode);
    	motorBR3.setMode(driveMode);
	}

	public static final double WHEEL_CIRCUMFERENCE = Math.PI * 4.0;

	/**
	 * Total Distance from last call of resetEncoders
	 */
	public VectorF getDistanceTravelled() {
		motorOdomX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motorOdomY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		return new VectorF(motorOdomX.getCurrentPosition(), motorOdomY.getCurrentPosition());
	}

	public void resetDistanceTracking() {
		motorOdomX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorOdomY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

}
