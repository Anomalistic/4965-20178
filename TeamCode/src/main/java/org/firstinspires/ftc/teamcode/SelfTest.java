package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SelfTest extends LinearOpMode{

	@Override
	public void runOpMode() throws InterruptedException {
		Robot_ r = new Robot_(hardwareMap, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		waitForStart();
		while(opModeIsActive()){
			r.left_drive.setPower((gamepad1.dpad_up?1:0) - (gamepad1.dpad_down?1:0));
			r.left_waist.setPower(gamepad1.left_stick_x);
			r.left_shoulder.setPower(-gamepad1.left_stick_y);
			r.left_elbow.setPower(-gamepad1.right_stick_y);
			r.left_wrist.setPosition(gamepad1.right_stick_x);
			r.left_left_claw.setPosition(gamepad1.left_trigger);
			r.left_right_claw.setPosition(gamepad1.right_trigger);

			r.right_drive.setPower((gamepad2.dpad_up?1:0) - (gamepad2.dpad_down?1:0));
			r.right_waist.setPower(gamepad2.left_stick_x);
			r.right_shoulder.setPower(-gamepad2.left_stick_y);
			r.right_elbow.setPower(-gamepad2.right_stick_y);
			r.right_wrist.setPosition(gamepad2.right_stick_x);
			r.right_left_claw.setPosition(gamepad2.left_trigger);
			r.right_right_claw.setPosition(gamepad2.right_trigger);

			telemetry.addData("Drive", String.format("%5d %5d", r.left_drive.getCurrentPosition(),  r.right_drive.getCurrentPosition()));
			telemetry.addData("Waist", String.format("%5d %5d", r.left_waist.getCurrentPosition(),  r.right_waist.getCurrentPosition()));
			telemetry.addData("Shldr", String.format("%5d %5d", r.left_shoulder.getCurrentPosition(),  r.right_shoulder.getCurrentPosition()));
			telemetry.addData("Elbow", String.format("%5d %5d", r.left_elbow.getCurrentPosition(),  r.right_elbow.getCurrentPosition()));
			telemetry.update();
		}
	}
}
