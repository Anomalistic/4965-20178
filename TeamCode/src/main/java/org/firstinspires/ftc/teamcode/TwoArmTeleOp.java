package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class TwoArmTeleOp extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        Robot_ r = new Robot_(hardwareMap, DcMotor.RunMode.RUN_TO_POSITION);
        OneSidedController left = new OneSidedController(false, r, gamepad1, telemetry),
                right = new OneSidedController(true, r, gamepad2, telemetry);
        StallDetector stallDetector = new StallDetector(hardwareMap);
        
        waitForStart();


        //long i = 0, t = System.currentTimeMillis();
        while (opModeIsActive()) {
            left.tick();
            right.tick();
            //stallDetector.tick();
            telemetry.addData("Waist", String.format("%4d %4d", r.left_waist.getTargetPosition()-r.left_waist.getCurrentPosition(),  r.right_waist.getTargetPosition()-r.right_waist.getCurrentPosition()));
            telemetry.addData("Shldr", String.format("%4d %4d", r.left_shoulder.getTargetPosition()-r.left_shoulder.getCurrentPosition(),  r.right_shoulder.getTargetPosition()-r.right_shoulder.getCurrentPosition()));
            telemetry.addData("Elbow", String.format("%4d %4d", r.left_elbow.getTargetPosition()-r.left_elbow.getCurrentPosition(),  r.right_elbow.getTargetPosition()-r.right_elbow.getCurrentPosition()));

            telemetry.addData("Waist", String.format("%4d %4d", r.left_waist.getCurrentPosition(),  r.right_waist.getCurrentPosition()));
            telemetry.addData("Shldr", String.format("%4d %4d", r.left_shoulder.getCurrentPosition(),  r.right_shoulder.getCurrentPosition()));
            telemetry.addData("Elbow", String.format("%4d %4d", r.left_elbow.getCurrentPosition(),  r.right_elbow.getCurrentPosition()));

            telemetry.addData("X", String.format("%.3f %.3f", left.armMath.x, right.armMath.x));
            telemetry.addData("Y", String.format("%.3f %.3f", left.armMath.y, right.armMath.y));
            telemetry.addData("Z", String.format("%.3f %.3f", left.armMath.z, right.armMath.z));
            telemetry.update();
            
            /*i++;
            if (System.currentTimeMillis() > t+200)
            {
                telemetry.addData("ticks/second", String.format("%6d",Math.round(1000*i/(System.currentTimeMillis()-t))));
                telemetry.update();
                t = System.currentTimeMillis();
                i = 0;
            }*/
        }
    }
}