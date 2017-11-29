package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot_ {
    public DcMotor left_drive, left_waist, left_shoulder, left_elbow;
    public Servo left_wrist, left_left_claw, left_right_claw;
    public DcMotor right_drive, right_waist, right_shoulder, right_elbow;
    public Servo right_wrist, right_left_claw, right_right_claw;

    public Robot_(HardwareMap hm, DcMotor.RunMode armRunMode){
        left_drive = hm.get(DcMotor.class, "left_drive");
        left_waist = hm.get(DcMotor.class, "left_waist");
        left_shoulder = hm.get(DcMotor.class, "left_shoulder");
        left_elbow = hm.get(DcMotor.class, "left_elbow");

        left_wrist = hm.get(Servo.class, "left_wrist");
        left_left_claw = hm.get(Servo.class, "left_left_claw");
        left_right_claw = hm.get(Servo.class, "left_right_claw");

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_waist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_waist.setMode(armRunMode);
        left_shoulder.setMode(armRunMode);
        left_elbow.setMode(armRunMode);

        left_waist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        right_drive = hm.get(DcMotor.class, "right_drive");
        right_waist = hm.get(DcMotor.class, "right_waist");
        right_shoulder = hm.get(DcMotor.class, "right_shoulder");
        right_elbow = hm.get(DcMotor.class, "right_elbow");

        right_wrist = hm.get(Servo.class, "right_wrist");
        right_left_claw = hm.get(Servo.class, "right_left_claw");
        right_right_claw = hm.get(Servo.class, "right_right_claw");

        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_waist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_waist.setMode(armRunMode);
        right_shoulder.setMode(armRunMode);
        right_elbow.setMode(armRunMode);

        left_waist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        right_drive.setDirection(DcMotor.Direction.REVERSE);
        right_waist.setDirection(DcMotor.Direction.REVERSE);
        right_shoulder.setDirection(DcMotor.Direction.REVERSE);
        right_elbow.setDirection(DcMotor.Direction.REVERSE);

        left_left_claw.setDirection(Servo.Direction.REVERSE);
        right_left_claw.setDirection(Servo.Direction.REVERSE);
    }
}
