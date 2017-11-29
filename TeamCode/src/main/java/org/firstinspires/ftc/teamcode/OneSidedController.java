package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class OneSidedController {
    private double servoMin = .3, servoMid = .42, servoMax = .95;
    private Robot_ r;
    private Gamepad g;
    @SuppressWarnings("unused")
    private Telemetry telemetry;
    public NewArmMath3 armMath;
    private DcMotor[] armMotors;
    private Servo[] servos;
    private boolean onRightSide;
    public OneSidedController(boolean onRightSide, Robot_ robot, Gamepad gamepad, Telemetry telemetry){
        r=robot; g=gamepad; this.telemetry=telemetry; this.onRightSide=onRightSide;
        armMath = new NewArmMath3(onRightSide);
        if(!onRightSide){
            armMotors = new DcMotor[]{r.left_waist, r.left_shoulder, r.left_elbow};
            servos = new Servo[]{r.left_wrist, r.left_left_claw, r.left_right_claw};
        }else{
            armMotors = new DcMotor[]{r.right_waist, r.right_shoulder, r.right_elbow};
            servos = new Servo[]{r.right_wrist, r.right_left_claw, r.right_right_claw};
        }
        
        g.setJoystickDeadzone(0);
        armMotors[0].setPower(1);//TODO try deleting these three lines
        armMotors[1].setPower(1);
        armMotors[2].setPower(1);

        armMotors[0].setTargetPosition((int)Math.round(armMath.waist));
        armMotors[1].setTargetPosition((int)Math.round(armMath.shoulder));
        armMotors[2].setTargetPosition((int)Math.round(armMath.elbow));
        r.left_drive.setPower(0);
        r.right_drive.setPower(0);
        servos[0].setPosition(armMath.wrist);
        telemetry.update();
        servos[1].setPosition(servoMin);
        servos[2].setPosition(servoMin);
    }

    boolean[] lastDown = new boolean[4];
    boolean drive = false;//It's tempting to change this: Don't. (because the Joysticks won't zero etc.)
    public void tick(){
        boolean[] down = {g.y, g.b, g.a, g.x};
        boolean[] pressed = new boolean[4];
        for(int i = 0; i < 4; i++){pressed[i] = down[i] && ! lastDown[i];}
        lastDown = down;
        double[] input = {g.left_stick_x, g.left_stick_y, g.right_stick_x, g.right_stick_y};

        if (pressed[0])
            drive = !drive;
        if (pressed[2])
            armMath.resetWrist();
            //armMath.setPosition(new double[]{5*(onRightSide?-1:1), 5, -55});
        if (pressed[1])
            armMath.zeroJoystick(input);
        if (pressed[3])
            armMath.resetPosition();

        if(drive){
            r.left_drive.setPower(-(g.left_stick_y-armMath.joystickZero[1]));
            r.right_drive.setPower(-(g.right_stick_y-armMath.joystickZero[3]));
        }else{
            armMath.update(input, telemetry);
            armMotors[0].setTargetPosition((int)Math.round(armMath.waist));
            armMotors[1].setTargetPosition((int)Math.round(armMath.shoulder));
            armMotors[2].setTargetPosition((int)Math.round(armMath.elbow));
            servos[0].setPosition(armMath.wrist);
            r.left_drive.setPower(0);
            r.right_drive.setPower(0);
        }
        if(g.left_trigger > .8 || g.right_trigger > .8 || g.left_bumper || g.right_bumper){
            servos[1].setPosition(servoMax);
            servos[2].setPosition(servoMax);
        }else if(g.left_trigger > .05 || g.right_trigger > .05){
            servos[1].setPosition(servoMid);//Left
            servos[2].setPosition(servoMid);//Right
        }else{
            servos[1].setPosition(servoMin);
            servos[2].setPosition(servoMin);
        }
    }
}
