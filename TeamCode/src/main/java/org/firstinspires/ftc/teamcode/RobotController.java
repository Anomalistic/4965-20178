package org.firstinspires.ftc.teamcodeV3;

import static java.lang.Math.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotController {
	public RobotSide[] sides;
	public Robot_ r;
	private Gamepad g;//TODO deleteme
	public RobotController(HardwareMap hm, Gamepad gamepad1, Gamepad gamepad2){
		g=gamepad1;//TODO deleteme
		r = new Robot_(hm, DcMotor.RunMode.RUN_TO_POSITION);
		sides = new RobotSide[]{new RobotSide(true, gamepad1), new RobotSide(false, gamepad2)};
	}
	
	public void tick(){
		sides[0].tick();
		sides[1].tick();
	}

	
	private double leftJewelMin = .3, leftJewelMax = .7,  rightJewelMin = .7, rightJewelMax = .3, jewelPos = -1;
	public void setJewelArmTarget(double pos){
		jewelPos = pos;
		r.left_jewel.setPosition(map(new double[]{0, 1, leftJewelMin, leftJewelMax}, pos));
		r.right_jewel.setPosition(map(new double[]{0, 1, rightJewelMin, rightJewelMax}, pos));
	}
	public double getJewelArmTarget(){return jewelPos;}
	public boolean jewelColorIsRed(){//TODO fix
		while(true){
			tick();
			if(g.b)
				return true;
			if(g.x)
				return false;
		}
	}
	
	public static double map(double[] mapping, double x){
		return (x-mapping[0])/(mapping[1]-mapping[0])*(mapping[3]-mapping[2])+mapping[2];
	}
	
	public class RobotSide{
		private double[] minAngles = {75/180.0*PI, 40/180.0*PI, 13.16/180.0*PI};//TODO (re)measure
		private double[] maxAngles = {75/180.0*PI+(3618)/(2*2516/(2*PI)), 40/180.0*PI+(2130)/(4*1256/(2*PI)), PI};//TODO (re)measure
		private double[] ticksPerRadian = {2*2516/(2*PI), 4*1256/(2*PI), 4*795/(2*PI)};//TODO remeasure
		private double horizontalLength = 28.80, armLength = 28.80;//32.00 mm * 9
		private boolean onLeftSide;
		private DcMotor driveMotor;
		private DcMotor[] armMotors;
		private Servo[] servos;
		private Gamepad g;
		public RobotSide(boolean onLeftSide, Gamepad gamepad){//TODO Zero joystick?
			this.onLeftSide = onLeftSide;
			g = gamepad;
			g.setJoystickDeadzone(0);
			if(onLeftSide){
				driveMotor = r.left_drive;
				armMotors = new DcMotor[]{r.left_waist, r.left_shoulder, r.left_elbow};
				servos = new Servo[]{r.left_wrist, r.left_left_claw, r.left_right_claw};
			}else{
				driveMotor = r.right_drive;
				armMotors = new DcMotor[]{r.right_waist, r.right_shoulder, r.right_elbow};
				servos = new Servo[]{r.right_wrist, r.right_left_claw, r.right_right_claw};
			}
			returnToOrigin();
			tick(); tick();//the second one is to calculate speed
		}
		
		private boolean taring;
		private long taringEndTime;
		private double[] armPos = new double[3], analogValues;
		private boolean[] lastButtonStates = new boolean[13], buttonPresses = new boolean[13];
		public void tick(){
			double[] newArmPos = new double[armMotors.length];
			for(int i = 0; i < armMotors.length; i++)
				newArmPos[i] = minAngles[i]+armMotors[i].getCurrentPosition()/ticksPerRadian[i];
			newArmPos = angleToCartesian(newArmPos);
			armPos = newArmPos;
			
			analogValues = new double[]{g.left_stick_x, -g.left_stick_y, g.right_stick_x, -g.right_stick_y, g.left_trigger, g.right_trigger};
			for(int i = 0; i < 4; i++)
				analogValues[i] *= 1+abs(analogValues[i]);
			boolean[] buttonStates = {g.a, g.b, g.x, g.y, g.dpad_right, g.dpad_up, g.dpad_left, g.dpad_down, g.left_bumper, g.right_bumper, g.back, g.start, g.guide};
			for(int i = 0; i < buttonStates.length; i++)
				buttonPresses[i] = buttonStates[i] && !lastButtonStates[i];
			lastButtonStates = buttonStates;
			
			if (taring && System.currentTimeMillis() > taringEndTime){
				for(DcMotor m:armMotors){
					m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
					m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
					m.setPower(armMaxSpeed);
				}
				setTargetPositionFromArmTarget();
				taring = false;
			}
		}
		
		//Target
		
		private double[] armTarget = new double[3];
		public void setArmTarget(double[] xyz){
			System.arraycopy(xyz, 0, armTarget, 0, armTarget.length);
			setTargetPositionFromArmTarget();
		}
		public double[] getArmTarget(){
			double[] out = new double[armTarget.length];
			System.arraycopy(armTarget, 0, out, 0, armTarget.length);
			return out;
		}
		
		private double wristMin = .18, wristMax = .7, wristPos = -1;
		public void setWristTarget(double pos){
			servos[0].setPosition(map(new double[]{0, 1, wristMin, wristMax}, wristPos=pos));
		}
		public double getWristTarget(){return wristPos;}
		
		private double clawPos;
		public void setClawTarget(double pos){
			clawPos = pos;
			servos[1].setPosition(pos);
			servos[2].setPosition(pos);
		}
		public double getClawTarget(){return clawPos;}
		
		//Controller
		
		public double[] getAnalogValues(){
			double[] out = new double[analogValues.length];
			System.arraycopy(analogValues, 0, out, 0, analogValues.length);
			return out;
		}
		public boolean[] getButtonPresses(){
			boolean[] out = new boolean[buttonPresses.length];
			System.arraycopy(buttonPresses, 0, out, 0, buttonPresses.length);
			return out;
		}
	
		//Speed
		
		private double armMaxSpeed = 1;
		public void setArmMaxSpeed(double speed){
			for(DcMotor m:armMotors)
				m.setPower(armMaxSpeed=speed);
		}
		public double getArmMaxSpeed(){return armMaxSpeed;}
	
		private double driveSpeed = 0;
		public void setDriveSpeed(double speed){driveMotor.setPower(driveSpeed=speed);}
		public double getDriveSpeed(){return driveSpeed;}
		
		//Methods
		
		public void returnToOrigin(){setArmTarget(angleToCartesian(minAngles));}
		public void tare(){
			for(DcMotor m:armMotors){
				m.setMode(RunMode.RUN_WITHOUT_ENCODER);
				m.setPower(-.2);
			}
			taring = true;
			taringEndTime = System.currentTimeMillis()+1000;
		}
		public double getDistanceFromTarget(){
			return hypot(hypot(armPos[0]-armTarget[0],armPos[1]-armTarget[1]),armPos[2]-armTarget[2]);
		}
		public int getSide(){return onLeftSide?0:1;}
		public double[] getCurrentArmPosition(){
			double[] out = new double[armPos.length];
			System.arraycopy(armPos, 0, out, 0, armPos.length);
			return out;
		}
		public int[] getMotorError(){
			int[] out = new int[armMotors.length];
			for(int i = 0; i < armMotors.length; i++)
				out[i] = armMotors[i].getTargetPosition()-armMotors[i].getCurrentPosition();
			return out;
		}
		
		//Private methods
		
		private void setTargetPositionFromArmTarget(){
			double x = armTarget[0], y = armTarget[1], z = armTarget[2];
			double h = horizontalLength, l = armLength;
	        double xp = hypot(x, z)-h, yp = y;
	        double oldR = hypot(xp, yp), r = min(2*armLength-.000001, oldR), theta = atan2(yp, xp);
	        double[] angles = {PI-atan2(z, x), PI/2-theta+asin(r/(2*l)),  2*asin(r/(2*l))};
	        for(int i = 0; i < angles.length; i++)
	        	angles[i] = max(minAngles[i], min(maxAngles[i], angles[i]));
	        armTarget = angleToCartesian(angles);
	        
	        if(!taring)
	        	for(int i = 0; i < angles.length; i++)
	        		armMotors[i].setTargetPosition((int) round((angles[i]-minAngles[i])*ticksPerRadian[i]));
		}
		private double[] angleToCartesian(double[] angles){
			double w = angles[0], s = angles[1], e = angles[2];
	        double h = horizontalLength, l = armLength;
	        double phi1 = PI-s, phi2 = e-s;
	        double xp = l*(cos(phi1)+cos(phi2)), yp = l*(sin(phi1)+sin(phi2));
	        return new double[]{-(xp+h)*cos(w), yp, (xp+h)*sin(w)};
		}
	}
	
/* ************************************************************************************************ */
	
	public static class Robot_{
		public DcMotor left_drive, left_waist, left_shoulder, left_elbow;
	    public Servo left_wrist, left_left_claw, left_right_claw;
	    public DcMotor right_drive, right_waist, right_shoulder, right_elbow;
	    public Servo right_wrist, right_left_claw, right_right_claw;
	    public Servo left_jewel, right_jewel;
	    //public ColorSensor jewel_sensor;

	    public Robot_(HardwareMap hm, DcMotor.RunMode armRunMode){
	        left_drive = hm.get(DcMotor.class, "left_drive");
	        left_waist = hm.get(DcMotor.class, "left_waist");
	        left_shoulder = hm.get(DcMotor.class, "left_shoulder");
	        left_elbow = hm.get(DcMotor.class, "left_elbow");

	        left_wrist = hm.get(Servo.class, "left_wrist");
	        left_left_claw = hm.get(Servo.class, "left_left_claw");
	        left_right_claw = hm.get(Servo.class, "left_right_claw");
	        
	        left_jewel = hm.get(Servo.class, "left_jewel");
	        
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
	        
	        right_jewel = hm.get(Servo.class, "right_jewel");

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
}
