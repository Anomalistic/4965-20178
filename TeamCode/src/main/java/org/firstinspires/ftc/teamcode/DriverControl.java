package org.firstinspires.ftc.teamcodeV3;

import static java.lang.Math.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriverControl extends LinearOpMode{
	OneSidedController[] controllers; 
	@Override public void runOpMode() throws InterruptedException {
		RobotController rc = new RobotController(hardwareMap, gamepad1, gamepad2);
		controllers = new OneSidedController[]{new OneSidedController(rc.sides[0]), new OneSidedController(rc.sides[1])};
		waitForStart();
		long lastTime = System.currentTimeMillis();
		while(opModeIsActive()){
			double dt = (-lastTime+(lastTime=System.currentTimeMillis()))/1000.0;
			rc.tick();
			controllers[0].tick(dt);
			controllers[1].tick(dt);
			
			telemetry.addData("Dist To Target", String.format("%4.1f %4.1f", rc.sides[0].getDistanceFromTarget(), rc.sides[1].getDistanceFromTarget()));
			telemetry.addData("Ticks Per Second", String.format("%5.1f", 1/dt));
			
			telemetry.addData("Error", "");
			int[] left = rc.sides[0].getMotorError(), right = rc.sides[1].getMotorError(); 
			telemetry.addData("Waist", String.format("%4d %4d", left[0], right[0]));
			telemetry.addData("Shldr", String.format("%4d %4d", left[1], right[1]));
			telemetry.addData("Elbow", String.format("%4d %4d", left[2], right[2]));
	        
            telemetry.addData("Current Positions", "");
            telemetry.addData("Waist", String.format("%4d %4d", rc.r.left_waist.getCurrentPosition(),  rc.r.right_waist.getCurrentPosition()));
            telemetry.addData("Shldr", String.format("%4d %4d", rc.r.left_shoulder.getCurrentPosition(),  rc.r.right_shoulder.getCurrentPosition()));
            telemetry.addData("Elbow", String.format("%4d %4d", rc.r.left_elbow.getCurrentPosition(),  rc.r.right_elbow.getCurrentPosition()));

            double[] left1 = rc.sides[0].getCurrentArmPosition(), right1 = rc.sides[1].getCurrentArmPosition(); 
            telemetry.addData("X", String.format("%.3f %.3f", left1[0], right1[0]));
            telemetry.addData("Y", String.format("%.3f %.3f", left1[1], right1[1]));
            telemetry.addData("Z", String.format("%.3f %.3f", left1[2], right1[2]));
            telemetry.update();
		}
		rc.setJewelArmTarget(0);
	}
	
	private class OneSidedController{
		RobotController.RobotSide r;
		public OneSidedController(RobotController.RobotSide robotSide){
			r = robotSide;
		}
		
/* ***************************************************************************************************** */
		double[] armSpeed = {20, 17, 20};
		double wristSpeed = 1;
		double triggerStickPoint = .9, clawMin = 0, clawMax = .5, clawStuckValue = 1;
		double driveSpeed = 1;
		
		boolean driving = false, clawStuck = false;
		int maneuver = -1, phase = 0;
		long startTime = 0;
		long lastXPress = -1;
		
		void tick(double dt){
			if(!driving){//Not driving
				//Arm movement
				double[] xyz = {(r.getSide()*2-1)*r.getAnalogValues()[0], r.getAnalogValues()[3], r.getAnalogValues()[1]};
				for(int i = 0; i < xyz.length; i++)
					xyz[i] = r.getArmTarget()[i]+xyz[i]*dt*armSpeed[i];
				r.setArmTarget(xyz);
				
				//Wrist movement
				r.setWristTarget(r.getWristTarget()+r.getAnalogValues()[2]*dt*wristSpeed*(r.getArmTarget()[2]>0?1:-1));
						
				//Claw movement
				double trigger = max(r.getAnalogValues()[4], r.getAnalogValues()[5]);
				if(r.getButtonPresses()[8] || r.getButtonPresses()[9]) clawStuck = false;
				if(trigger >= triggerStickPoint) clawStuck = true;
				if(clawStuck) r.setClawTarget(clawStuckValue);
				else r.setClawTarget(RobotController.map(new double[]{0, triggerStickPoint, clawMin, clawMax}, trigger));
				
			}else if(!(controllers[0].driving && r.getSide() == 1)){//Driving
				controllers[0].r.setDriveSpeed(r.getAnalogValues()[1]*driveSpeed);
				controllers[1].r.setDriveSpeed(r.getAnalogValues()[3]*driveSpeed);
			}
			
			if(r.getButtonPresses()[3]){//toggle driving
				driving = !driving;
				if(!(controllers[0].driving || controllers[1].driving)){
					controllers[0].r.setDriveSpeed(0);
					controllers[1].r.setDriveSpeed(0);
				}
			}
			
			if(r.getButtonPresses()[2]){//Return to origin and tare
				if(System.currentTimeMillis() < lastXPress+1000)
					r.tare();
				else
					r.returnToOrigin();
				lastXPress = System.currentTimeMillis();
			}
			
			if(r.getButtonPresses()[5]){
				maneuver = 0;//To pit
				phase = 0;
				startTime = System.currentTimeMillis();
			}
			
			if(r.getButtonPresses()[7]){
				maneuver = 1;//To shelf
				phase = 0;
				startTime = System.currentTimeMillis();
			}
			
			if(maneuver == 0){//To pit
				if(phase == 0){
					clawStuck = false;
					phase++;
				}
				if(phase == 1 && System.currentTimeMillis() > startTime+50){
					double[] pos = r.getCurrentArmPosition();
					pos[1] += 15;//Y
					pos[2] += 5;//Z
					r.setArmTarget(pos);
					phase++;
				}
				if(phase == 2 && r.getDistanceFromTarget() < 10){
					double[] pos = r.getArmTarget();
					pos[1] += 5;//Y
					pos[2] += 15;//Z
					r.setArmTarget(pos);
					phase++;
				}
				if(phase == 3 && r.getDistanceFromTarget() < 10){
					double[] pos = r.getArmTarget();
					pos[0] += 15;//X
					pos[2] += 5;//Z
					r.setArmTarget(pos);
					phase++;
				}
				if(phase == 4 && r.getDistanceFromTarget() < 10){
					r.setArmTarget(new double[]{17, 25, 43});
					r.setWristTarget((r.getSide()*2-1)*.2);
					maneuver = -1;
				}
			}
			
			if(maneuver == 1){//To shelf
				if(phase == 0){
					clawStuck = true;
					phase++;
				}
				if(phase == 1 && System.currentTimeMillis() > startTime+50){
					double[] pos = r.getCurrentArmPosition();
					pos[1] += 20;//Y
					pos[2] += -10;//Z
					r.setArmTarget(pos);
					phase++;
				}
				if(phase == 2 && r.getDistanceFromTarget() < 10){
					r.setArmTarget(new double[]{0, 36, -33});
					maneuver = -1;
				}
			}
		}
	}
}
