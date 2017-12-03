package org.firstinspires.ftc.teamcodeV3;

import static java.lang.Math.max;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Autonomous_ extends LinearOpMode{
	@Override public void runOpMode() throws InterruptedException {
		RobotController rc = new RobotController(hardwareMap, gamepad1, gamepad2);

//Select starting position
		double triggerStickPoint = .9, clawMin = 0, clawMax = .5, clawStuckValue = 1;
		boolean[] stuck = new boolean[2];
		boolean onRed = false, headingLeft = true;
		while(!isStarted()){
			rc.tick();
			if(rc.sides[0].getButtonPresses()[1] || rc.sides[1].getButtonPresses()[1]) onRed = true;
			if(rc.sides[0].getButtonPresses()[2] || rc.sides[1].getButtonPresses()[2]) onRed = false;
			if(rc.sides[0].getButtonPresses()[6] || rc.sides[1].getButtonPresses()[6]) headingLeft = true;
			if(rc.sides[0].getButtonPresses()[4] || rc.sides[1].getButtonPresses()[4]) headingLeft = false;

			double trigger = 0; boolean release = false;
			for(int i = 0; i <= 1; i++) for (int j = 4; j <= 5; j++){
				trigger = max(trigger, rc.sides[i].getAnalogValues()[j]);
				release |= rc.sides[i].getButtonPresses()[j];
			}
			if(release) stuck[headingLeft?0:1] = false;
			if(trigger > triggerStickPoint) stuck[headingLeft?0:1] = true;
			if(stuck[headingLeft?0:1]) rc.sides[headingLeft?0:1].setClawTarget(clawStuckValue);
			else rc.sides[headingLeft?0:1].setClawTarget(RobotController.map(new double[]{0, triggerStickPoint, clawMin, clawMax}, trigger));
			if(stuck[headingLeft?1:0]) rc.sides[headingLeft?1:0].setClawTarget(clawStuckValue);
			else rc.sides[headingLeft?1:0].setClawTarget(clawMin);
			
			telemetry.addData("Team", onRed?"Red":"Blue");
            telemetry.addData("Direction to move", headingLeft?"<- Left":"Right ->");
            telemetry.addData("Lane", headingLeft^onRed?"Short":"Long");
            telemetry.update();
		}

		int dir = headingLeft?1:-1;//Assume we are heading left, and multiply by this if necessary
//Jewel
		boolean jewelSensorClockwiseOfArm = true;
		rc.setJewelArmTarget(1);
		if(!rc.jewelColorIsRed()^onRed^jewelSensorClockwiseOfArm^headingLeft){
			double turnPower = 1;
			long turnTime = 500;
			rc.sides[0].setDriveSpeed(-dir*turnPower);
			rc.sides[1].setDriveSpeed(dir*turnPower);
			sleep(turnTime);
			rc.setJewelArmTarget(0);
			rc.sides[0].setDriveSpeed(dir*turnPower);
			rc.sides[1].setDriveSpeed(-dir*turnPower);
			sleep(turnTime);
			rc.sides[0].setDriveSpeed(0);
			rc.sides[1].setDriveSpeed(0);
		}

//Shelve
		RobotController.RobotSide r = rc.sides[headingLeft?0:1];
		r.setWristTarget(dir*.5);
		
		r.setArmMaxSpeed(.2);
		r.setArmTarget(new double[]{55, 10, 10});
		while(r.getDistanceFromTarget() > 10)rc.tick();

		r.setArmMaxSpeed(.1);
		r.setArmTarget(new double[]{55, 10, -5});
		while(r.getDistanceFromTarget() > 10)rc.tick();
		sleep(1000);
		
		r.setClawTarget(clawMin);
		sleep(200);
		
		r.setArmMaxSpeed(.1);
		r.setArmTarget(new double[]{55, 10, 20});
		while(r.getDistanceFromTarget() > 10)rc.tick();

		r.setClawTarget(clawMax);
		r.setArmMaxSpeed(.2);
		r.returnToOrigin();
		while(r.getDistanceFromTarget() > 20)rc.tick();
		sleep(2000);

//Leave balancing stone
		double turnPower = 1;
		long jewelTurnTime = 500, turnTime = 1000;
		rc.sides[0].setDriveSpeed(dir*turnPower);
		rc.sides[1].setDriveSpeed(-dir*turnPower);
		sleep(jewelTurnTime);
		rc.setJewelArmTarget(0);
		sleep(turnTime-jewelTurnTime);

		double drivePower = .4;
		long driveTime = 500;
		rc.sides[0].setDriveSpeed(-drivePower);
		rc.sides[1].setDriveSpeed(-drivePower);
		sleep(driveTime);
		
		rc.sides[0].setDriveSpeed(-dir*turnPower);
		rc.sides[1].setDriveSpeed(dir*turnPower);
		sleep(turnTime);

		rc.sides[0].setDriveSpeed(0);
		rc.sides[1].setDriveSpeed(0);

		rc.setJewelArmTarget(.5);
	}
}
