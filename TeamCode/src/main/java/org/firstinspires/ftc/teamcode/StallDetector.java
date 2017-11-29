package org.firstinspires.ftc.teamcode;

import java.util.List;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StallDetector {
    SingleMotorStallDetector[] detectors;
    public StallDetector(HardwareMap h){
        List<DcMotor> motors = h.getAll(DcMotor.class);
        detectors = new SingleMotorStallDetector[motors.size()];
        for(int i = 0; i < detectors.length; i++)
            detectors[i] = new SingleMotorStallDetector(motors.get(i), h.getNamesOf(motors.get(i)).iterator().next());
    }
    
    public void tick(){
        for(SingleMotorStallDetector d:detectors)
            d.test();
    }
    
    private class SingleMotorStallDetector{
        DcMotor motor;
        String name;
        public SingleMotorStallDetector(DcMotor motor, String name){
            this.motor = motor;
            this.name = name;
        }
    
        long lastPos, positionMotionTollerence = 5, lastPositionMoveTime = -1;
        long lastTarget, targetMotionTollerence = 0, lastTargetMoveTime = -1;
        int[] stationaryPositionTime = new int[] {15000, 3000, 7000, 4000},
                stationaryTargetTime = new int[] {15000, 3000, -1,   -1},
                        discrepencys = new int[] {10,    20,   20,   50};
        String[] description = new String[]{"Abandoned under strain for a long time", "Abandoned under high strain", "Commanded to remain at an invalid position for a long time", "Commanded to remain at a highly invalid position"};
        public void test(){
            int pos = motor.getCurrentPosition(), target = motor.getTargetPosition();
            if(lastPositionMoveTime == -1){
                lastPositionMoveTime = System.currentTimeMillis();
                lastPos = pos;
                lastTargetMoveTime = System.currentTimeMillis();
                lastTarget = target;
            }
            if(Math.abs(pos-lastPos) > 0){
                lastPositionMoveTime = System.currentTimeMillis();
                lastPos += Math.signum(pos-lastPos)*(Math.abs(pos-lastPos)-positionMotionTollerence);
            }
            if(Math.abs(target-lastTarget) > 0){
                lastTargetMoveTime = System.currentTimeMillis();
                lastTarget += Math.signum(target-lastTarget)*(Math.abs(target-lastTarget)-targetMotionTollerence);
            }
            int positionTime = (int) (System.currentTimeMillis()-lastPositionMoveTime);
            int targetTime = (int) (System.currentTimeMillis()-lastTargetMoveTime);
            int discrepency = Math.abs(motor.getCurrentPosition()-motor.getTargetPosition());
            
            for(int i = 0; i < stationaryPositionTime.length; i++){
                if(positionTime > stationaryPositionTime[i] && targetTime > stationaryTargetTime[i] && discrepency > discrepencys[i])
                    throw new RuntimeException(String.format("%s was %s: Stall detected. position has not changed in %d ms, target has not changed in %d ms, and the difference between current and target positions is %d", 
                            name, description[i], positionTime, targetTime, discrepency));
            }
        }
    }
}
