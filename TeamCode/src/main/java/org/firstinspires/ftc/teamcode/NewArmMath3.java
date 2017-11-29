package org.firstinspires.ftc.teamcode;

import static java.lang.Math.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NewArmMath3 {
    public double
    wristMin = .18, wristMax = .60,  wristStart = (wristMin+wristMax)/2,
    waistStart = 75/180.0*PI, shoulderStart = 40/180.0*PI, elbowStart = 13.16/180.0*PI,
    waistTPR = 2*2516/(2*PI), shoulderTPR = 4*1256/(2*PI), elbowTPR = 4*795/(2*PI),//Per radian not per rotation
    horizontalLength = 28.80, armLength = 28.80,//32.00 mm * 9
    deadzone = .02, controllAugmentation = 0;
    private double[]
    speed = {25, 25, .5/*2 This value for relative wrist positioning*/, 25};//cm/s,cm/s,Z*/s,cm/s
    public double[]
    joystickZero = {-2, -2, -2, -2};
    public double
    x, y, z,
    waistAngle, shoulderAngle, elbowAngle,
    waist, shoulder, elbow, wrist;//Encoder (or Z*) values
    public boolean onRightSide;

    private long lastTime = -1;
    public NewArmMath3(boolean onRightSide){this.onRightSide = onRightSide; resetPosition();}
    public void zeroJoystick(double[] joystickValues){joystickZero = joystickValues.clone();}
    public void resetPosition(){wrist=wristStart; waistAngle=waistStart; shoulderAngle=shoulderStart; elbowAngle=elbowStart; waist=shoulder=elbow=0;updateRect();}
    public void resetWrist(){wrist=wristStart;}
    public void setPosition(double[] xyz){x = xyz[0]; y = xyz[1]; z = xyz[2]; updateMotors();}
    public void setWrist(double wrist){this.wrist = wrist;}
    public void update(double[] joystickValues, Telemetry t) {
        if(joystickZero[0] == -2)
            zeroJoystick(joystickValues);
        if(lastTime == -1) lastTime = System.currentTimeMillis();
        long dt = System.currentTimeMillis()-lastTime;
        lastTime += dt;
        for(int i = 0; i < 4; i+=2){
            joystickValues[i] -= joystickZero[i];
            joystickValues[i+1] -= joystickZero[i+1];
            double dist = hypot(joystickValues[i], joystickValues[i+1]);
            double scale = dist <= deadzone?0:(1+dist);
            joystickValues[i] *= dt*scale*speed[i]/1000;
            joystickValues[i+1] *= -dt*scale*speed[i+1]/1000;
        }
        x += joystickValues[0]*(this.onRightSide?1:-1); y += joystickValues[1]; wrist += (y>0?1:-1)*joystickValues[2]; z += joystickValues[3];
        wrist = Math.max(Math.min(wrist, wristMax), wristMin);
        
        //Wrist is absolute
        /*wrist = wristStart+joystickValues[2]*(x>0?1:-1);
        t.addData("ws", wristStart);
        t.addData("w", wrist);*/
        
        updateMotors();
    }
    public void updateRect(){
        double w = waistAngle, s = shoulderAngle, e = elbowAngle;
        double h = horizontalLength, l = armLength;
        double phi1 = PI-s, phi2 = e-s;
        double xp = l*(cos(phi1)+cos(phi2)), yp = l*(sin(phi1)+sin(phi2));
        x = -(xp+h)*cos(w); y = (xp+h)*sin(w); z = yp;
    }
    private void updateMotors(){
        double h = horizontalLength, l = armLength;
        double xp = hypot(x, y)-h, yp = z;
        double oldR = hypot(xp, yp), r = min(2*armLength-.000001, oldR), theta = atan2(yp, xp);
        waistAngle = PI-atan2(y, x); shoulderAngle = PI/2-theta+asin(r/(2*l)); elbowAngle = 2*asin(r/(2*l));
        waist = (waistAngle-waistStart)*waistTPR; shoulder = (shoulderAngle-shoulderStart)*shoulderTPR; elbow = (elbowAngle-elbowStart)*elbowTPR; 
        if (r != oldR)
            updateRect();
            
        //Prevent overheating
        waist = Math.min(Math.max(0, waist),3618);
        shoulder = Math.min(Math.max(0, shoulder),2130);
        elbow = Math.max(0, elbow);
    }
}