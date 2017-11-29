package org.firstinspires.ftc.teamcode;

public class Targets {
	double[][][][] shelf = new double[2][3][4][3];//Side, x, y
	
	public Targets(){
		double s0 = 0, si = -30, 
			   x0 = -5, xi = 10, 
			   y0 = -2, yi = 8,
			   z0 = -50;
		for(int s=0; s<shelf.length; s++)
			for(int x=0; x<shelf[0].length; x++)
				for(int y=0; y<shelf[0][0].length; y++)
					shelf[s][x][y] = new double[]{s0+x0+si*s+xi*x, y0+yi*y, z0};
	}
}
