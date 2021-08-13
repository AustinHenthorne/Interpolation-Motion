// motion.h defines all operations for a motion class

#ifndef MOTION_H
#define MOTION_H

#include <vector>

#include "DualQuaternion.h"

class Motion
{
public:
	~Motion();
	void readCtrlPositions(char fileNameArg[50]);
	void writePositions(char *fileNameArg);
	void drawCtrlPositions(void);

	// Calculates screw motion between control positions/orientations
	void screwMotion(void);			
	
	// Calculates Bezier motion approximation using deCasteljau algorithm 
	// on input data
	void curveBezier(void);			

	// Calculates affine motion approximation using deCasteljau algorithm 
	// on homogeneous matrices for input data
	void curveAffine(void);			
	
	// Calculates rational B-spline motion approximation on input data
	void curveBSpline(int p, int res);		

	// Calculates rational B-spline interpolation/approximation through
	// input data points or through polar decomposition of input data points
	void motionSolve(int p, int c, int res, int DQ, int PD);


public:
	vector<DualQuaternion> ctrlPos;
	int numberOfPositions;
	vector<double>		u;
	vector<double>		tk;
	vector<double>		w;
	vector<double>		N;
	Matrix				Nmatrix;
	vector<double>		Ntemp;
	double				L;
	double				R;
	double				r;
	double				d;
	Quaternion			D;
	vector<Quaternion>	G;
	vector<Quaternion>	H;
	vector<double>		Gangles;
	vector<double>		Hangles;
	vector<double>		GHangles;
	vector<DualQuaternion> cp;
	vector<Quaternion>	cpG;
	vector<Quaternion>	cpH;
};

#endif
