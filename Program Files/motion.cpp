#include <iostream>
#include <fstream>
#include <vector>
#include <GL\glut.h>
#include "motion.h"
#include "Quaternion.h"
#include "hMatrix.h"
#include "hPoint.h"
#define _USE_MATH_DEFINES
#include "math.h"
#include "color.h"

using namespace std;

void Motion::readCtrlPositions(char fileNameArg[50])
{
	ifstream inFile(fileNameArg, ios::in);
	extern int drawCtrlPositions;

	if (!inFile)
	{
		cerr << "File " << fileNameArg << " could not be opened" << endl;
		//exit(1);
		drawCtrlPositions = 0;
		return;
	}

	int i;
	double Ltemp;
	L = 0;
	double d2;
	G.clear(); H.clear();
	Gangles.clear(); Hangles.clear();	GHangles.clear();
	extern float Rmod;
	extern int useRmod;

	inFile >> numberOfPositions;
	
	Quaternion *RotationQuaternion = new Quaternion[numberOfPositions];
	Vector *TranslationVector = new Vector[numberOfPositions];

	for (i = 0; i < numberOfPositions; i++)
	{
		inFile >> RotationQuaternion[i];
	}

	// Determine Lmax and calculate R
	for (i = 0; i < numberOfPositions; i++)
	{
		inFile >> TranslationVector[i];
		for (int j = 0; j <= 2; ++j)
		{
			Ltemp = TranslationVector[i].coord[j];
			if (fabs(Ltemp) > L)
			{
				L = fabs(Ltemp);
			}
		}
	}

	if (useRmod)
	{
		R = Rmod;
	}
	else
	{
		R = 24 * L / M_PI;
	}

	ctrlPos.clear();
	for (i = 0; i < numberOfPositions; i++)
	{
		d = 0;
		d2 = 0;
		DualQuaternion dQ(RotationQuaternion[i], TranslationVector[i]);
		ctrlPos.push_back(dQ);

		// Break dual quaternions into quaternions G and H and calculate corresponding angles
		for (int j = 0; j <= 2; ++j)
		{
			d2 += pow(TranslationVector[i].coord[j], 2);
		}
		d = sqrt(d2);
		r = sqrt((4 * pow(R, 2)) + pow(d, 2)) / (2 * R);
		
		G.push_back((ctrlPos[i].GetReal() + (ctrlPos[i].GetDual() / R)) / r);
		H.push_back((ctrlPos[i].GetReal() - (ctrlPos[i].GetDual() / R)) / r);

		Gangles.push_back(2 * acos(G[i].q[3]));
		Hangles.push_back(2 * acos(H[i].q[3]));
		GHangles.push_back(sqrt(pow(Gangles[i], 2) + pow(Hangles[i], 2)));
	}

    delete[] RotationQuaternion;
    delete[] TranslationVector;
    
}

void Motion::writePositions(char *fileNameArg)
{
	ofstream outFile(fileNameArg, ios::out);
	
	if (!outFile)
	{
		cerr<<"File" << fileNameArg << "could not be opened for writing" << endl;
		exit(1);
	}

	int i;
	
	outFile << numberOfPositions << endl;
	
	for(i=0; i<numberOfPositions; i++)
		outFile<<ctrlPos[i]<<endl;
}


void Motion::drawCtrlPositions(void)
{
	vector <hMatrix> homogeneousMatricesForCtrlPositions;
	for (int i=0; i<numberOfPositions; i++)
	{
		homogeneousMatricesForCtrlPositions.push_back(ctrlPos[i].dualQuaternionToHomogeneousMatrix().transpose());
		double MatrixforOpenGLStack[16];

		for (int i1=0; i1<4; i1++)
			for (int i2=0; i2<4; i2++)
				MatrixforOpenGLStack[4*i1+i2] =  homogeneousMatricesForCtrlPositions.at(i).m[i1][i2];
			
		::glPushMatrix();
		::glMultMatrixd(MatrixforOpenGLStack);
		glutSolidTeapot(0.15);
		::glPopMatrix();
	}

}

// Calculate screw motion between control positions/orientations
void Motion::screwMotion(void)
{
	hMatrix homogeneousMatrixForScrewMotion;
	DualQuaternion ctrlPosParameterized;
	int DIV = 10;
	double MatrixforOpenGLStack[16];

	for (int i = 0; i < numberOfPositions - 1; i++)
	{
		for (int t = 0; t <= DIV; t++)
		{
			ctrlPosParameterized = (ctrlPos[i] * (1 - ((float)t / (float)DIV))) + (ctrlPos[i + 1] * ((float)t / (float)DIV));
			homogeneousMatrixForScrewMotion = ctrlPosParameterized.dualQuaternionToHomogeneousMatrix().transpose();

			for (int i1 = 0; i1 < 4; i1++)
			{
				for (int i2 = 0; i2 < 4; i2++)
				{
					MatrixforOpenGLStack[4 * i1 + i2] = homogeneousMatrixForScrewMotion.m[i1][i2];
				}
			}
			::glPushMatrix();
			::glMultMatrixd(MatrixforOpenGLStack);
			glutSolidTeapot(0.15);
			::glPopMatrix();
		}
	}
}

// Calculate Bezier motion approximation using deCasteljau algorithm 
// on dual quarternions for control positions/orientations
void Motion::curveBezier(void)
{
	vector<DualQuaternion> p;
	int DIV = 20;
	hMatrix homogeneousMatrixForBezierCurve;
	double MatrixforOpenGLStack[16];

	for (int i = 0; i < numberOfPositions; i++)
	{
		p.push_back(ctrlPos[i]);
	}
	for (int t = 0; t <= DIV; t++)
	{
		for (int r = 1; r < numberOfPositions; r++)
		{
			for (int i = 0; i < numberOfPositions - r; i++)
			{
				p[i] = ((1.0 - ((float)t / (float)DIV)) * p[i]) + (((float)t / (float)DIV) * p[i + 1]);
			}
		}
	
		homogeneousMatrixForBezierCurve = p[0].dualQuaternionToHomogeneousMatrix().transpose();

		for (int i1 = 0; i1 < 4; i1++)
		{
			for (int i2 = 0; i2 < 4; i2++)
			{
				MatrixforOpenGLStack[4 * i1 + i2] = homogeneousMatrixForBezierCurve.m[i1][i2];
			}
		}

		::glPushMatrix();
		::glMultMatrixd(MatrixforOpenGLStack);
		glutSolidTeapot(0.15);
		::glPopMatrix();
	}
}

// Calculate affine motion approximation using deCasteljau algorithm 
// on homogeneous matrices for control positions/orientations
void Motion::curveAffine(void)
{
	int DIV = 20;
	vector <hMatrix> homogeneousMatricesForAffineCurve;
	double MatrixforOpenGLStack[16];

	for (int i = 0; i < numberOfPositions; i++)
	{
		homogeneousMatricesForAffineCurve.push_back(ctrlPos[i].dualQuaternionToHomogeneousMatrix().transpose());
	}

	for (int t = 0; t <= DIV; t++)
	{
		for (int r = 1; r < numberOfPositions; r++)
		{
			for (int i = 0; i < numberOfPositions - r; i++)
			{
				homogeneousMatricesForAffineCurve[i] = ((1.0 - ((float)t / (float)DIV)) * homogeneousMatricesForAffineCurve[i]) + (((float)t / (float)DIV) * homogeneousMatricesForAffineCurve[i + 1]);
			}
		}
	
		for (int i1 = 0; i1 < 4; i1++)
		{
			for (int i2 = 0; i2 < 4; i2++)
			{
				MatrixforOpenGLStack[4 * i1 + i2] = homogeneousMatricesForAffineCurve[0].m[i1][i2];
			}
		}

		::glPushMatrix();
		::glMultMatrixd(MatrixforOpenGLStack);
		glutSolidTeapot(0.15);
		::glPopMatrix();
	
	}
}



// Calculate rational B-spline motion approximation on dual quaternions
// for control positions/orientations
void Motion::curveBSpline(int p, int res)
{
	int DIV = res;
	hMatrix homogeneousMatrixForBSplineCurve;
	double MatrixforOpenGLStack[16];
	Quaternion zero = { 0, 0, 0, 0 };
	DualQuaternion sd = { zero, zero };
	int i, m;
	double den, left, right, knot, t;
	m = numberOfPositions + p;

	// Set weights equal to 1 for this project
	w.clear();
	for (i = 0; i <= numberOfPositions - 1; ++i)
	{
		w.push_back(1);
	}

	// Set uniformly spaced, clamped, knot vector
	u.clear();
	for (i = 0; i <= m; ++i)
	{
		knot = (double)i / (double)m;
		u.push_back(knot);
	}
	for (i = 0; i <= p; ++i)
	{
		u[i] = 0;
		u[m - i] = 1;
	}

	// Iterate to calculate position/orientation at various points
	for (int z = 0; z <= DIV; ++z)
	{
		t = (double)z / DIV;
		N.clear();
		for (i = 0; i < m; ++i)
		{
			if ((t >= u[i]) && (t < u[i + 1]))
			{
				N.push_back(1);
			}
			else
			{
				N.push_back(0);
			}
		}
		for (int s = 1; s <= p; s++)
		{
			for (i = 0; i <= m - s - 1; i++)
			{
				if (N[i] == 0)
				{
					left = 0;
				}
				else
				{
					left = (t - u[i]) / (u[i + s] - u[i]);
				}
				if (N[i + 1] == 0)
				{
					right = 0;
				}
				else
				{
					right = (u[i + s + 1] - t) / (u[i + s + 1] - u[i + 1]);
				}

				N[i] = (left*N[i]) + (right*N[i + 1]);
			}
		}
		if (z == DIV)
		{
			for (i = 0; i <= numberOfPositions - 1; ++i)
			{
				N[i] = 0;
			}
			N[numberOfPositions - 1] = 1;
		}

		sd = { zero, zero };
		den = 0;
		for (i = 0; i <= numberOfPositions - 1; ++i)
		{
			sd = sd + (N[i] * ctrlPos[i] * w[i]);
			den += N[i] * w[i];
		}
		sd = sd / den;


		homogeneousMatrixForBSplineCurve = sd.dualQuaternionToHomogeneousMatrix().transpose();

		for (int i1 = 0; i1 < 4; i1++)
		{
			for (int i2 = 0; i2 < 4; i2++)
			{
				MatrixforOpenGLStack[4 * i1 + i2] = homogeneousMatrixForBSplineCurve.m[i1][i2];
			}
		}

		::glPushMatrix();
		::glMultMatrixd(MatrixforOpenGLStack);
		glutSolidTeapot(0.15);
		::glPopMatrix();

	}

}

void Motion::motionSolve(int p, int c, int res, int DQ, int PD)
{
	int i, j, k;
	double GHanglesTotal = 0;
	double GHtemp = 0;
	int m = c + p;
	double left, right;

	for (i = 0; i < numberOfPositions - 1; ++i)
	{
		GHanglesTotal += fabs(GHangles[i + 1] - GHangles[i]);
	}

	tk.clear();
	/*
	// Calculate parameter values using chord length method
	tk.push_back(0);
	
	for (i = 0; i < numberOfPositions - 1; ++i)
	{
		GHtemp += fabs(GHangles[i + 1] - GHangles[i]);
		tk.push_back(GHtemp / GHanglesTotal);
	}
	*/

	// Calculate parameters uniformly spaced
	for (i = 0; i <= numberOfPositions - 1; ++i)
	{
		tk.push_back((double)i / ((double)numberOfPositions - 1));
	}
	
	u.clear();
	/*
	// Generate clamped knot vector using averaging method
	for (i = 0; i <= m; ++i)
	{
		u.push_back(0);
	}
	for (j = 1; j <= c - p; ++j)
	{
		double tktemp = 0;
		for (i = j; i <= j + p - 1; ++i)
		{
			tktemp += tk[i];
		}
		u[p + j] = tktemp / p;
	}
	for (i = m - p; i <= m; ++i)
	{
		u[i] = 1;
	}
	*/
	
	// Generate uniformly spaced, clamped knot vector
	for (i = 0; i <= m; ++i)
	{
		u.push_back((double)i / (double)m);
	}
	for (i = 0; i <= p; ++i)
	{
		u[i] = 0;
		u[m - i] = 1;
	}

	// Initialize basis function matrix to solve the system
	Nmatrix.row = numberOfPositions;
	Nmatrix.column = c;
	Nmatrix.MallocSpace();

	for (j = 0; j <= numberOfPositions - 1; ++j)
	{
		Ntemp.clear();
		if (j == 0)
		{
			for (i = 0; i <= c - 1; ++i)
			{
				if (i == 0)
				{
					Ntemp.push_back(1);
				}
				else
				{
					Ntemp.push_back(0);
				}
			}
		}
		else if (j == numberOfPositions - 1)
		{
			for (i = 0; i <= c - 1; ++i)
			{
				if (i == c - 1)
				{
					Ntemp.push_back(1);
				}
				else
				{
					Ntemp.push_back(0);
				}
			}
		}
		else
		{
			for (i = 0; i < m; ++i)
			{
				if ((tk[j] >= u[i]) && (tk[j] < u[i + 1]))
				{
					Ntemp.push_back(1);
				}
				else
				{
					Ntemp.push_back(0);
				}
			}
			for (int s = 1; s <= p; s++)
			{
				for (i = 0; i <= m - s - 1; i++)
				{
					if (Ntemp[i] == 0)
					{
						left = 0;
					}
					else
					{
						left = (tk[j] - u[i]) / (u[i + s] - u[i]);
					}
					if (Ntemp[i + 1] == 0)
					{
						right = 0;
					}
					else
					{
						right = (u[i + s + 1] - tk[j]) / (u[i + s + 1] - u[i + 1]);
					}
					Ntemp[i] = (left*Ntemp[i]) + (right*Ntemp[i + 1]);
				}
			}
		}
		for (i = 0; i <= c - 1; ++i)
		{
			Nmatrix.m[j][i] = Ntemp[i];
		}
	}

	// Solve for control points for full duel quaternions as well as decomponsed G and H quaternions
	Matrix NT_N_NT = ((Nmatrix.Transpose() * Nmatrix).Inverse()) * Nmatrix.Transpose();
	Quaternion cpGtemp, cpHtemp;
	DualQuaternion cptemp;
	cpG.clear(); cpH.clear(); cp.clear();
	for (j = 0; j <= c - 1; ++j)
	{
		cpGtemp = { 0, 0, 0, 0 };
		cpHtemp = { 0, 0, 0, 0 };
		cptemp = { cpGtemp, cpHtemp };
		for (i = 0; i <= numberOfPositions - 1; ++i)
		{
			cpGtemp = cpGtemp + (NT_N_NT.m[j][i] * G[i]);
			cpHtemp = cpHtemp + (NT_N_NT.m[j][i] * H[i]);
			cptemp  = cptemp  + (NT_N_NT.m[j][i] * ctrlPos[i]);
		}
		cpG.push_back(cpGtemp);
		cpH.push_back(cpHtemp);
		cp.push_back(cptemp);
	}

	// Generate resulting motion using new control points
	// Generate motion using dual quaternions as well as decomposed G and H quaternions
	int DIV = res;
	hMatrix homogeneousMatrixForBSplineCurve;
	double MatrixforOpenGLStack[16];
	hMatrix GHhomogeneousMatrixForBSplineCurve;
	double GHMatrixforOpenGLStack[16];

	Quaternion zero = { 0, 0, 0, 0 };
	Quaternion sdG = zero; Quaternion sdH = zero;
	Quaternion sdQ;	Quaternion sdT;
	DualQuaternion sd = { zero, zero };
	DualQuaternion sdGH = { zero, zero };
	double den, denG, denH, t;

	// Set all weights to 1 for this project
	w.clear();
	for (i = 0; i <= c - 1; ++i)
	{
		w.push_back(1);
	}

	// Iterate to generate positions/orientations for various points
	for (int z = 0; z <= DIV; ++z)
	{
		t = (double)z / DIV;
		sdG = zero;
		sdH = zero;
		sd = { zero, zero };
		sdGH = { zero, zero };
		Ntemp.clear();
		for (i = 0; i < m; ++i)
		{
			if ((t >= u[i]) && (t < u[i + 1]))
			{
				Ntemp.push_back(1);
			}
			else
			{
				Ntemp.push_back(0);
			}
		}
		for (int s = 1; s <= p; s++)
		{
			for (i = 0; i <= m - s - 1; i++)
			{
				if (Ntemp[i] == 0)
				{
					left = 0;
				}
				else
				{
					left = (t - u[i]) / (u[i + s] - u[i]);
				}
				if (Ntemp[i + 1] == 0)
				{
					right = 0;
				}
				else
				{
					right = (u[i + s + 1] - t) / (u[i + s + 1] - u[i + 1]);
				}
				Ntemp[i] = (left*Ntemp[i]) + (right*Ntemp[i + 1]);
			}
		}

		den = 0;
		denG = 0;
		denH = 0;
		for (i = 0; i <= c - 1; ++i)
		{
			sd  = sd  + (Ntemp[i] * cp[i]  * w[i]);
			sdG = sdG + (Ntemp[i] * cpG[i] * w[i]);
			sdH = sdH + (Ntemp[i] * cpH[i] * w[i]);
			den += Ntemp[i] * w[i];
			denG += Ntemp[i] * w[i];
			denH += Ntemp[i] * w[i];
		}
		sd = sd / den;
		sdG = sdG / denG;
		sdH = sdH / denH;

		// Rebuild dual quaternion from resulting G and H interpolation/approximation
		sdQ = (sdG + sdH) / sqrt((sdG + sdH).Modulus());
		double D4 = sqrt((sdG + sdH).Modulus()) / 2;
		r = 1 / D4;
		sdT = R * ((r * sdG) - sdQ);
		sdGH = { sdQ, sdT };
	
		homogeneousMatrixForBSplineCurve = sd.dualQuaternionToHomogeneousMatrix().transpose();
		GHhomogeneousMatrixForBSplineCurve = sdGH.dualQuaternionToHomogeneousMatrix().transpose();

		for (int i1 = 0; i1 < 4; i1++)
		{
			for (int i2 = 0; i2 < 4; i2++)
			{
				MatrixforOpenGLStack[4 * i1 + i2] = homogeneousMatrixForBSplineCurve.m[i1][i2];
				GHMatrixforOpenGLStack[4 * i1 + i2] = GHhomogeneousMatrixForBSplineCurve.m[i1][i2];
			}
		}

		// Display motion using dual quaternion calculation method
		if (DQ)
		{
			::glMaterialfv(GL_FRONT, GL_DIFFUSE, MAGENTA);
			::glPushMatrix();
			::glMultMatrixd(MatrixforOpenGLStack);
			glutSolidTeapot(0.15);
			::glPopMatrix();
		}

		// Display motion using polar decomposition method
		if (PD)
		{
			::glMaterialfv(GL_FRONT, GL_DIFFUSE, BLUE);
			::glPushMatrix();
			::glMultMatrixd(GHMatrixforOpenGLStack);
			glutSolidTeapot(0.15);
			::glPopMatrix();
		}

	}
	
}



Motion::~Motion()
{
	
}