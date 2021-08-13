/**********************************************************************************
 *****************  Final Term Project - Team 2 ***********************************
 **********************************************************************************/

#include <iostream>
#include <vector>
#include <GL/glut.h>
#include <GL/glui.h>
#include "motion.h"
#include "OpenGLInitialization.h"
#include "Interaction.h"
#include "color.h"
#include "string.h"
#include <string>

using namespace std;

GLUI *glui;					// A pointer to GLUI
GLUI_EditText *edittext;	// A pointer to GLUI_edittext
int win_id;					// each curve on each screen

char fname[50] = "input.txt";
int n = 0;					// Tracks number of input data points
int c = 4;					// Tracks number of desired control points
int p = 1;					// Tracks degree for B-spline curves
int res = 20;				// Tracks motion resolution
float Rmod = 10;			// To demonstrate changing hyperspherical radius
int useRmod = 0;
float incrdecr = 0.5;

int drawCtrlPositions = 0;	// Toggles for displays
int screwMotion = 0;
int curveBezier = 0;
int curveAffine = 0;
int curveBspline = 0;
int curveInterpwDQ = 0;
int curveInterpwPD = 0;

void display(void)
{
	static Motion m;
	m.readCtrlPositions(fname);
	n = m.numberOfPositions;

	::glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    ::glPushMatrix();
	::glTranslated( Interaction::m_xTrans, Interaction::m_yTrans, Interaction::m_zTrans);
	::glRotated(Interaction::m_xRotate, 1.0, 0.0, 0.0);
	::glRotated(Interaction::m_yRotate, 0.0, 1.0, 0.0);
	::glRotated(Interaction::m_zRotate, 0.0, 0.0, 1.0);


	// Display input data
	if (drawCtrlPositions)
	{
		::glMaterialfv(GL_FRONT, GL_DIFFUSE, YELLOW);
		m.drawCtrlPositions();
	}

	// Display screw motion between control positions/orientations
	if (screwMotion)
	{
		::glMaterialfv(GL_FRONT, GL_DIFFUSE, BROWN);
		m.screwMotion();
	}

	// Display Bezier motion approximation using deCasteljau algorithm 
	// on dual quarternions for control positions/orientations
	if (curveBezier)
	{
		::glMaterialfv(GL_FRONT, GL_DIFFUSE, GREEN);
		m.curveBezier();
	}

	// Display affine motion approximation using deCasteljau algorithm 
	// on homogeneous matrices for control positions/orientations
	if (curveAffine)
	{
		::glMaterialfv(GL_FRONT, GL_DIFFUSE, BLACK);
		m.curveAffine();
	}

	// Display rational B-spline motion approximation using the input
	// data as control points
	if (curveBspline)
	{
		::glMaterialfv(GL_FRONT, GL_DIFFUSE, RED);
		m.curveBSpline(p, res);
	}

	// Display rational B-spline motion interpolation/approximation between
	// the input data points
	if (curveInterpwDQ || curveInterpwPD)
	{
		m.motionSolve(p, c, res, curveInterpwDQ, curveInterpwPD);
	}


	::glPopMatrix();
	::glutSwapBuffers();
}


void shellMessages(void)
{
	cout << "Steven Zilg & Austin Henthorne \nMEC 572 \nFinal Project \n\n\n"
		<< "This program displays various methods of motion generation "
		<< "for a given set of data points. The data points are displayed in YELLOW.\n\n"
		<< "The possible motions include:\n\n"
		<< "1) Screw motion between the dual quarternion control points -> BROWN \n\n"
		<< "2) Bezier motion approximation using the de Casteljau algorithm on the input data points -> GREEN \n\n"
		<< "3) Affine motion approximation using the de Casteljau algorithm on the homoageneous matrices of the input data points -> BLACK \n\n"
		<< "4) Rational B-spline motion approximation on the input data points -> RED \n\n"
		<< "5) Rational B-spline motion interpolation/approximation using global curve interpolation/least squares approximation on the input data points -> MAGENTA \n\n"
		<< "6) Rational B-spline motion interpolation/approximation using global curve interpolation/least squares approximation on the polar decomposition of the input data points -> BLUE \n\n";
}


void selectMessage(int msg)
{
	switch (msg) {
	case 1:					
		if (p < 1)
		{
			p = 1;
		}
		else if (p >= (n - 1))
		{
			p = n - 1;
		}
		break;
	case 2:					
		if (p < (n - 1))
		{
			p += 1;
		}
		else if (p >= (n - 1))
		{
			p = n - 1;
		}
		break;
	case 3:					
		if (p > 1)
		{
			p -= 1;
		} 
		break;
	case 4:					
		if (c < 1)
		{
			c = 1;
		}
		break;
	case 5:
		c += 1;
		break;
	case 6:
		if (c > 1)
		{
			c -= 1;
		}
		break;
	case 7:
		if (Rmod <= 0)
		{
			Rmod = 10;
		}
		break;
	case 8:
		Rmod += incrdecr;
		break;
	case 9:
		if (Rmod > incrdecr)
		{
			Rmod -= incrdecr;
		}
		break;
	case 10:
		if (incrdecr < 0)
		{
			incrdecr = 0.5;
		}
		if (Rmod < incrdecr)
		{
			Rmod = incrdecr;
		}
		break;
	case 11:
		if (res < 1 || res > 200)
		{
			res = 20;
		}
		break;
	case 12:
		p = 1;
		c = 4;
		res = 20;
		Rmod = 10;
		useRmod = 0;
		incrdecr = 0.5;
		drawCtrlPositions = 1;
		curveAffine = 0;
		curveBezier = 0;
		curveBspline = 0;
		curveInterpwDQ = 0;
		curveInterpwPD = 0;
		screwMotion = 0;
		break;
	default:
		break;
	}

	// synchronize the GLUI display with the variables:
	glui->sync_live();

}


int main(int argc, char *argv[])
{
	//Initialize Default OpenGL window 
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(500, 100);
	int win_id = glutCreateWindow("MEC 572 Final Project - Team 2");

	init();
	shellMessages();
	setupLight();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutMouseFunc(Interaction::mouseButtonEvents);
	glutMotionFunc(Interaction::mouseMotionEvents);
	glutKeyboardFunc(Interaction::keyboard);

	/*******************************************************************************/
	/*****************		Creating The GUI		********************************/
	/*******************************************************************************/

	glui = GLUI_Master.create_glui("Controls", 0, 10, 150);
	glui->set_main_gfx_window(win_id);
	GLUI_Master.set_glutIdleFunc(NULL);

	GLUI_EditText *filetxt;
	filetxt = glui->add_edittext("Input Filename: ", GLUI_EDITTEXT_TEXT, &fname, 12, selectMessage);
	filetxt->set_w(200);
	glui->add_checkbox("Uploaded Data", &drawCtrlPositions, 0);
	
	glui->add_separator();
	glui->add_separator();

	GLUI_Panel *AltMotion = glui->add_panel("Alternative Motion");
	glui->add_checkbox_to_panel(AltMotion, "Screw Motion", &screwMotion, 0);
	glui->add_checkbox_to_panel(AltMotion, "Bezier Curve", &curveBezier, 0);
	glui->add_checkbox_to_panel(AltMotion, "Affine", &curveAffine, 0);

	glui->add_column();

	GLUI_Panel *Bspline = glui->add_panel("Rational B-spline Motion");
	glui->add_checkbox_to_panel(Bspline, "B-spline", &curveBspline, 0);
	glui->add_statictext_to_panel(Bspline, "Interpolation/Approximation");
	glui->add_checkbox_to_panel(Bspline, "using Dual Quaternions", &curveInterpwDQ, 0);
	glui->add_checkbox_to_panel(Bspline, "using Polar Decomposition", &curveInterpwPD, 0);

	glui->add_edittext_to_panel(Bspline, "Degree p = ", GLUI_EDITTEXT_INT, &p, 1, selectMessage);
	glui->add_button_to_panel(Bspline, "+1", 2, selectMessage);
	glui->add_button_to_panel(Bspline, "-1", 3, selectMessage);

	glui->add_statictext_to_panel(Bspline, "Number of control points");
	glui->add_edittext_to_panel(Bspline, "n = ", GLUI_EDITTEXT_INT, &c, 4, selectMessage);
	glui->add_button_to_panel(Bspline, "+1", 5, selectMessage);
	glui->add_button_to_panel(Bspline, "-1", 6, selectMessage);

	glui->add_checkbox_to_panel(Bspline, "Manual Hyperspherical Radius", &useRmod, 0);
	glui->add_edittext_to_panel(Bspline, "R = ", GLUI_EDITTEXT_FLOAT, &Rmod, 7, selectMessage);
	glui->add_button_to_panel(Bspline, "+", 8, selectMessage);
	glui->add_button_to_panel(Bspline, "-", 9, selectMessage);
	glui->add_edittext_to_panel(Bspline, "Increment/Decrement", GLUI_EDITTEXT_FLOAT, &incrdecr, 10, selectMessage);

	glui->add_column();

	GLUI_Panel *Resolution = glui->add_panel("Resolution");
	glui->add_edittext_to_panel(Resolution, "No. of Positions", GLUI_EDITTEXT_INT, &res, 11, selectMessage);
	GLUI_HSlider *slider = glui->add_slider_to_panel(Resolution, 0, GLUI_HSLIDER_INT, &res, 11, selectMessage);
	slider->set_int_limits(1, 200);
	slider->set_slider_val(res);

	glui->add_separator();
	glui->add_separator();

	glui->add_button("Quit", 0, (GLUI_Update_CB)exit);



	/***********************************************************************/



	glutMainLoop();

	return 0;
}
