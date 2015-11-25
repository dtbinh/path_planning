/**
 * simulator/graphics.h
 * Copyright 2015
 *
 * Uber Advanced Technology Center

 * Control Instructions (who can see this code):
 * Confidential.  Not for public release unless permission granted
 * by program manager.
 *
 * Usage Rights (who can use this code):
 * Usage allowed for all programs with permissions from author and
 * program manager.
 *
 * This notice must appear in all copies of this file and its derivatives.
 *
 */

#include "graphics.h"

#include <cmath>
#include <memory>
#include <sstream>
#include <random>
#include <algorithm>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glext.h>
#include <GL/freeglut.h>
#include <QObject>
#include <QWidget>
#include <QLabel>
#include <QGridLayout>
#include <QApplication>
#include <QGLWidget>
#include <QtGui>
#include <QFont>
#include <QPen>
#include <QWidget>
#include <thread>
#include <mutex>
#include <unistd.h>

#define GRAPHICS_WINODW_SIZE 1000.0

/*
 * ################################################################################################
 */

sim_graphics::sim_graphics(const sim::world_model& w_model):
	w_model_(w_model)
{
	setWindowTitle(tr("Pedestrian Simulation"));
	setGeometry(0,0,1000,1000);
}

sim_graphics::~sim_graphics() { }

void sim_graphics::update_state(const sim::world_state &state)
{
	state_mutex_.lock();
	current_state_ = state;
	state_mutex_.unlock();
	update();
}

void sim_graphics::initializeGL()
{
    qglClearColor(QColor::fromRgb(30, 30, 30, 255));
}

void sim_graphics::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	state_mutex_.lock();
	render_state(current_state_);
	state_mutex_.unlock();
}

void sim_graphics::resizeGL(int width, int height)
{
    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-15, 15, -15, 15, -10, 10);

	glFlush();
}


void sim_graphics::render_bounds()
{
	glLineWidth(5.0);
    glBegin(GL_LINE_STRIP);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    glVertex2d(-13.5,-13.5);
    glVertex2d(-13.5,13.5);
    glVertex2d(13.5,13.5);
    glVertex2d(13.5,-13.5);
    glVertex2d(-13.5,-13.5);
    glEnd();
}

void sim_graphics::render_world()
{
	// crosswalks
	for(auto& cw : w_model_.crosswalks)
	{
		auto& cross = cw.second;

		glLineWidth(1.0);
		switch(current_state_.signal_states[cross.signal_id])
		{
			case world::cw_signal::STOP: glColor3d(1.0, 0.0, 0.0); break;
			case world::cw_signal::GO: glColor3d(0.0, 1.0, 0.0); break;
			case world::cw_signal::BLINK: glColor3d(1.0, 1.0, 0.0); break;
		}

		glBegin(GL_LINES);
		{
			glVertex3d(cross.center_line.pt_1.x, cross.center_line.pt_1.y,-0.1);
			glVertex3d(cross.center_line.pt_2.x, cross.center_line.pt_2.y,-0.1);
		}
		glEnd();

	}

	// corners
	for(auto& c : w_model_.corners)
	{
		auto& corner = c.second;

		glLineWidth(1.0);
		glColor3d(0.0, 0.0, 1.0);
		glBegin(GL_LINE_LOOP);
		{
			glVertex2d(corner.bounding_box.upper_left.x, corner.bounding_box.upper_left.y);
			glVertex2d(corner.bounding_box.upper_left.x, corner.bounding_box.lower_right.y);
			glVertex2d(corner.bounding_box.lower_right.x, corner.bounding_box.lower_right.y);
			glVertex2d(corner.bounding_box.lower_right.x, corner.bounding_box.upper_left.y);
		}
		glEnd();

		glColor3d(0.4, 0.4, 0.4);
		glBegin(GL_QUADS);
		{
			glVertex2d(corner.bounding_box.upper_left.x, corner.bounding_box.upper_left.y);
			glVertex2d(corner.bounding_box.upper_left.x, corner.bounding_box.lower_right.y);
			glVertex2d(corner.bounding_box.lower_right.x, corner.bounding_box.lower_right.y);
			glVertex2d(corner.bounding_box.lower_right.x, corner.bounding_box.upper_left.y);
		}
		glEnd();

	}


}

void sim_graphics::render_actor_state(const sim::actor_state& a_state)
{
	GLUquadric *disc = gluNewQuadric();
	glPushMatrix();
	glTranslated(a_state.pose.position.x, a_state.pose.position.y, 0.1);
	double radius = 0.5;
	gluDisk(disc, 0,radius, 36,1);
	gluDeleteQuadric(disc);
	glPopMatrix();
}

void sim_graphics::render_state(sim::world_state w_state)
{
	render_bounds();
	render_world();

    glColor4f(0.0f, 1.0f, 0.0f, 1.5f);

    for(auto& a_state : w_state.actor_states)
    {
	    render_actor_state(a_state.second);
    }
}


/*
 * ################################################################################################
 */


sim_graphics_wrapper::sim_graphics_wrapper():
	thread_ready(new bool(false)),
	qt_thread(NULL),
	sim_widget(NULL)
{
}
sim_graphics_wrapper::~sim_graphics_wrapper()
{
	if(*thread_ready)
	{
		qt_thread->join();
	}
}

void sim_graphics_wrapper::initialize(const sim::world_model& w_model)
{
	//create the thread with a shared ptr and mutex
	*thread_ready = false;
	qt_thread = new std::thread(&sim_graphics_wrapper::sim_graphics_thread, this, w_model);
	while(!(*thread_ready))
	{
		usleep(0);
	}
}

void sim_graphics_wrapper::update_state(const sim::world_state &state)
{
	sim_widget->update_state(state);
}

void sim_graphics_wrapper::sim_graphics_thread(const sim::world_model& w_model)
{
    std::string executable_path = "simulation";
    int argc = 1;
    char *argv[] = { const_cast<char*>(executable_path.c_str()), NULL };

	QApplication app(argc,argv, true);
	sim_widget = new sim_graphics(w_model);
	sim_widget->show();
	*thread_ready = true;
	app.exec();
}
