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

#include "lib/simulator.h"
//#include <cmath>
//#include <memory>
//#include <sstream>
//#include <random>
//#include <algorithm>
//#include <GL/gl.h>
//#include <GL/glu.h>
//#include <GL/glut.h>
//#include <GL/glext.h>
//#include <GL/freeglut.h>
//#include <QObject>
//#include <QWidget>
//#include <QLabel>
//#include <QGridLayout>
//#include <QApplication>
#include <QGLWidget>
//#include <QtGui>
//#include <QFont>
//#include <QPen>
//#include <QWidget>
#include <thread>
#include <mutex>
#include <unistd.h>

/*
 * ################################################################################################
 */

class sim_graphics : public QGLWidget
{
	Q_OBJECT

	public:
		sim_graphics(const sim::world_model& w_model);
		~sim_graphics();

		void update_state(const sim::world_state& state);

	protected:

	    void initializeGL() override;
	    void paintGL() override;
	    void resizeGL(int width, int height) override;

	    void render_bounds();
	    void render_world();
	    void render_actor_state(const sim::actor_state& a_state);
		void render_state(sim::world_state w_state);

	private:
		std::mutex state_mutex_;
		sim::world_state current_state_;
		const sim::world_model& w_model_;
};

class sim_graphics_wrapper
{
	public:

		sim_graphics_wrapper();
		~sim_graphics_wrapper();

		void initialize(const sim::world_model& w_model);
		void update_state(const sim::world_state& state);

	private:
		void sim_graphics_thread(const sim::world_model& w_model);

		std::shared_ptr<bool> thread_ready;
		std::thread *qt_thread;
		sim_graphics *sim_widget;
};
