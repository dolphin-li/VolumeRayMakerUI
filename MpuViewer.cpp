#include "MpuViewer.h"
#include "glut.h"
#include "util.h"
#include "global_data_holder.h"


const static float id8[3][8] =
{
	1, -1, 1, 1, 1, -1, -1, -1,
	1, -1, -1, 1, -1, 1, -1, 1,
	1, -1, -1, -1, 1, 1, 1, -1
};
const static int id12[2][12] =
{
	0, 3, 2, 4, 1, 6, 7, 7, 1, 4, 5, 7,
	3, 2, 4, 0, 6, 5, 5, 1, 2, 6, 0, 3
};

//#define TEST_DENSE_VISUALIZE

MpuViewer::MpuViewer(QWidget *parent)
: QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
	m_viewType = ViewTypeDenseVolume;
	m_pMesh = nullptr;
	m_pDenseVolume = nullptr;
	m_meshShowType = Renderable::SW_F | Renderable::SW_FLAT | Renderable::SW_LIGHTING | Renderable::SW_V;

	m_defaultCameraLocation = ldp::Float3(0, 0, 2);
	m_defaultCameraDirection = ldp::Float3(0, 0, -1);
	m_defaultCameraUp = ldp::Float3(0, 1, 0);
	m_dataScale = 1.f;

	setMouseTracking(true);
}

MpuViewer::~MpuViewer()
{

}

void MpuViewer::setViewType(ViewType type)
{
	m_viewType = type;
	updateGL();
}

void MpuViewer::initializeGL()
{
	makeCurrent();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
#ifdef TEST_DENSE_VISUALIZE
	glEnable(GL_MULTISAMPLE);
#endif
	m_camera.lookAt(m_defaultCameraLocation, m_defaultCameraLocation + m_defaultCameraDirection, m_defaultCameraUp);
}

void MpuViewer::resizeGL(int w, int h)
{
	float aspect = w / (float)(h ? h : 1);
	m_camera.setViewPort(0, w, 0, h);
	m_camera.setPerspective(30, aspect, 0.001, 100);
}

void MpuViewer::paintGL()
{
	m_camera.apply();

	float s = 0.3f * m_camera.getScalar().length();
	ldp::Float3 sv = s;
	glLightfv(GL_LIGHT0, GL_DIFFUSE, sv.ptr());

#ifdef TEST_DENSE_VISUALIZE
	//glClearColor(1.f, 1.f, 1.f, 0.0f);
	glClearColor(0.f, 0.f, 0.f, 0.0f);
	glEnable(GL_LINE_SMOOTH);
#else
	glClearColor(0.f, 0.f, 0.f, 0.0f);
#endif
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	switch (m_viewType)
	{
	case MpuViewer::ViewTypeMesh:
		renderMesh();
		break;
	case MpuViewer::ViewTypeDenseVolume:
		renderDenseVolume();
		break;
	default:
		break;
	}

#ifndef TEST_DENSE_VISUALIZE
	// render the axis
	{
		Camera axisCam = m_camera;
		axisCam.setScalar(1);
		axisCam.setViewPort(0, width() / 4, height()*3 / 4, height());
		axisCam.setLocation(axisCam.getLocation().normalize()*1.2);
		axisCam.apply();
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);
		glEnable(GL_LINE_SMOOTH);
		glLineWidth(4);
		glBegin(GL_LINES);
		glColor3f(1, 0, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(1, 0, 0);
		glColor3f(0, 1, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 1, 0);
		glColor3f(0, 0, 1);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 1);
		glEnd();
		glPopAttrib();
	}
#endif
}

void MpuViewer::setMesh(ObjMesh* mesh, bool updateCamera)
{
	m_pMesh = mesh;

	if (updateCamera)
	{
		kdtree::AABB box(m_pMesh->boundingBox[0], m_pMesh->boundingBox[1]);
		m_dataScale = box.getExtents().length();
		m_camera.arcballSetCenter(box.getCenter());
		m_defaultCameraLocation = 0.f - m_defaultCameraDirection*m_dataScale;
	}	
	updateGL();
}

void MpuViewer::setDenseVolume(mpu::VolumeData& volume, bool updateCamera)
{
	m_pDenseVolume = &volume;

	// update the camera if necessary==============================================
	kdtree::AABB box = m_pDenseVolume->getBound();

	m_dataScale = box.getExtents().length();
	m_defaultCameraLocation = 0.f - m_defaultCameraDirection*m_dataScale;
	if (updateCamera)
	{
		m_camera.arcballSetCenter(box.getCenter());
	}

	updateGL();
}

void MpuViewer::setViewClipBox(kdtree::AABB box)
{
	m_viewClipBox = box;
	updateGL();
}

void MpuViewer::renderMesh()
{
	int typeNoV = m_meshShowType;
	if (typeNoV & Renderable::SW_V)
		typeNoV -= Renderable::SW_V;

	glColor3f(1, 1, 1);
	if(m_pMesh)
		m_pMesh->render(typeNoV);

	if (m_meshShowType & Renderable::SW_V)
	{
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glBegin(GL_POINTS);
		for (size_t i = 0; i < g_dataholder.m_pointCloud.vertex_list.size(); i++)
		{
			glNormal3fv(g_dataholder.m_pointCloud.vertex_normal_list[i].ptr());
			glVertex3fv(g_dataholder.m_pointCloud.vertex_list[i].ptr());
		}
		glEnd();
		glPopAttrib();
	} // end V
}

mpu::VolumeData* MpuViewer::getCurrentDenseVolume()
{
	return m_pDenseVolume;
}

void MpuViewer::renderDenseVolume()
{
	mpu::VolumeData* volume = getCurrentDenseVolume();
	if (volume == nullptr)
		return;

	// generate the volume for visualization======================================
	ldp::Float3 bmin = m_viewClipBox.min;
	ldp::Float3 bmax = m_viewClipBox.max;
	ldp::Float3 bsz = bmax - bmin;
	const float step = volume->getVoxelSize();

	if (bsz[0] < std::numeric_limits<float>::epsilon() 
		|| bsz[1] < std::numeric_limits<float>::epsilon() 
		|| bsz[2] < std::numeric_limits<float>::epsilon())
		return;

	// six surfaces
	const static ldp::Int4 surfaces[6] = {
		ldp::Int4(0, 1, 2, 3),
		ldp::Int4(4, 7, 6, 5),
		ldp::Int4(0, 3, 7, 4),
		ldp::Int4(1, 5, 6, 2),
		ldp::Int4(0, 4, 5, 1),
		ldp::Int4(3, 2, 6, 7)
	};
	const int gridNumXYZ[3] =
	{
		std::lroundf(bsz[0] / step),
		std::lroundf(bsz[1] / step),
		std::lroundf(bsz[2] / step)
	};
	const ldp::Int2 gridXYPlane[6] = {
		ldp::Int2(gridNumXYZ[0], gridNumXYZ[1]),
		ldp::Int2(gridNumXYZ[0], gridNumXYZ[1]),
		ldp::Int2(gridNumXYZ[0], gridNumXYZ[2]),
		ldp::Int2(gridNumXYZ[0], gridNumXYZ[2]),
		ldp::Int2(gridNumXYZ[1], gridNumXYZ[2]),
		ldp::Int2(gridNumXYZ[1], gridNumXYZ[2])
	};

	// 8 coner points
	ldp::Float3 corners[8];
	corners[0] = bmin;
	corners[1] = corners[0] + ldp::Float3(0, bsz[1], 0);
	corners[2] = corners[0] + ldp::Float3(bsz[0], bsz[1], 0);
	corners[3] = corners[0] + ldp::Float3(bsz[0], 0, 0);
	corners[4] = bmin + ldp::Float3(0, 0, bsz[2]);
	corners[5] = corners[4] + ldp::Float3(0, bsz[1], 0);
	corners[6] = corners[4] + ldp::Float3(bsz[0], bsz[1], 0);
	corners[7] = corners[4] + ldp::Float3(bsz[0], 0, 0);

	ldp::Float3 stepXYofPlane[6][2] =
	{
		{ ldp::Float3(step, 0, 0), ldp::Float3(0, step, 0) },
		{ ldp::Float3(step, 0, 0), ldp::Float3(0, step, 0) },
		{ ldp::Float3(step, 0, 0), ldp::Float3(0, 0, step) },
		{ ldp::Float3(step, 0, 0), ldp::Float3(0, 0, step) },
		{ ldp::Float3(0, step, 0), ldp::Float3(0, 0, step) },
		{ ldp::Float3(0, step, 0), ldp::Float3(0, 0, step) }
	};

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1., 1.);

	// render faces===============================================
	if (m_meshShowType & Renderable::SW_F)
	{
		glBegin(GL_QUADS);
		for (int i_surface = 0; i_surface < 6; i_surface++)
		{
			ldp::Float3 C = corners[surfaces[i_surface][0]];
			int nx = gridXYPlane[i_surface][0];
			int ny = gridXYPlane[i_surface][1];
			ldp::Float3 stepx = stepXYofPlane[i_surface][0];
			ldp::Float3 stepy = stepXYofPlane[i_surface][1];

			for (int iy = 0; iy < ny; iy++)
			{
				for (int ix = 0; ix < nx; ix++)
				{
					ldp::Float3 p[4], cl[4];
					p[0] = C + stepy * iy + stepx * ix;
					p[1] = p[0] + stepy;
					p[2] = p[1] + stepx;
					p[3] = p[0] + stepx;

					for (int k = 0; k < 4; k++)
					for (int kc = 0; kc < 3; kc++)
						p[k][kc] = std::min(bmax[kc], std::max(bmin[kc], p[k][kc]));
					
					ldp::Float3 gid = volume->getVolumeIndexFromWorldPos(p[0]);
					ldp::Float3 color;
					{
						float d = mpu::INVALID_CELL_VALUE;
						if (volume->contains(gid))
							d = (*volume).trilinear_at(gid);
						if (mpu::is_hole_cell_value(d))
						{
							color = 1.f;
						}
						else
						{
							d = (d + 1.f) / 2.f;
							color = calcTemperatureJet(d);
						}
					}

					glColor3fv(color.ptr());
					for (int k = 0; k < 4; k++)
						glVertex3fv(p[k].ptr());
				}// ix
			}// iy
		}// end for i_surface
		glEnd();
	}// end if show faces

	// render edges===============================================
	if (m_meshShowType & Renderable::SW_E)
	{
		std::vector<ldp::Float4> tarCenter_vals;

#ifdef TEST_DENSE_VISUALIZE
		glLineWidth(2);
#endif
		glBegin(GL_LINES);
		for (int i_surface = 0; i_surface < 6; i_surface++)
		{
			ldp::Float3 C = corners[surfaces[i_surface][0]];
			int nx = gridXYPlane[i_surface][0];
			int ny = gridXYPlane[i_surface][1];
			ldp::Float3 stepx = stepXYofPlane[i_surface][0];
			ldp::Float3 stepy = stepXYofPlane[i_surface][1];

#ifdef TEST_DENSE_VISUALIZE
			stepx *= 16;
			stepy *= 16;
#endif

			for (int iy = 0; iy < ny; iy++)
			{
				for (int ix = 0; ix < nx; ix++)
				{
					ldp::Float3 p[4], cl[4];
					p[0] = C + stepy * iy + stepx * ix;
					p[1] = p[0] + stepy;
					p[2] = p[1] + stepx;
					p[3] = p[0] + stepx;

					for (int k = 0; k < 4; k++)
					for (int kc = 0; kc < 3; kc++)
						p[k][kc] = std::min(bmax[kc], std::max(bmin[kc], p[k][kc]));

					for (int k1 = 3, k = 0; k < 4; k1 = k++)
					{
						glColor3f(0.4, 0.6, 0.8);
						glVertex3fv(p[k].ptr());
						glVertex3fv(p[k1].ptr());
					}
				}// ix
			}// iy
		}// end for i_surface
		glEnd();

		glPointSize(3);
		glBegin(GL_POINTS);
		for (int i = 0; i < tarCenter_vals.size(); i++)
		{
			float d = tarCenter_vals[i][3];
			d = (d + 1.f) / 2.f;
			ldp::Float3 color = calcTemperatureJet(d);
			glColor3fv(color.ptr());
			glVertex3fv(tarCenter_vals[i].ptr());
		}
		glEnd();
	}// end if show edges

	glPopAttrib();

	if (m_meshShowType & Renderable::SW_N)
	{
		int oldType = m_meshShowType;
		//m_meshShowType |= Renderable::SW_F;
		renderMesh();
		m_meshShowType = oldType;
	}

	return;
}

void MpuViewer::toggleShowType(int type)
{
	if (m_meshShowType & type)
		m_meshShowType -= type;
	else
		m_meshShowType |= type;
}

void MpuViewer::mousePressEvent(QMouseEvent *ev)
{
	setFocus();
	m_lastPos = ev->pos();
	m_buttons = ev->buttons();
	m_lastMousePressPos = ev->pos();

	if (ev->button() == Qt::MouseButton::LeftButton)
	{
		// mesh roate begin
		m_camera.arcballClick(ldp::Float2(ev->pos().x(), ev->pos().y()));
		updateGL();
	}

	// move operation
	if (ev->modifiers() == Qt::NoModifier)
	{
		if (ev->button() == Qt::MouseButton::MiddleButton)
		{
			m_camera.lookAt(m_defaultCameraLocation, m_defaultCameraLocation + m_defaultCameraDirection, m_defaultCameraUp);
			m_camera.setScalar(1);
			m_camera.arcballSetCenter(m_viewClipBox.getCenter());
			updateGL();
		}
	}
}

void MpuViewer::keyPressEvent(QKeyEvent*ev)
{
	bool noMod = ((ev->modifiers() & Qt::SHIFT) == 0)
		&& ((ev->modifiers() & Qt::CTRL) == 0)
		& ((ev->modifiers() & Qt::ALT) == 0);
	switch (ev->key())
	{
	case Qt::Key_S:
		toggleShowType(Renderable::SW_SMOOTH);
		toggleShowType(Renderable::SW_FLAT);
		break;
	case Qt::Key_E:
		toggleShowType(Renderable::SW_E);
		break;
	case Qt::Key_F:
		toggleShowType(Renderable::SW_F);
		break;
	case Qt::Key_V:
		toggleShowType(Renderable::SW_V);
		break;
	case Qt::Key_L:
		toggleShowType(Renderable::SW_LIGHTING);
		break;
	case Qt::Key_T:
		toggleShowType(Renderable::SW_TEXTURE);
		break;
	case Qt::Key_N:
		toggleShowType(Renderable::SW_N);
		break;
	case Qt::Key_Space:
		m_viewType = ViewType((int(m_viewType) + 1) % int(ViewTypeDenseEnd));
		break;
	default:
		break;
	}
	updateGL();
}

void MpuViewer::mouseReleaseEvent(QMouseEvent *ev)
{
	// clear buttons
	m_buttons = Qt::NoButton;

	// backup last position
	m_lastPos = ev->pos();
}

void MpuViewer::mouseMoveEvent(QMouseEvent*ev)
{
	// move operation
	if (ev->modifiers() == Qt::NoModifier)
	{
		if (m_buttons == Qt::MouseButton::LeftButton)
		{
			m_camera.arcballDrag(ldp::Float2(ev->pos().x(), ev->pos().y()));
		}
	}

	if (m_buttons == Qt::MouseButton::RightButton)
	{
		QPoint dif = ev->pos() - m_lastPos;
		ldp::Float3 t(-(float)dif.x() / width(), (float)dif.y() / height(), 0);
		m_camera.translate(t * m_dataScale);
	}

	// backup last position
	m_lastPos = ev->pos();

	updateGL();
}

void MpuViewer::wheelEvent(QWheelEvent*ev)
{
	float s = 1.2;
	if (ev->delta() < 0)
		s = 1 / s;

	m_camera.scale(s);
	updateGL();
}
