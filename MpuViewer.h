#pragma once

#include <QtOpenGL>
#include "Camera.h"
#include "ObjMesh.h"
#include "mpu\VolumeData.h"
class MpuViewer : public QGLWidget
{
	Q_OBJECT
public:
	enum ViewType
	{
		ViewTypeMesh,
		ViewTypeDenseVolume,
		ViewTypeDenseEnd,
	};
public:
	MpuViewer(QWidget *parent);
	~MpuViewer();

	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();

	Camera& camera(){ return m_camera; }
	const Camera& camera()const{ return m_camera; }

	void setViewType(ViewType type);
	ViewType getViewType()const{ return m_viewType; }
	mpu::VolumeData* getCurrentDenseVolume();
	void setDenseVolume(mpu::VolumeData& volume, bool updateCamera = true);
	void setMesh(ObjMesh* mesh, bool updateCamera = true);

	void setViewClipBox(kdtree::AABB box);
	kdtree::AABB getViewClipBox(){ return m_viewClipBox; }
protected:
	void mousePressEvent(QMouseEvent *);
	void mouseReleaseEvent(QMouseEvent *);
	void mouseMoveEvent(QMouseEvent*);
	void wheelEvent(QWheelEvent*);
	void keyPressEvent(QKeyEvent*);

	void renderMesh();
	void renderDenseVolume();

	void toggleShowType(int type);
protected:
	// camera related
	Camera m_camera;
	ldp::Float3 m_defaultCameraLocation;
	ldp::Float3 m_defaultCameraDirection;
	ldp::Float3 m_defaultCameraUp;
	Qt::MouseButtons m_buttons;
	QPoint m_lastPos, m_lastMousePressPos;

	ViewType m_viewType;
	int m_meshShowType;
	kdtree::AABB m_viewClipBox;

	// data related
	ObjMesh* m_pMesh;
	mpu::VolumeData* m_pDenseVolume;
	float m_dataScale;
};