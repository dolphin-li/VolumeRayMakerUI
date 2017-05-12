#include "volumemakerui.h"
#include <QFileDialog>
#include "global_data_holder.h"

VolumeMakerUI::VolumeMakerUI(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	g_dataholder.init();
	updateUiByParam();
}

VolumeMakerUI::~VolumeMakerUI()
{

}

void VolumeMakerUI::on_actionOpen_triggered()
{
	QString name = QFileDialog::getOpenFileName(this, "Open", "", "*.*");
	if (name.isEmpty())
		return;
	try
	{
		if (name.endsWith(".obj") || name.endsWith(".off"))
			openMesh(name);
		else if (name.endsWith(".dvol") || name.endsWith(".hfdvol"))
			openVolume(name);
		else if (name.endsWith(".hdf5") || name.endsWith(".hdf5_bool"))
			openHdf5(name);
		else
			printf("non supported file format!");
	}
	catch (std::exception e)
	{
		std::cout << e.what() << std::endl;
	}
}

void VolumeMakerUI::openMesh(QString fullname)
{
	std::string path, name, ext;
	ldp::fileparts(fullname.toStdString(), path, name, ext);
	if (ext == ".obj")
		g_dataholder.m_mesh.loadObj(fullname.toStdString().c_str(), true, true);
	else if (ext == ".off")
		g_dataholder.m_mesh.loadOff(fullname.toStdString().c_str(), true);
	else
		printf("non-supported mesh ext: %s\n", ext.c_str());
	g_dataholder.m_mesh_noRot = g_dataholder.m_mesh;
	ui.widget->setMesh(&g_dataholder.m_mesh);
	ui.widget->setViewType(MpuViewer::ViewTypeMesh);
	kdtree::AABB box(g_dataholder.m_mesh.boundingBox[0], g_dataholder.m_mesh.boundingBox[1]);
	ui.widget->setViewClipBox(box);
	updateUiByParam();
}

void VolumeMakerUI::openVolume(QString name)
{
	g_dataholder.m_volume.load(name.toStdString().c_str());
	global_param::volume_res[0] = g_dataholder.m_volume.getResolution()[0];
	global_param::volume_res[1] = g_dataholder.m_volume.getResolution()[1];
	global_param::volume_res[2] = g_dataholder.m_volume.getResolution()[2];
	ui.widget->setDenseVolume(g_dataholder.m_volume);
	ui.widget->setViewType(MpuViewer::ViewTypeDenseVolume);
	ui.widget->setViewClipBox(g_dataholder.m_volume.getBound());
	updateUiByParam();
}

void VolumeMakerUI::openHdf5(QString name)
{
	if (name.endsWith(".hdf5"))
		g_dataholder.loadHdf5(name.toStdString().c_str());
	else if (name.endsWith(".hdf5_bool"))
		g_dataholder.loadHdf5bool(name.toStdString().c_str());
	global_param::volume_res[0] = g_dataholder.m_hdf5dims[2];
	global_param::volume_res[1] = g_dataholder.m_hdf5dims[3];
	global_param::volume_res[2] = g_dataholder.m_hdf5dims[4];
	g_dataholder.fetchHdf5Index(0);
	ui.widget->setDenseVolume(g_dataholder.m_volume);
	ui.widget->setViewType(MpuViewer::ViewTypeDenseVolume);
	ui.widget->setViewClipBox(g_dataholder.m_volume.getBound());
	updateUiByParam();
}

void VolumeMakerUI::updateUiByParam()
{
	ldp::Float3 res = g_dataholder.m_volume.getResolution();
	ldp::Float3 vmin = ui.widget->getViewClipBox().min;
	ldp::Float3 vmax = ui.widget->getViewClipBox().max;
	ldp::Float3 bmin = g_dataholder.m_volume.getBound().min;
	ldp::Float3 bmax = g_dataholder.m_volume.getBound().max;
	vmin = (vmin - bmin) / (bmax - bmin);
	vmax = (vmax - bmin) / (bmax - bmin);
	ui.sbXres->setValue(res[0]);
	ui.sbYres->setValue(res[1]);
	ui.sbZres->setValue(res[2]);
	ui.hsXclip_b->setValue(std::lroundf(ui.hsXclip_b->maximum() * vmin[0]));
	ui.hsXclip_e->setValue(std::lroundf(ui.hsXclip_e->maximum() * vmax[0]));
	ui.hsYclip_b->setValue(std::lroundf(ui.hsYclip_b->maximum() * vmin[1]));
	ui.hsYclip_e->setValue(std::lroundf(ui.hsYclip_e->maximum() * vmax[1]));
	ui.hsZclip_b->setValue(std::lroundf(ui.hsZclip_b->maximum() * vmin[2]));
	ui.hsZclip_e->setValue(std::lroundf(ui.hsZclip_e->maximum() * vmax[2]));
	ui.sbHdf5Idx->setMaximum(g_dataholder.m_hdf5dims[0]);
}

void VolumeMakerUI::on_sbXres_valueChanged(int v)
{
	global_param::volume_res[0] = v;
}

void VolumeMakerUI::on_sbYres_valueChanged(int v)
{
	global_param::volume_res[1] = v;
}

void VolumeMakerUI::on_sbZres_valueChanged(int v)
{
	global_param::volume_res[2] = v;
}

void VolumeMakerUI::on_hsXclip_b_valueChanged(int v)
{
	kdtree::AABB box = ui.widget->getViewClipBox();
	ldp::Float3 bmin = g_dataholder.m_volume.getBound().min;
	ldp::Float3 bmax = g_dataholder.m_volume.getBound().max;
	box.min[0] = float(v) / float(ui.hsXclip_e->maximum())*(bmax[0] - bmin[0]) + bmin[0];
	ui.widget->setViewClipBox(box);
}

void VolumeMakerUI::on_hsXclip_e_valueChanged(int v)
{
	kdtree::AABB box = ui.widget->getViewClipBox();
	ldp::Float3 bmin = g_dataholder.m_volume.getBound().min;
	ldp::Float3 bmax = g_dataholder.m_volume.getBound().max;
	box.max[0] = float(v) / float(ui.hsXclip_e->maximum())*(bmax[0] - bmin[0]) + bmin[0];
	ui.widget->setViewClipBox(box);
}

void VolumeMakerUI::on_hsYclip_b_valueChanged(int v)
{
	kdtree::AABB box = ui.widget->getViewClipBox();
	ldp::Float3 bmin = g_dataholder.m_volume.getBound().min;
	ldp::Float3 bmax = g_dataholder.m_volume.getBound().max;
	box.min[1] = float(v) / float(ui.hsXclip_e->maximum())*(bmax[1] - bmin[1]) + bmin[1];
	ui.widget->setViewClipBox(box);
}

void VolumeMakerUI::on_hsYclip_e_valueChanged(int v)
{
	kdtree::AABB box = ui.widget->getViewClipBox();
	ldp::Float3 bmin = g_dataholder.m_volume.getBound().min;
	ldp::Float3 bmax = g_dataholder.m_volume.getBound().max;
	box.max[1] = float(v) / float(ui.hsXclip_e->maximum())*(bmax[1] - bmin[1]) + bmin[1];
	ui.widget->setViewClipBox(box);
}

void VolumeMakerUI::on_hsZclip_b_valueChanged(int v)
{
	kdtree::AABB box = ui.widget->getViewClipBox();
	ldp::Float3 bmin = g_dataholder.m_volume.getBound().min;
	ldp::Float3 bmax = g_dataholder.m_volume.getBound().max;
	box.min[2] = float(v) / float(ui.hsXclip_e->maximum())*(bmax[2] - bmin[2]) + bmin[2];
	ui.widget->setViewClipBox(box);
}

void VolumeMakerUI::on_hsZclip_e_valueChanged(int v)
{
	kdtree::AABB box = ui.widget->getViewClipBox();
	ldp::Float3 bmin = g_dataholder.m_volume.getBound().min;
	ldp::Float3 bmax = g_dataholder.m_volume.getBound().max;
	box.max[2] = float(v) / float(ui.hsXclip_e->maximum())*(bmax[2] - bmin[2]) + bmin[2];
	ui.widget->setViewClipBox(box);
}

void VolumeMakerUI::on_pbVolume2Mesh_clicked()
{
	try
	{
		g_dataholder.volume2mesh(g_dataholder.m_mesh, g_dataholder.m_volume);
		ui.widget->setMesh(&g_dataholder.m_mesh, false);
		ui.widget->setViewType(MpuViewer::ViewTypeMesh);
	}
	catch (std::exception e)
	{
		std::cout << e.what() << std::endl;
	}
}

void VolumeMakerUI::on_pbMesh2Volume_clicked()
{
	try
	{
		g_dataholder.mesh2volume(g_dataholder.m_volume, g_dataholder.m_mesh, g_dataholder.m_pointCloud);
		ui.widget->setDenseVolume(g_dataholder.m_volume, false);
		ui.widget->setViewType(MpuViewer::ViewTypeDenseVolume);
		ui.widget->setViewClipBox(g_dataholder.m_volume.getBound());
		updateUiByParam();
	}
	catch (std::exception e)
	{
		std::cout << e.what() << std::endl;
	}
}

void VolumeMakerUI::on_pbGenViews_clicked()
{
	try
	{
		g_dataholder.generateViews();
	}
	catch (std::exception e)
	{
		std::cout << e.what() << std::endl;
	}
}

void VolumeMakerUI::on_sbViewId_valueChanged(int v)
{
	try
	{
		global_param::view_id = v;
		g_dataholder.applyView(g_dataholder.m_mesh_noRot, g_dataholder.m_mesh, v);
		ui.widget->setMesh(&g_dataholder.m_mesh, false);
		ui.widget->updateGL();
	}
	catch (std::exception e)
	{
		std::cout << e.what() << std::endl;
	}
}

void VolumeMakerUI::on_pbBatchConfig_clicked()
{
	QString name = QFileDialog::getOpenFileName(this, "BatchConfig", "", "*.config.txt");
	if (name.isEmpty())
		return;
	try
	{
		g_dataholder.batch_config(name.toStdString().c_str());
	}
	catch (std::exception e)
	{
		std::cout << e.what() << std::endl;
	}
}

void VolumeMakerUI::on_sbHdf5Idx_valueChanged(int v)
{
	try
	{
		g_dataholder.fetchHdf5Index(v);
		ui.widget->setDenseVolume(g_dataholder.m_volume);
		ui.widget->setViewType(MpuViewer::ViewTypeDenseVolume);
		ui.widget->setViewClipBox(g_dataholder.m_volume.getBound());
		updateUiByParam();
	}
	catch (std::exception e)
	{
		std::cout << e.what() << std::endl;
	}
}
