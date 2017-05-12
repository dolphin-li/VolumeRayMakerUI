#ifndef VOLUMEMAKERUI_H
#define VOLUMEMAKERUI_H

#include <QtWidgets/QMainWindow>
#include "ui_volumemakerui.h"

class VolumeMakerUI : public QMainWindow
{
	Q_OBJECT

public:
	VolumeMakerUI(QWidget *parent = 0);
	~VolumeMakerUI();

	protected slots:
	void on_actionOpen_triggered();

	void on_sbXres_valueChanged(int v);
	void on_sbYres_valueChanged(int v);
	void on_sbZres_valueChanged(int v);

	void on_hsXclip_b_valueChanged(int v);
	void on_hsXclip_e_valueChanged(int v);
	void on_hsYclip_b_valueChanged(int v);
	void on_hsYclip_e_valueChanged(int v);
	void on_hsZclip_b_valueChanged(int v);
	void on_hsZclip_e_valueChanged(int v);

	void on_pbVolume2Mesh_clicked();
	void on_pbMesh2Volume_clicked();
	void on_pbBinarizeVolume_clicked();

	void on_pbGenViews_clicked();
	void on_sbViewId_valueChanged(int v);

	void on_pbBatchConfig_clicked();

	void on_sbHdf5Idx_valueChanged(int v);
protected:
	void openMesh(QString name);
	void openVolume(QString name);
	void openHdf5(QString name);
	void updateUiByParam();
private:
	Ui::VolumeMakerUIClass ui;
};

#endif // VOLUMEMAKERUI_H
