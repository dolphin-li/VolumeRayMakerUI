/********************************************************************************
** Form generated from reading UI file 'volumemakerui.ui'
**
** Created by: Qt User Interface Compiler version 5.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VOLUMEMAKERUI_H
#define UI_VOLUMEMAKERUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "mpuviewer.h"

QT_BEGIN_NAMESPACE

class Ui_VolumeMakerUIClass
{
public:
    QAction *actionOpen;
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    MpuViewer *widget;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents;
    QGroupBox *groupBox;
    QSpinBox *sbXres;
    QSpinBox *sbYres;
    QSpinBox *sbZres;
    QSlider *hsXclip_b;
    QSlider *hsYclip_b;
    QSlider *hsZclip_b;
    QSlider *hsXclip_e;
    QSlider *hsYclip_e;
    QSlider *hsZclip_e;
    QPushButton *pbVolume2Mesh;
    QPushButton *pbMesh2Volume;
    QGroupBox *groupBox_2;
    QSpinBox *sbViewId;
    QLabel *label;
    QPushButton *pbGenViews;
    QPushButton *pbBatchConfig;
    QGroupBox *groupBox_3;
    QSpinBox *sbHdf5Idx;

    void setupUi(QMainWindow *VolumeMakerUIClass)
    {
        if (VolumeMakerUIClass->objectName().isEmpty())
            VolumeMakerUIClass->setObjectName(QStringLiteral("VolumeMakerUIClass"));
        VolumeMakerUIClass->resize(1087, 829);
        actionOpen = new QAction(VolumeMakerUIClass);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        centralWidget = new QWidget(VolumeMakerUIClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        widget = new MpuViewer(centralWidget);
        widget->setObjectName(QStringLiteral("widget"));

        gridLayout->addWidget(widget, 0, 0, 1, 1);

        VolumeMakerUIClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(VolumeMakerUIClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1087, 23));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        VolumeMakerUIClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(VolumeMakerUIClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        VolumeMakerUIClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(VolumeMakerUIClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        VolumeMakerUIClass->setStatusBar(statusBar);
        dockWidget = new QDockWidget(VolumeMakerUIClass);
        dockWidget->setObjectName(QStringLiteral("dockWidget"));
        dockWidget->setMinimumSize(QSize(200, 38));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QStringLiteral("dockWidgetContents"));
        groupBox = new QGroupBox(dockWidgetContents);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(0, 0, 201, 281));
        sbXres = new QSpinBox(groupBox);
        sbXres->setObjectName(QStringLiteral("sbXres"));
        sbXres->setGeometry(QRect(10, 20, 51, 22));
        sbXres->setMaximum(9999);
        sbYres = new QSpinBox(groupBox);
        sbYres->setObjectName(QStringLiteral("sbYres"));
        sbYres->setGeometry(QRect(70, 20, 51, 22));
        sbYres->setMaximum(9999);
        sbZres = new QSpinBox(groupBox);
        sbZres->setObjectName(QStringLiteral("sbZres"));
        sbZres->setGeometry(QRect(130, 20, 51, 22));
        sbZres->setMaximum(9999);
        hsXclip_b = new QSlider(groupBox);
        hsXclip_b->setObjectName(QStringLiteral("hsXclip_b"));
        hsXclip_b->setGeometry(QRect(10, 50, 171, 22));
        hsXclip_b->setOrientation(Qt::Horizontal);
        hsYclip_b = new QSlider(groupBox);
        hsYclip_b->setObjectName(QStringLiteral("hsYclip_b"));
        hsYclip_b->setGeometry(QRect(10, 140, 171, 22));
        hsYclip_b->setOrientation(Qt::Horizontal);
        hsZclip_b = new QSlider(groupBox);
        hsZclip_b->setObjectName(QStringLiteral("hsZclip_b"));
        hsZclip_b->setGeometry(QRect(10, 230, 171, 22));
        hsZclip_b->setOrientation(Qt::Horizontal);
        hsXclip_e = new QSlider(groupBox);
        hsXclip_e->setObjectName(QStringLiteral("hsXclip_e"));
        hsXclip_e->setGeometry(QRect(10, 80, 171, 22));
        hsXclip_e->setValue(99);
        hsXclip_e->setOrientation(Qt::Horizontal);
        hsYclip_e = new QSlider(groupBox);
        hsYclip_e->setObjectName(QStringLiteral("hsYclip_e"));
        hsYclip_e->setGeometry(QRect(10, 170, 171, 22));
        hsYclip_e->setValue(99);
        hsYclip_e->setOrientation(Qt::Horizontal);
        hsZclip_e = new QSlider(groupBox);
        hsZclip_e->setObjectName(QStringLiteral("hsZclip_e"));
        hsZclip_e->setGeometry(QRect(10, 260, 171, 22));
        hsZclip_e->setValue(99);
        hsZclip_e->setOrientation(Qt::Horizontal);
        pbVolume2Mesh = new QPushButton(dockWidgetContents);
        pbVolume2Mesh->setObjectName(QStringLiteral("pbVolume2Mesh"));
        pbVolume2Mesh->setGeometry(QRect(10, 310, 75, 23));
        pbMesh2Volume = new QPushButton(dockWidgetContents);
        pbMesh2Volume->setObjectName(QStringLiteral("pbMesh2Volume"));
        pbMesh2Volume->setGeometry(QRect(100, 310, 75, 23));
        groupBox_2 = new QGroupBox(dockWidgetContents);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 390, 171, 121));
        sbViewId = new QSpinBox(groupBox_2);
        sbViewId->setObjectName(QStringLiteral("sbViewId"));
        sbViewId->setGeometry(QRect(20, 20, 42, 22));
        sbViewId->setMaximum(9999);
        label = new QLabel(groupBox_2);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(80, 20, 46, 13));
        pbGenViews = new QPushButton(groupBox_2);
        pbGenViews->setObjectName(QStringLiteral("pbGenViews"));
        pbGenViews->setGeometry(QRect(10, 60, 75, 23));
        pbBatchConfig = new QPushButton(dockWidgetContents);
        pbBatchConfig->setObjectName(QStringLiteral("pbBatchConfig"));
        pbBatchConfig->setGeometry(QRect(10, 700, 75, 23));
        groupBox_3 = new QGroupBox(dockWidgetContents);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(10, 530, 120, 80));
        sbHdf5Idx = new QSpinBox(groupBox_3);
        sbHdf5Idx->setObjectName(QStringLiteral("sbHdf5Idx"));
        sbHdf5Idx->setGeometry(QRect(10, 20, 42, 22));
        sbHdf5Idx->setMaximum(0);
        dockWidget->setWidget(dockWidgetContents);
        VolumeMakerUIClass->addDockWidget(static_cast<Qt::DockWidgetArea>(1), dockWidget);

        menuBar->addAction(menuFile->menuAction());
        menuFile->addAction(actionOpen);

        retranslateUi(VolumeMakerUIClass);

        QMetaObject::connectSlotsByName(VolumeMakerUIClass);
    } // setupUi

    void retranslateUi(QMainWindow *VolumeMakerUIClass)
    {
        VolumeMakerUIClass->setWindowTitle(QApplication::translate("VolumeMakerUIClass", "VolumeMakerUI", 0));
        actionOpen->setText(QApplication::translate("VolumeMakerUIClass", "open", 0));
        menuFile->setTitle(QApplication::translate("VolumeMakerUIClass", "file", 0));
        groupBox->setTitle(QApplication::translate("VolumeMakerUIClass", "Volume", 0));
        pbVolume2Mesh->setText(QApplication::translate("VolumeMakerUIClass", "Volume2Mesh", 0));
        pbMesh2Volume->setText(QApplication::translate("VolumeMakerUIClass", "Mesh2Volume", 0));
        groupBox_2->setTitle(QApplication::translate("VolumeMakerUIClass", "ViewSamples", 0));
        label->setText(QApplication::translate("VolumeMakerUIClass", "viewId", 0));
        pbGenViews->setText(QApplication::translate("VolumeMakerUIClass", "GenViews", 0));
        pbBatchConfig->setText(QApplication::translate("VolumeMakerUIClass", "BatchConfig", 0));
        groupBox_3->setTitle(QApplication::translate("VolumeMakerUIClass", "hdf5", 0));
    } // retranslateUi

};

namespace Ui {
    class VolumeMakerUIClass: public Ui_VolumeMakerUIClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VOLUMEMAKERUI_H
