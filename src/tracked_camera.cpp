#include "vive_ros/tracked_camera_openvr_sample.hpp"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    CQTrackedCameraOpenVRTest w;
    w.show();

    return a.exec();
}