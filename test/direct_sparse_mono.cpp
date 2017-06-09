#include <fstream>
#include "Config.h"
#include "Camera.h"
#include "Memory.h"
#include "Tracker.h"

using namespace std;


int main ( int argc, char** argv )
{
    google::InstallFailureSignalHandler();
    if (argc < 2)
    {
        std::cout << "usage: ./direct_sparse_mono <path-to-TUM-dataset>" << std::endl;
        return 1;
    }

    std::string folder = std::string(argv[1]);
    std::ifstream fin(folder + "/associate.txt");
    std::vector<string> rgbFiles, depthFiles;
    vector<double> rgbTimestamp, depthTimestamp;
    while ( !fin.eof() )
    {
        double timestampRGB, timestampDepth;
        std::string rgbFile, depthFile;
        fin >> timestampRGB >> rgbFile >> timestampDepth >> depthFile;

        rgbFiles.push_back(rgbFile);
        depthFiles.push_back(depthFile);

        if (!fin.good()) break;
    }

    Config::SetConfigurationFile("../config/tum_fr1_rgbd.yaml");
    //std::shared_ptr<PinholeCamera> camera = std::make_shared<PinholeCamera>();
    PinholeCamera* camera = new PinholeCamera();
    camera->PrintCameraParameters();

  //  Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();
  //  Sophus::SE3 Tcw = Sophus::SE3();
  //  LOG(INFO) << std::endl << "Init Tcw: " << std::endl << Tcw.matrix() << std::endl;

    std::shared_ptr<Tracker> pTracker = std::make_shared<Tracker>(camera);

    for(size_t f = 0; f < rgbFiles.size(); f++)
    {
        if (f < 100) continue;
        if (f > 200) break;

        LOG(INFO) << "image " << f << std::endl;
        cv::Mat color = cv::imread(folder + "/" + rgbFiles[f]);
        if (color.empty()) continue;
        cv::Mat depth = cv::imread(folder + "/" + depthFiles[f], cv::IMREAD_UNCHANGED);

        cv::Mat undistorted;
        camera->UndistortImage(color, undistorted);
        Frame* frame = Memory::CreateFrame();
        frame->InitFrame(undistorted, camera, f == 0);

        //cv::imshow("curr", undistored);
        //cv::waitKey(30);
        //cv::destroyWindow("curr");

        LOG(INFO) << "feature.size():" << frame->_features.size() << std::endl;

        if (f == 0) // 1st frame as ref_frame and generate 3d points
        {
            //frame->_TCW = Sophus::SE3( Sophus::SO3(0.0,0.0,0.0), Eigen::Vector3d(1.0,1.0,1.0) );
            for(Feature* pFeature : frame->_features)
            {
                ushort d = depth.ptr<ushort>( cvRound(pFeature->_pixel[1]) )[ cvRound(pFeature->_pixel[0]) ];
                if (d == 0) continue;

                MapPoint* mp = Memory::CreateMapPoint();
                mp->_pos_world = camera->Pixel2Camera(pFeature->_pixel, d / camera->scale());
           //     LOG(INFO) << "pos_world:" << mp->_pos_world;

                pFeature->_depth = d / camera->scale();
                pFeature->_mappoint = mp;
            }

            pTracker->SetReference(frame);
            LOG(INFO)<<"1st pose:\n"<< frame->_TCW.matrix()<< std::endl;
            continue;
        }

        pTracker->TrackRefFrame(frame);

        LOG(INFO) << "Current pose esitmated by SVO image align:\n" << frame->_TCW.matrix() << std::endl;
        LOG(INFO) << "CamCenter: \n" << frame->GetCamCenter() << std::endl;
        break;
    }

    delete camera;

    return 0;
}
