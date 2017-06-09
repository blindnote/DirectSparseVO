//
// Created by rochelle on 17-5-19.
//

#include <fstream>
#include "Config.h"
#include "Camera.h"
#include "Memory.h"
#include "Detector.h"
#include "Tracker.h"
#include "Utils.h"

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
    //std::ifstream fin(folder + "/associate.txt");
    std::ifstream fin(folder + "/associate_with_groundtruth.txt");
    std::vector<string> rgbFiles, depthFiles;
    vector<double> rgbTimestamp, depthTimestamp;
    vector<string> gtTimestamp;
   // std::vector<Eigen::Matrix<double,4,4>> gtPoses;
    std::vector<Sophus::SE3> gtPoses;
    while ( !fin.eof() )
    {
        double timestampRGB, timestampDepth;
        std::string rgbFile, depthFile;
        fin >> timestampRGB >> rgbFile >> timestampDepth >> depthFile;
        rgbFiles.push_back(rgbFile);
        depthFiles.push_back(depthFile);

        string timestampGroundTruth;
        fin >> timestampGroundTruth;
//        Translation translation;
//        Quaternion quaternion;
//        fin >> timestampGroundTruth >> translation.x >> translation.y >> translation.z;
//        fin >> quaternion.q1 >> quaternion.q2 >> quaternion.q3 >> quaternion.q0;
        gtTimestamp.push_back(timestampGroundTruth);
//        gtPoses.push_back(TransQua2Mat(translation, quaternion));

        double tx, ty, tz, qx, qy, qz, qw;
        fin >> tx >> ty >> tz;
        fin >> qx >> qy >> qz >> qw;
        Eigen::Vector3d t_ = Eigen::Vector3d(tx, ty, tz);
        Eigen::Quaterniond q_ = Eigen::Quaterniond(qw, qx, qy, qz);
        q_.normalize();
        gtPoses.push_back(Sophus::SE3(q_, t_));

        if (!fin.good()) break;
    }

    std::ofstream fout(folder + "/estimated_poses.txt");

    Config::SetConfigurationFile("../config/tum_fr1_rgbd.yaml");
    PinholeCamera* camera = new PinholeCamera();
    camera->PrintCameraParameters();

    std::shared_ptr<Tracker> pTracker = std::make_shared<Tracker>(camera);
    Frame* prevFrame = nullptr;

    //for(size_t f = 0; f < rgbFiles.size(); f++)
    for(size_t f = 0; f < 2; f++)
    {
        LOG(INFO) << "=============================================================================" << endl;
        LOG(INFO) << "Reading [" << f << "]: " << rgbFiles[f] << ", " << depthFiles[f] << std::endl;
        cv::Mat color = cv::imread(folder + "/" + rgbFiles[f]);
        if (color.empty()) continue;
        cv::Mat depth = cv::imread(folder + "/" + depthFiles[f], cv::IMREAD_UNCHANGED);

        cv::Mat undistorted;
 //       camera->UndistortImage(color, undistorted);
        Frame* frame = Memory::CreateFrame();
        frame->InitFrame(color, camera, f == 0);

        if (f == 0) // 1st frame as ref_frame and generate 3d points
        {
 //           frame->_TCW = Sophus::SE3( Sophus::SO3(0.0,0.0,0.0), Eigen::Vector3d(1.0,1.0,1.0) );
//            frame->_TCW = Sophus::SE3(gtPoses[f].block<3, 3>(0, 0),
//                                      Eigen::Vector3d(gtPoses[f].block<3, 1>(0, 3))).inverse();
              frame->_TCW = gtPoses[f].inverse();

            Detector detector;
            detector.Detect(frame);
            LOG(INFO) << "feature.size():" << frame->_features.size() << std::endl;
            int cnt_mp = 0;
            std::ofstream fout_fea(folder + "/my_feas.txt");
            for(Feature* pFeature : frame->_features)
            {
                ushort d = depth.ptr<ushort>( cvRound(pFeature->_pixel[1]) )[ cvRound(pFeature->_pixel[0]) ];
                if (d == 0) continue;

                fout_fea << pFeature->_level << " " << pFeature->_pixel[0] << " " << pFeature->_pixel[1] << " " << d/5000.0 << std::endl;
                pFeature->_depth = d / camera->scale();
                MapPoint* mp = Memory::CreateMapPoint();
                mp->_pos_world = camera->Pixel2World(pFeature->_pixel, frame->_TCW, pFeature->_depth);
//                /LOG(INFO) << "pos_world:" << mp->_pos_world.transpose();
                pFeature->_mappoint = mp;
                cnt_mp++;
            }
            LOG(INFO) << "Set "<<cnt_mp<<" map points. "<<endl;
            fout_fea.close();

            pTracker->SetReference(frame);

            prevFrame = frame;

            Eigen::Vector3d pose = frame->GetCamCenter();
            std::cout << gtTimestamp[f] << " " << pose[0] << " " << pose[1] << " " << pose[2] << std::endl;
            continue;
        }
        else
        {
            frame->_TCW = prevFrame->_TCW;
        }

        LOG(INFO) << "Before alignment: TCW:" << std::endl << frame->_TCW.matrix() << std::endl;
        pTracker->TrackRefFrame(frame);

        LOG(INFO) << "Current pose esitmated by SVO image align:\n" << frame->_TCW.matrix().inverse() << std::endl;
        Eigen::Vector3d espose = frame->GetCamCenter();
     //   Eigen::Vector3d gtpose = Eigen::Vector3d(gtPoses[f].block<3, 1>(0, 3));
        Eigen::Vector3d gtpose = gtPoses[f].translation();
        Eigen::Vector3d poseErr = espose - gtpose;

        LOG(INFO) << "Error: estimated(" << espose[0] << ", " << espose[1] << ", " << espose[2] << ") - ";
        LOG(INFO) << "ground truth (" << gtpose[0] << ", " << gtpose[1] << ", " << gtpose[2] << ") =";
        LOG(INFO) <<"(" << poseErr[0] << ", " << poseErr[1] << ", " << poseErr[2] << ") = " << poseErr.norm() << std::endl;
        //break;

//        Sophus::SE3 T_f_gt = frame->_TCW * Sophus::SE3(gtPoses[f].block<3, 3>(0, 0),
//                                                       Eigen::Vector3d(gtPoses[f].block<3, 1>(0, 3)));
        Sophus::SE3 T_f_gt = frame->_TCW * gtPoses[f];
        LOG(INFO) << "T_f_gt:(" << T_f_gt.translation()[0] << ", " << T_f_gt.translation()[1] << ", " << T_f_gt.translation()[2] << ")" << std::endl;
        double err = T_f_gt.translation().norm();
        LOG(INFO) << "[" << f << "] translation error = " << err << std::endl;

        std:;cout << gtTimestamp[f] << " " << espose[0] << " " << espose[1] << " " << espose[2] << std::endl;
    }

    delete camera;
    cv::destroyAllWindows();

    Config::Release();

    return 0;
}
