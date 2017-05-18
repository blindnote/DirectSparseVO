#include "Config.h"

bool Config::SetConfigurationFile(const std::string& filename)
{
    if (mConfigPtr == nullptr)
        mConfigPtr = std::shared_ptr<Config>(new Config);

    mConfigPtr->mFile = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    if (!(mConfigPtr->mFile.isOpened()))
    {
        LOG(ERROR) << "Configuration file" << filename << " fail to open!" << std::endl;
        mConfigPtr->mFile.release();
        return false;
    }

    return true;
}

Config::~Config()
{
    if (mFile.isOpened())
        mFile.release();
}

std::shared_ptr<Config> Config::mConfigPtr = nullptr;
