//
// Created by rochelle on 17-5-16.
//

#ifndef DIRECTSPARSEVO_CONFIG_H
#define DIRECTSPARSEVO_CONFIG_H

#include "Common.h"

class Config
{
public:
    ~Config();

    static bool SetConfigurationFile(const std::string& filename);

    template< typename T >
    static T Get(const std::string& key)
    {
        return T(Config::mConfigPtr->mFile[key]);
    }

    static void Release()
    {
        if (mConfigPtr)
        {
            mConfigPtr.reset();
        }
        mConfigPtr = nullptr;
    }

private:
    static std::shared_ptr<Config> mConfigPtr;
    cv::FileStorage mFile;

    Config() {}
};

#endif //DIRECTSPARSEVO_CONFIG_H
