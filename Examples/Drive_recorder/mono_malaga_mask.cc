/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <regex>
#include <unistd.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;
namespace fs = boost::filesystem;

void LoadImages(const string &path, map<double, string> &timeStampedImageNames);

int main(int argc, char **argv) {
    if (argc != 5) {
        cerr << endl << "Usage: ./mono_malaga_mask path_to_vocabulary path_to_settings path_to_sequence path_to_mask" <<
        endl;
        return 1;
    }

    // Retrieve paths to images
    const fs::path sequencePath(argv[3]);
    const fs::path maskPath(argv[4]);
    map<double, string> timeStampedImageNames;
    LoadImages(sequencePath.string(), timeStampedImageNames);

    int nImages = timeStampedImageNames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat image, mask;
    for (auto timeStampedImageName : timeStampedImageNames) {
        // Read image from file
        image = cv::imread(sequencePath.string() + "/" + timeStampedImageName.second + ".jpg", CV_LOAD_IMAGE_UNCHANGED);
        if (image.empty()) {
            cerr << endl << "Failed to load image: " << sequencePath.string() + timeStampedImageName.second + ".jpg" << endl;
            return 1;
        }

        mask = cv::imread(maskPath.string()+ "/" + timeStampedImageName.second + ".png", CV_LOAD_IMAGE_GRAYSCALE);
        if (mask.empty()) {
            cerr << endl << "Failed to load image: " << maskPath.string() + timeStampedImageName.second + ".png" << endl;
            mask = cv::Mat(image.size(), CV_8UC3, cv::Scalar::all(0));
        }

        double timeStamp = timeStampedImageName.first;


        // Pass the image to the SLAM system
        SLAM.TrackMonocular(image, timeStamp, cv::Mat());
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}

void LoadImages(const string &path, map<double, string> &timeStampedImageNames) {
    //timestampをidとして扱う
    fs::directory_iterator end;
    for (fs::directory_iterator it(path); it != end; ++it) {
        regex re("img_CAMERA1_([0-9.]*)_left");
        smatch match;
        if (regex_match(it->path().stem().string(), match, re)) {
            const double timeStamp = stod(match[1]);
            cout << fixed << timeStamp << endl;
            timeStampedImageNames[timeStamp] = it->path().stem().string();
        }
    }
}
