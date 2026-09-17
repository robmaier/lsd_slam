/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"

#include <sstream>
#include <fstream>
#include <filesystem>
#include <dirent.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <csignal>
#include <cstring>
#include <cstdlib>

#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"

#include "util/Undistorter.h"

#include "opencv2/opencv.hpp"

namespace {
volatile std::sig_atomic_t g_shouldExit = 0;
void handleSigInt(int)
{
	g_shouldExit = 1;
}
}

std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char c){ return !std::isspace(c); }));
        return s;
}
std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char c){ return !std::isspace(c); }).base(), s.end());
        return s;
}
std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
}
int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
    	std::string name = std::string(dirp->d_name);

    	if(name != "." && name != "..")
    		files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
	for(unsigned int i=0;i<files.size();i++)
	{
		if(files[i].at(0) != '/')
			files[i] = dir + files[i];
	}

    return files.size();
}

int getFile (std::string source, std::vector<std::string> &files)
{
	std::ifstream f(source.c_str());

	if(f.good() && f.is_open())
	{
		while(!f.eof())
		{
			std::string l;
			std::getline(f,l);

			l = trim(l);

			if(l == "" || l[0] == '#')
				continue;

			files.push_back(l);
		}

		f.close();

		size_t sp = source.find_last_of('/');
		std::string prefix;
		if(sp == std::string::npos)
			prefix = "";
		else
			prefix = source.substr(0,sp);

		for(unsigned int i=0;i<files.size();i++)
		{
			if(files[i].at(0) != '/')
				files[i] = prefix + "/" + files[i];
		}

		return (int)files.size();
	}
	else
	{
		f.close();
		return -1;
	}

}


namespace {
void printUsage(const char* argv0)
{
	printf("Usage: %s -c CALIB_FILE -f IMAGE_FOLDER_OR_LIST [-r HZ] [-o OUTPUT_DIR] [--gui]\n", argv0);
	printf("  -c, --calib   path to the camera calibration file (required)\n");
	printf("  -f, --files   path to a folder of images, or a text file listing image paths (required)\n");
	printf("  -r, --hz      playback rate in Hz; 0 (default) runs as fast as possible\n");
	printf("  -o, --output  path to output directory (points.ply will be created inside) (optional)\n");
	printf("  --gui         show debug visualization windows (off by default; not supported on all platforms)\n");
}

/// Very small command line parser: supports "-c val", "--calib val" and "--calib=val".
bool parseArgs(int argc, char** argv, std::string &calibFile, std::string &source, double &hz, bool &gui, std::string &outDir)
{
	auto matches = [](const char* arg, const char* shortOpt, const char* longOpt)
	{
		return (shortOpt && shortOpt[0] && strcmp(arg, shortOpt) == 0) || (longOpt && longOpt[0] && strcmp(arg, longOpt) == 0);
	};

	for(int i = 1; i < argc; ++i)
	{
		std::string arg = argv[i];

		auto takeValue = [&]() -> std::string
		{
			size_t eqPos = arg.find('=');
			if(eqPos != std::string::npos)
				return arg.substr(eqPos + 1);
			if(i + 1 < argc)
				return argv[++i];
			return "";
		};

		if(matches(argv[i], "-c", "--calib") || arg.rfind("--calib=", 0) == 0 || arg.rfind("-c=", 0) == 0)
			calibFile = takeValue();
		else if(matches(argv[i], "-f", "--files") || arg.rfind("--files=", 0) == 0 || arg.rfind("-f=", 0) == 0)
			source = takeValue();
		else if(matches(argv[i], "-r", "--hz") || arg.rfind("--hz=", 0) == 0 || arg.rfind("-r=", 0) == 0)
			hz = std::atof(takeValue().c_str());
		else if(matches(argv[i], "-o", "--output") || matches(argv[i], "", "--out") || arg.rfind("--output=", 0) == 0 || arg.rfind("-o=", 0) == 0 || arg.rfind("--out=", 0) == 0)
			outDir = takeValue();
		else if(matches(argv[i], "", "--gui"))
			gui = true;
		else if(matches(argv[i], "-h", "--help"))
			return false;
		else
		{
			printf("unknown argument: %s\n", argv[i]);
			return false;
		}
	}

	return !calibFile.empty() && !source.empty();
}
}

using namespace lsd_slam;
int main( int argc, char** argv )
{
	std::signal(SIGINT, handleSigInt);

	std::string calibFile;
	std::string source;
	std::string outDir;
	double hz = 0;
	bool gui = false;

	if(!parseArgs(argc, argv, calibFile, source, hz, gui, outDir))
	{
		printUsage(argv[0]);
		return 1;
	}


	packagePath = "";

	// get camera calibration in form of an undistorter object.
	// if no undistortion is required, the undistorter will just pass images through.
	Undistorter* undistorter = Undistorter::getUndistorterForFile(calibFile.c_str());

	if(undistorter == 0)
	{
		printf("need valid camera calibration file! (%s)\n", calibFile.c_str());
		exit(0);
	}

	int w = undistorter->getOutputWidth();
	int h = undistorter->getOutputHeight();

	int w_inp = undistorter->getInputWidth();
	int h_inp = undistorter->getInputHeight();

	float fx = undistorter->getK().at<double>(0, 0);
	float fy = undistorter->getK().at<double>(1, 1);
	float cx = undistorter->getK().at<double>(2, 0);
	float cy = undistorter->getK().at<double>(2, 1);
	Sophus::Matrix3f K;
	K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;


	// make output wrapper; the base class is a no-op wrapper for headless (non-ROS) operation.
	Output3DWrapper* outputWrapper = new Output3DWrapper();


	// make slam system
	SlamSystem* system = new SlamSystem(w, h, K, doSlam);
	system->setVisualization(outputWrapper);



	// open image files: first try to open as folder, then as a list file.
	std::vector<std::string> files;
	if(getdir(source, files) >= 0)
	{
		printf("found %d image files in folder %s!\n", (int)files.size(), source.c_str());
	}
	else if(getFile(source, files) >= 0)
	{
		printf("found %d image files in file %s!\n", (int)files.size(), source.c_str());
	}
	else
	{
		printf("could not load file list! wrong path / file?\n");
	}



	cv::Mat image = cv::Mat(h,w,CV_8U);
	int runningIDX=0;
	float fakeTimeStamp = 0;
	std::vector<std::string> acceptedFilenames;

	const auto frameDuration = hz > 0
		? std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / hz))
		: std::chrono::steady_clock::duration::zero();

	for(unsigned int i=0;i<files.size() && !g_shouldExit;i++)
	{
		auto frameStart = std::chrono::steady_clock::now();

		cv::Mat imageDist = cv::imread(files[i], cv::IMREAD_GRAYSCALE);

		if(imageDist.rows != h_inp || imageDist.cols != w_inp)
		{
			if(imageDist.rows * imageDist.cols == 0)
				printf("failed to load image %s! skipping.\n", files[i].c_str());
			else
				printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
						files[i].c_str(),
						w,h,imageDist.cols, imageDist.rows);
			continue;
		}
		assert(imageDist.type() == CV_8U);

		undistorter->undistort(imageDist, image);
		assert(image.type() == CV_8U);

		if(runningIDX == 0)
			system->randomInit(image.data, fakeTimeStamp, runningIDX);
		else
			system->trackFrame(image.data, runningIDX ,hz == 0,fakeTimeStamp);
		acceptedFilenames.push_back(files[i]);
		runningIDX++;
		fakeTimeStamp+=0.03;

		if(hz != 0)
		{
			auto elapsed = std::chrono::steady_clock::now() - frameStart;
			if(elapsed < frameDuration)
				std::this_thread::sleep_for(frameDuration - elapsed);
		}

		if(fullResetRequested)
		{

			printf("FULL RESET!\n");
			delete system;

			system = new SlamSystem(w, h, K, doSlam);
			system->setVisualization(outputWrapper);

			fullResetRequested = false;
			runningIDX = 0;
		}
	}


	system->finalize();



	delete system;
	delete undistorter;
	delete outputWrapper;
	return 0;
}
