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

#include "GlobalMapping/KeyFrameGraph.h"
#include "DataStructures/Frame.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/estimate_propagator.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>

#include "opencv2/opencv.hpp"

#include <g2o/types/sim3/sim3.h>
#include "GlobalMapping/g2oTypeSim3Sophus.h"


#include "IOWrapper/ImageDisplay.h"

// for mkdir
#include <sys/types.h>
#include <sys/stat.h>
// for iterating over files in a directory
#include <dirent.h>
#include <queue>

#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>

#include "util/globalFuncs.h"

namespace lsd_slam
{

namespace
{
constexpr float MAX_POINT_CLOUD_RELATIVE_IDEPTH_STDDEV = 0.25f;

bool hasGoodPointCloudConfidence(float inverseDepth, float inverseDepthVariance)
{
	return std::isfinite(inverseDepth) && inverseDepth > 0.0f &&
		std::isfinite(inverseDepthVariance) && inverseDepthVariance > 0.0f &&
		std::sqrt(inverseDepthVariance) < MAX_POINT_CLOUD_RELATIVE_IDEPTH_STDDEV * inverseDepth;
}
}

KFConstraintStruct::~KFConstraintStruct()
{
	if(edge != 0)
		delete edge;
}

KeyFrameGraph::KeyFrameGraph()
: nextEdgeId(0)
{
	typedef g2o::BlockSolver_7_3 BlockSolver;
	typedef g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType> LinearSolver;
	//typedef g2o::LinearSolverPCG<BlockSolver::PoseMatrixType> LinearSolver;
	// g2o (post-2017) requires the solver chain to be handed over as std::unique_ptr.
	auto linearSolver = std::make_unique<LinearSolver>();
	LinearSolver* solver = linearSolver.get();
	auto blockSolverPtr = std::make_unique<BlockSolver>(std::move(linearSolver));
	BlockSolver* blockSolver = blockSolverPtr.get();
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolverPtr));
	graph.setAlgorithm(algorithm);
	
    graph.setVerbose(false); // printOptimizationInfo
	solver->setWriteDebug(true);
	blockSolver->setWriteDebug(true);
	algorithm->setWriteDebug(true);


	totalPoints=0;
	totalEdges=0;
	totalVertices=0;


}

KeyFrameGraph::~KeyFrameGraph()
{
	// deletes edges
	for (KFConstraintStruct* edge : newEdgeBuffer)
		delete edge;	// deletes the g2oedge, which deletes the kernel.

	// deletes keyframes (by deleting respective shared pointers).

	idToKeyFrame.clear();

	// deletes pose structs (their memory is managed by graph)
	// WARNING: at this point, all Frames have to be deleted, otherwise it night cause segfaults!
	for(FramePoseStruct* p : allFramePoses)
		delete p;
}


void KeyFrameGraph::addFrame(Frame* frame)
{

	frame->pose->isRegisteredToGraph = true;
	FramePoseStruct* pose = frame->pose;


	allFramePosesMutex.lock();
	allFramePoses.push_back(pose);
	allFramePosesMutex.unlock();
}

void KeyFrameGraph::savePointCloud(const std::string& filename)
{
	keyframesAllMutex.lock_shared();

#pragma pack(push, 1)
	struct PlyVertex
	{
		float x, y, z;
		unsigned char r, g, b;
		float intensity;
	};
#pragma pack(pop)

	size_t totalValidPoints = 0;
	for(size_t k = 0; k < keyframesAll.size(); ++k)
	{
		Frame* kf = keyframesAll[k];
		if(!kf || !kf->hasIDepthBeenSet()) continue;

		boost::shared_lock<boost::shared_mutex> lock = kf->getActiveLock();
		const float* idepth = kf->idepth(0);
		const float* idepthVar = kf->idepthVar(0);
		int w = kf->width(0);
		int h = kf->height(0);
		float fxInv = kf->fxInv(0);
		float fyInv = kf->fyInv(0);
		float cxInv = kf->cxInv(0);
		float cyInv = kf->cyInv(0);
		Sim3 camToWorld = kf->getScaledCamToWorld();

		if(!idepth) continue;

		for(int y = 0; y < h; ++y)
		{
			for(int x = 0; x < w; ++x)
			{
				int idx = x + y * w;
				float id = idepth[idx];
				if(!std::isfinite(id) || id <= 0.0f) continue;
				if(idepthVar == nullptr || !hasGoodPointCloudConfidence(id, idepthVar[idx])) continue;

				float depth = 1.0f / id;
				if(!std::isfinite(depth)) continue;
				Sophus::Vector3f pt_cam((x * fxInv + cxInv) * depth, (y * fyInv + cyInv) * depth, depth);
				Sophus::Vector3f pt_world = (camToWorld * pt_cam.cast<sophusType>()).cast<float>();
				if(!std::isfinite(pt_world[0]) || !std::isfinite(pt_world[1]) || !std::isfinite(pt_world[2])) continue;
				totalValidPoints++;
			}
		}
	}

	printf("Saving %zu points from %zu keyframes to %s...\n", totalValidPoints, keyframesAll.size(), filename.c_str());

	std::ofstream f(filename.c_str(), std::ios::out | std::ios::binary);
	if(!f.is_open())
	{
		printf("Error: Could not open file %s for writing PLY!\n", filename.c_str());
		keyframesAllMutex.unlock_shared();
		return;
	}

	f << "ply\n";
	f << "format binary_little_endian 1.0\n";
	f << "element vertex " << totalValidPoints << "\n";
	f << "property float x\n";
	f << "property float y\n";
	f << "property float z\n";
	f << "property uchar red\n";
	f << "property uchar green\n";
	f << "property uchar blue\n";
	f << "property float intensity\n";
	f << "end_header\n";

	const size_t CHUNK_SIZE = 4096;
	std::vector<PlyVertex> vertexBuffer;
	vertexBuffer.reserve(CHUNK_SIZE);

	for(size_t k = 0; k < keyframesAll.size(); ++k)
	{
		Frame* kf = keyframesAll[k];
		if(!kf || !kf->hasIDepthBeenSet()) continue;

		boost::shared_lock<boost::shared_mutex> lock = kf->getActiveLock();
		const float* idepth = kf->idepth(0);
		const float* idepthVar = kf->idepthVar(0);
		const float* image = kf->image(0);
		int w = kf->width(0);
		int h = kf->height(0);
		float fxInv = kf->fxInv(0);
		float fyInv = kf->fyInv(0);
		float cxInv = kf->cxInv(0);
		float cyInv = kf->cyInv(0);
		Sim3 camToWorld = kf->getScaledCamToWorld();

		if(!idepth) continue;

		for(int y = 0; y < h; ++y)
		{
			for(int x = 0; x < w; ++x)
			{
				int idx = x + y * w;
				float id = idepth[idx];
				if(!std::isfinite(id) || id <= 0.0f) continue;
				if(idepthVar == nullptr || !hasGoodPointCloudConfidence(id, idepthVar[idx])) continue;

				float depth = 1.0f / id;
				if(!std::isfinite(depth)) continue;
				Sophus::Vector3f pt_cam((x * fxInv + cxInv) * depth, (y * fyInv + cyInv) * depth, depth);
				Sophus::Vector3f pt_world = (camToWorld * pt_cam.cast<sophusType>()).cast<float>();
				if(!std::isfinite(pt_world[0]) || !std::isfinite(pt_world[1]) || !std::isfinite(pt_world[2])) continue;

				float val = (image != nullptr) ? image[idx] : 0.0f;
				if(!std::isfinite(val)) val = 0.0f;
				if(val < 0.0f) val = 0.0f;
				if(val > 255.0f) val = 255.0f;
				unsigned char c = static_cast<unsigned char>(val + 0.5f);

				PlyVertex v;
				v.x = pt_world[0];
				v.y = pt_world[1];
				v.z = pt_world[2];
				v.r = c;
				v.g = c;
				v.b = c;
				v.intensity = val / 255.0f;

				vertexBuffer.push_back(v);
				if(vertexBuffer.size() >= CHUNK_SIZE)
				{
					f.write(reinterpret_cast<const char*>(vertexBuffer.data()), vertexBuffer.size() * sizeof(PlyVertex));
					vertexBuffer.clear();
				}
			}
		}
	}

	if(!vertexBuffer.empty())
	{
		f.write(reinterpret_cast<const char*>(vertexBuffer.data()), vertexBuffer.size() * sizeof(PlyVertex));
		vertexBuffer.clear();
	}

	f.flush();
	f.close();

	keyframesAllMutex.unlock_shared();
	printf("Done saving PLY file with %zu points to %s!\n", totalValidPoints, filename.c_str());
}

void KeyFrameGraph::saveKeyframePointClouds(const std::string& outputDirectory, const std::vector<std::string>& inputFilenames)
{
#pragma pack(push, 1)
	struct PlyVertex
	{
		float x, y, z;
		unsigned char r, g, b;
		float intensity;
	};
#pragma pack(pop)

	keyframesAllMutex.lock_shared();
	for(size_t k = 0; k < keyframesAll.size(); ++k)
	{
		Frame* keyframe = keyframesAll[k];
		if(!keyframe || !keyframe->hasIDepthBeenSet()) continue;

		const int frameId = keyframe->id();
		if(frameId < 0 || static_cast<size_t>(frameId) >= inputFilenames.size()) continue;

		boost::shared_lock<boost::shared_mutex> lock = keyframe->getActiveLock();
		const float* idepth = keyframe->idepth(0);
		const float* idepthVar = keyframe->idepthVar(0);
		const float* image = keyframe->image(0);
		if(!idepth) continue;

		const int width = keyframe->width(0);
		const int height = keyframe->height(0);
		const float fxInv = keyframe->fxInv(0);
		const float fyInv = keyframe->fyInv(0);
		const float cxInv = keyframe->cxInv(0);
		const float cyInv = keyframe->cyInv(0);
		const Sim3 camToWorld = keyframe->getScaledCamToWorld();
		std::vector<PlyVertex> vertices;

		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; ++x)
			{
				const int pixel = x + y * width;
				const float inverseDepth = idepth[pixel];
				if(!std::isfinite(inverseDepth) || inverseDepth <= 0.0f) continue;
				if(idepthVar == nullptr || !hasGoodPointCloudConfidence(inverseDepth, idepthVar[pixel])) continue;

				const float depth = 1.0f / inverseDepth;
				if(!std::isfinite(depth)) continue;
				const Sophus::Vector3f pointCamera((x * fxInv + cxInv) * depth, (y * fyInv + cyInv) * depth, depth);
				const Sophus::Vector3f pointWorld = (camToWorld * pointCamera.cast<sophusType>()).cast<float>();
				if(!std::isfinite(pointWorld[0]) || !std::isfinite(pointWorld[1]) || !std::isfinite(pointWorld[2])) continue;

				float intensity = image != nullptr ? image[pixel] : 0.0f;
				if(!std::isfinite(intensity)) intensity = 0.0f;
				intensity = std::max(0.0f, std::min(255.0f, intensity));
				const unsigned char color = static_cast<unsigned char>(intensity + 0.5f);
				vertices.push_back({pointWorld[0], pointWorld[1], pointWorld[2], color, color, color, intensity / 255.0f});
			}
		}

		const std::string& inputFilename = inputFilenames[frameId];
		const size_t filenameStart = inputFilename.find_last_of("/\\") + 1;
		const std::string basename = inputFilename.substr(filenameStart);
		const size_t extensionStart = basename.find_last_of('.');
		const std::string outputPath = outputDirectory + "/" + basename.substr(0, extensionStart) + ".ply";
		std::ofstream output(outputPath.c_str(), std::ios::out | std::ios::binary);
		if(!output.is_open())
		{
			printf("Could not open point cloud file %s.\n", outputPath.c_str());
			continue;
		}

		output << "ply\nformat binary_little_endian 1.0\n";
		output << "element vertex " << vertices.size() << "\n";
		output << "property float x\nproperty float y\nproperty float z\n";
		output << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
		output << "property float intensity\nend_header\n";
		if(!vertices.empty())
			output.write(reinterpret_cast<const char*>(vertices.data()), vertices.size() * sizeof(PlyVertex));
	}
	keyframesAllMutex.unlock_shared();
}

void KeyFrameGraph::saveDepthVisualizations(const std::string& outputDirectory, const std::vector<std::string>& inputFilenames)
{
	keyframesAllMutex.lock_shared();

	for(size_t k = 0; k < keyframesAll.size(); ++k)
	{
		Frame* keyframe = keyframesAll[k];
		if(!keyframe || !keyframe->hasIDepthBeenSet()) continue;

		const int frameId = keyframe->id();
		if(frameId < 0 || static_cast<size_t>(frameId) >= inputFilenames.size())
		{
			printf("No input filename for keyframe %d; skipping depth visualization.\n", frameId);
			continue;
		}

		boost::shared_lock<boost::shared_mutex> lock = keyframe->getActiveLock();
		const float* idepth = keyframe->idepth(0);
		const float* idepthVar = keyframe->idepthVar(0);
		const float* image = keyframe->image(0);
		const int width = keyframe->width(0);
		const int height = keyframe->height(0);
		if(!idepth || !image) continue;

		cv::Mat baseImage(height, width, CV_8UC1);
		cv::Mat normalizedIdepth(height, width, CV_8UC1, cv::Scalar(0));
		cv::Mat validMask(height, width, CV_8UC1, cv::Scalar(0));
		float minIdepth = std::numeric_limits<float>::max();
		float maxIdepth = std::numeric_limits<float>::lowest();
		for(int pixel = 0; pixel < width * height; ++pixel)
		{
			baseImage.data[pixel] = cv::saturate_cast<unsigned char>(image[pixel]);
			if(idepth[pixel] > 0.0f && (idepthVar == nullptr || idepthVar[pixel] > 0.0f))
			{
				minIdepth = std::min(minIdepth, idepth[pixel]);
				maxIdepth = std::max(maxIdepth, idepth[pixel]);
				validMask.data[pixel] = 255;
			}
		}
		if(minIdepth > maxIdepth) continue;

		const float inverseRange = maxIdepth - minIdepth;
		for(int pixel = 0; pixel < width * height; ++pixel)
		{
			if(validMask.data[pixel] == 0) continue;
			normalizedIdepth.data[pixel] = inverseRange > 0.0f
				? cv::saturate_cast<unsigned char>(255.0f * (idepth[pixel] - minIdepth) / inverseRange)
				: 127;
		}

		cv::Mat visualization;
		cv::cvtColor(baseImage, visualization, cv::COLOR_GRAY2BGR);
		cv::Mat depthColors;
		cv::applyColorMap(normalizedIdepth, depthColors, cv::COLORMAP_TURBO);
		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; ++x)
			{
				if(validMask.at<unsigned char>(y, x) == 0) continue;
				cv::Vec3b& outputColor = visualization.at<cv::Vec3b>(y, x);
				const cv::Vec3b& depthColor = depthColors.at<cv::Vec3b>(y, x);
				outputColor = 0.4 * outputColor + 0.6 * depthColor;
			}
		}

		const std::string& inputFilename = inputFilenames[frameId];
		const size_t filenameStart = inputFilename.find_last_of("/\\") + 1;
		const std::string outputPath = outputDirectory + "/" + inputFilename.substr(filenameStart);
		if(!cv::imwrite(outputPath, visualization))
			printf("Could not save depth visualization %s.\n", outputPath.c_str());
	}

	keyframesAllMutex.unlock_shared();
}

void KeyFrameGraph::dumpMap(std::string folder)
{
	printf("DUMP MAP: dumping to %s\n", folder.c_str());

	keyframesAllMutex.lock_shared();
	char buf[100];
	int succ = system(("rm -rf "+folder).c_str());
	succ += system(("mkdir "+folder).c_str());

	for(unsigned int i=0;i<keyframesAll.size();i++)
	{
		snprintf(buf, 100, "%s/depth-%d.png", folder.c_str(), i);
		cv::imwrite(buf, getDepthRainbowPlot(keyframesAll[i], 0));

		snprintf(buf, 100, "%s/frame-%d.png", folder.c_str(), i);
		cv::imwrite(buf, cv::Mat(keyframesAll[i]->height(), keyframesAll[i]->width(),CV_32F,keyframesAll[i]->image()));

		snprintf(buf, 100, "%s/var-%d.png", folder.c_str(), i);
		cv::imwrite(buf, getVarRedGreenPlot(keyframesAll[i]->idepthVar(),keyframesAll[i]->image(),keyframesAll[i]->width(),keyframesAll[i]->height()));
	}


	int i = keyframesAll.size()-1;
	Util::displayImage("VAR PREVIEW", getVarRedGreenPlot(keyframesAll[i]->idepthVar(),keyframesAll[i]->image(),keyframesAll[i]->width(),keyframesAll[i]->height()));

	printf("DUMP MAP (succ %d): dumped %d depthmaps\n", succ,  (int)keyframesAll.size());

	Eigen::MatrixXf res, resD, resP, huber, usage, consistency, distance, error;
	Eigen::VectorXf meanRootInformation, usedPixels;

	res.resize(keyframesAll.size(), keyframesAll.size());
	resD.resize(keyframesAll.size(), keyframesAll.size());
	resP.resize(keyframesAll.size(), keyframesAll.size());
	usage.resize(keyframesAll.size(), keyframesAll.size());
	consistency.resize(keyframesAll.size(), keyframesAll.size());
	distance.resize(keyframesAll.size(), keyframesAll.size());
	error.resize(keyframesAll.size(), keyframesAll.size());
	meanRootInformation.resize(keyframesAll.size());
	usedPixels.resize(keyframesAll.size());
	res.setZero();
	resD.setZero();
	resP.setZero();
	usage.setZero();
	consistency.setZero();
	distance.setZero();
	error.setZero();
	meanRootInformation.setZero();
	usedPixels.setZero();

	for(unsigned int i=0;i<keyframesAll.size();i++)
	{
		meanRootInformation[i] = keyframesAll[i]->meanInformation;
		usedPixels[i] = keyframesAll[i]->numPoints / (float)keyframesAll[i]->numMappablePixels;
	}


	edgesListsMutex.lock_shared();
	for(unsigned int i=0;i<edgesAll.size();i++)
	{
		KFConstraintStruct* e = edgesAll[i];

		res(e->firstFrame->idxInKeyframes, e->secondFrame->idxInKeyframes) = e->meanResidual;
		resD(e->firstFrame->idxInKeyframes, e->secondFrame->idxInKeyframes) = e->meanResidualD;
		resP(e->firstFrame->idxInKeyframes, e->secondFrame->idxInKeyframes) = e->meanResidualP;
		usage(e->firstFrame->idxInKeyframes, e->secondFrame->idxInKeyframes) = e->usage;
		consistency(e->firstFrame->idxInKeyframes, e->secondFrame->idxInKeyframes) = e->reciprocalConsistency;
		distance(e->firstFrame->idxInKeyframes, e->secondFrame->idxInKeyframes) = e->secondToFirst.translation().norm();
		error(e->firstFrame->idxInKeyframes, e->secondFrame->idxInKeyframes) = e->edge->chi2();
	}
	edgesListsMutex.unlock_shared();
	keyframesAllMutex.unlock_shared();


	std::ofstream fle;

	fle.open(folder+"/residual.txt");
	fle << res;
	fle.close();

	fle.open(folder+"/residualD.txt");
	fle << resD;
	fle.close();

	fle.open(folder+"/residualP.txt");
	fle << resP;
	fle.close();

	fle.open(folder+"/usage.txt");
	fle << usage;
	fle.close();

	fle.open(folder+"/consistency.txt");
	fle << consistency;
	fle.close();

	fle.open(folder+"/distance.txt");
	fle << distance;
	fle.close();

	fle.open(folder+"/error.txt");
	fle << error;
	fle.close();

	fle.open(folder+"/meanRootInformation.txt");
	fle << meanRootInformation;
	fle.close();

	fle.open(folder+"/usedPixels.txt");
	fle << usedPixels;
	fle.close();

	printf("DUMP MAP: dumped %d edges\n", (int)edgesAll.size());
}



void KeyFrameGraph::addKeyFrame(Frame* frame)
{
	if(frame->pose->graphVertex != nullptr)
		return;

	// Insert vertex into g2o graph
	VertexSim3* vertex = new VertexSim3();
	vertex->setId(frame->id());

	Sophus::Sim3d camToWorld_estimate = frame->getScaledCamToWorld();

	if(!frame->hasTrackingParent())
		vertex->setFixed(true);

	vertex->setEstimate(camToWorld_estimate);
	vertex->setMarginalized(false);

	frame->pose->graphVertex = vertex;

	newKeyframesBuffer.push_back(frame);

}

void KeyFrameGraph::insertConstraint(KFConstraintStruct* constraint)
{
	EdgeSim3* edge = new EdgeSim3();
	edge->setId(nextEdgeId);
	++ nextEdgeId;

	totalEdges++;

	edge->setMeasurement(constraint->secondToFirst);
	edge->setInformation(constraint->information);
	edge->setRobustKernel(constraint->robustKernel);

	edge->resize(2);
	assert(constraint->firstFrame->pose->graphVertex != nullptr);
	edge->setVertex(0, constraint->firstFrame->pose->graphVertex);
	assert(constraint->secondFrame->pose->graphVertex != nullptr);
	edge->setVertex(1, constraint->secondFrame->pose->graphVertex);

	constraint->edge = edge;
	newEdgeBuffer.push_back(constraint);


	constraint->firstFrame->neighbors.insert(constraint->secondFrame);
	constraint->secondFrame->neighbors.insert(constraint->firstFrame);

	for(int i=0;i<totalVertices;i++)
	{
		//shortestDistancesMap
	}



	edgesListsMutex.lock();
	constraint->idxInAllEdges = edgesAll.size();
	edgesAll.push_back(constraint);
	edgesListsMutex.unlock();
}


bool KeyFrameGraph::addElementsFromBuffer()
{
	bool added = false;

	keyframesForRetrackMutex.lock();
	for (auto newKF : newKeyframesBuffer)
	{
		graph.addVertex(newKF->pose->graphVertex);
		assert(!newKF->pose->isInGraph);
		newKF->pose->isInGraph = true;

		keyframesForRetrack.push_back(newKF);

		added = true;
	}
	keyframesForRetrackMutex.unlock();

	newKeyframesBuffer.clear();
	for (auto edge : newEdgeBuffer)
	{
		graph.addEdge(edge->edge);
		added = true;
	}
	newEdgeBuffer.clear();

	return added;
}

int KeyFrameGraph::optimize(int num_iterations)
{
	// Abort if graph is empty, g2o shows an error otherwise
	if (graph.edges().size() == 0)
		return 0;
	
	graph.setVerbose(false); // printOptimizationInfo
	graph.initializeOptimization();
	

	return graph.optimize(num_iterations, false);

}



void KeyFrameGraph::calculateGraphDistancesToFrame(Frame* startFrame, std::unordered_map< Frame*, int >* distanceMap)
{
	distanceMap->insert(std::make_pair(startFrame, 0));
	
	std::multimap< int, Frame* > priorityQueue;
	priorityQueue.insert(std::make_pair(0, startFrame));
	while (! priorityQueue.empty())
	{
		auto it = priorityQueue.begin();
		int length = it->first;
		Frame* frame = it->second;
		priorityQueue.erase(it);
		
		auto mapEntry = distanceMap->find(frame);
		
		if (mapEntry != distanceMap->end() && length > mapEntry->second)
		{
			continue;
		}
		
		for (Frame* neighbor : frame->neighbors)
		{
			auto neighborMapEntry = distanceMap->find(neighbor);
			
			if (neighborMapEntry != distanceMap->end() && length + 1 >= neighborMapEntry->second)
				continue;
			
			if (neighborMapEntry != distanceMap->end())
				neighborMapEntry->second = length + 1;
			else
				distanceMap->insert(std::make_pair(neighbor, length + 1));
			priorityQueue.insert(std::make_pair(length + 1, neighbor));
		}
	}
}

}
