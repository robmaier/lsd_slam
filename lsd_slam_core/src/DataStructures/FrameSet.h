#pragma once
#include <set>
#include <Eigen/Core>

namespace lsd_slam
{

class Frame;

/** Orders frames by id instead of by address, so iteration order does not depend on heap layout. */
struct FrameIdLess
{
	// defined in Frame.h
	bool operator()(const Frame* a, const Frame* b) const;
};

typedef std::set<Frame*, FrameIdLess, Eigen::aligned_allocator<Frame*> > FrameSet;

}
