#ifndef SVO_SOLOVE_5POINTS
#define SVO_SOLOVE_5POINTS

#include <svo/global.h>
#include <Eigen/Dense>
using namespace cv;


namespace svo
{
	void decomposeEssentialMat( InputArray _E, OutputArray _R1, OutputArray _R2, OutputArray _t );
	static int recoverPose( InputArray E, InputArray _points1, InputArray _points2, InputArray _cameraMatrix,
                     OutputArray _R, OutputArray _t, InputOutputArray _mask);
	static int recoverPose( InputArray E, InputArray _points1, InputArray _points2, OutputArray _R,
                     OutputArray _t, double focal, Point2d pp, InputOutputArray _mask);
	
}

#endif