#include <iostream>
#include <vector>
#include <fstream>
#include <string>


using namespace std;

class featureDetection{

public:
	featureDetection();
	~featureDetection();
	void filteringPipeLine(Mat);
	vector<Vec2f> houghTransform();
	Mat lineItr(Mat,vector<Vec2f>, string);
	int _width, _height;

protected:
	bool findIntersection(vector<Point>, Point&);
	vector<Point2f> ransac(vector<Point2f>);
	void visualize(Mat);
	vector<Vec2f> _lines;
	ofstream myfile;
	Mat _detectedEdges;
	int _LMWidth;
	int _thres;
	float _rho, _theta,_ransacThres, _houghThres;
	
};
