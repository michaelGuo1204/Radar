#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "opencv2/features2d.hpp"
#include <vector>
#include "mapcam/CameraCtl.hpp"
struct gaussian
{
    double mean[3], covariance;
    double weight; // Represents the measure to which a particular component defines the pixel value
    gaussian *Next;
    gaussian *Previous;
};


struct Node0
{
    gaussian *pixel_s;
    gaussian *pixel_r;
    int no_of_components;
    Node0 *Next;
};

struct Node1
{
    cv::Mat gauss;
    int no_of_comp;
    Node1 *Next;
};
class Gp{
    public:
        bool first_time = true;
        // Some constants for the algorithm
        const double pi = 3.142;
        const double cthr = 0.00001;
        const double alpha = 0.09;
        const double cT = 0.05;
        const double covariance0 = 6.0;
        const double cf = 0.5;
        const double cfbar = 1.0 - cf;
        const double temp_thr = 9.0 * covariance0 * covariance0;
        const double prune = -alpha * cT;
        const double alpha_bar = 1.0 - alpha;

        double mal_dist;
        double sum = 0.0;
        int count = 0;
        bool close = false;
        int background;
        double mult;
        double duration, duration1, duration2, duration3;
        double temp_cov = 0.0;
        double weight = 0.0;
        double var = 0.0;
        double muR, muG, muB, dR, dG, dB, rVal, gVal, bVal;
        
        int nL, nC;
        uchar *r_ptr;
	    uchar *b_ptr;
        cv::Mat bin_img;
        int i, j, k;
        //Structure used for saving various components for each pixel
    
        gaussian *ptr, *start, *rear, *g_temp, *save, *next, *previous, *nptr, *temp_ptr;

        Node0 *N_ptr, *N_start, *N_rear;

        Node1 *N1_ptr, *N1_start, *N1_rear;
    
    public: 
        //Some function associated with the structure management
        Gp();
        Node0 *Create_Node(double info1, double info2, double info3);
        void Insert_End_Node(Node0 *np);
        gaussian *Create_gaussian(double info1, double info2, double info3);
        void Insert_End_gaussian(gaussian *nptr);
        int Gpmain(cv::Mat frame, cv::Mat &gp);
        gaussian* Delete_gaussian(gaussian*);
        // get frame from Radar
    

};