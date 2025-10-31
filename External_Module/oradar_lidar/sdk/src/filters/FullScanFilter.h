// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#ifndef FULLSCANFILTER_H
#define FULLSCANFILTER_H
#include "ordlidar_protocol.h"

#define   Rth    30  
#define   Ath    0.8  
#define     pai      3.1415926
#define     DEC      (pai/180)
#define     Rad      (180/pai)
#define     w_r       2  



typedef struct FilterParas
{
    int filter_type=0;    

	//Smooth filtering
	int maxRange = 150; 
	int minRange = 0;    

	//Bilateral filtering
	int Sigma_D = 5;  
	int Sigma_R = 3;  
	
	//Intensity filtering
	int IntesntiyFilterRange = 70;  
	int Weak_Intensity_Th = 31;  
	
	//Trailing filter
	int Rotation = 10;  
	int level = 0; 
} FilterPara;


class FullScanFilter
{
public:
    enum FilterStrategy
    {
        FS_Smooth,      //Smooth filtering
        FS_Bilateral,   //Bilateral filtering
        FS_Tail,        //Trailing filter
        FS_Intensity,   //Intensity filtering
    };
public:
    FullScanFilter();
    ~FullScanFilter();
    void filter(const full_scan_data_st &in,
                 FilterPara ParaInf,
                 full_scan_data_st &out);

protected:
    void smooth_filter(const full_scan_data_st &in,
                      FilterPara ParaInf,
                      full_scan_data_st &out);

    void bilateral_filter(const full_scan_data_st &in,
                      FilterPara ParaInf,
                      full_scan_data_st &out);

    void tail_filter(const full_scan_data_st &in,
                     FilterPara ParaInf,
                     full_scan_data_st &out);

    void intensity_filter(const full_scan_data_st &in,
                      FilterPara ParaInf,
                      full_scan_data_st &out);


    bool isValidRange(FilterPara ParaInf, uint16_t current_data);

    void swap(uint16_t *a, uint16_t *b);
    void BubbleSort(uint16_t *data, int len);
protected:
    static const int FILTER_WINDOW_SIZE = 3; //5;

};

#endif // FULLSCANFILTER_H
