/*
    FILE: DBSCAN.h
    ------------------------
    function header of DBSCAN
*/
#ifndef OBSTALCE_CLUSTERING_DBSCAN_H
#define OBSTALCE_CLUSTERING_DBSCAN_H
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <map>

using namespace std;

const int DBSCAN_NOISE = -2;
const int NOT_CLASSIFIED = -1;

class Point {
public:
    double x, y, z;
    int ptsCnt, cluster;
    Point (double _x, double _y, double _z){
        x = _x;
        y = _y;
        z = _z;
        ptsCnt = 0;
        cluster = -1;
    }
    double getDis(const Point & ot) {
        return sqrt((x-ot.x)*(x-ot.x)+(y-ot.y)*(y-ot.y)+(z-ot.z)*(z-ot.z));
    }
};

class DBSCAN {
public:
    int n, minPts;
    double eps;
    vector<Point> points;
    int size;
    vector<vector<int>> adjPoints;
    vector<bool> visited;
    vector<vector<int>> cluster;
    int clusterIdx;
    
    DBSCAN(double eps, int minPts, vector<Point> points) {
        // this->n = n;
        this->eps = eps;
        this->minPts = minPts;
        this->points = points;
        this->size = (int)points.size();
        adjPoints.resize(this->size);
        this->clusterIdx=-1;
    }
    void run () {
        checkNearPoints();
        
        for(int i=0;i<this->size;i++) {
            if(points[i].cluster != NOT_CLASSIFIED) continue;
            
            if(isCoreObject(i)) {
                dfs(i, ++clusterIdx);
            } else {
                points[i].cluster = DBSCAN_NOISE;
            }
        }
        
        cluster.resize(clusterIdx+1);
        for(int i=0;i<this->size;i++) {
            if(points[i].cluster != DBSCAN_NOISE) {
                cluster[points[i].cluster].push_back(i);
            }
        }
    }
    
    void dfs (int now, int c) {
        points[now].cluster = c;
        if(!isCoreObject(now)) return;
        
        for(auto&next:adjPoints[now]) {
            if(points[next].cluster != NOT_CLASSIFIED) continue;
            dfs(next, c);
        }
    }
    
    void checkNearPoints() {
        for(int i=0;i<this->size;i++) {
            for(int j=0;j<this->size;j++) {
                if(i==j) continue;
                if(points[i].getDis(points[j]) <= this->eps) {
                    points[i].ptsCnt++;
                    adjPoints[i].push_back(j);
                }
            }
        }
    }
    // is idx'th point core object?
    bool isCoreObject(int idx) {
        return points[idx].ptsCnt >= minPts;
    }
    
    vector<vector<int>> getCluster() {
        return cluster;
    }
};

#endif