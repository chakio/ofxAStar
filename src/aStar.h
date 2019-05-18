#pragma once
#include "ofMain.h"

class aStar
{
    private:
        vector < vector<double> >_totalCost;
        vector < vector<double> >_hCost;
        vector < vector<double> >_costMap;

        //Open,Closeを用意
        vector < vector<int> >_open;
        vector < vector<int> >_close;

        //start,goal
        ofPoint _startPos;
        ofPoint _goalPos;

        //親のメモリ確保
        vector < vector<int> >_parent;

        //n,mの確保
        vector<int>_n;
        vector<int>_m;		

        vector< vector<int> > _path;

    public:
        aStar();
        void init(ofPoint startPos, ofPoint goalPos);
        void setCostMap(vector < vector<double> > costMap);
        bool solveOneStep();
        vector< vector<int> > getOpen();
        vector< vector<int> > getClose();
        vector< vector<int> > getPath();
};