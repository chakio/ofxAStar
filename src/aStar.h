#pragma once
#include "ofMain.h"
#include "gridField.h"
class aStar
{
    private:
        gridField _costMap;
        vector < vector<double> >_totalCost;
        vector < vector<double> >_hCost;

        //Open,Closeを用意
        vector < vector<int> >_open;
        vector < vector<int> >_beforeOpen;
        vector < vector<int> >_close;
        bool openCleared = false;

        //start,goal
        ofPoint _startPos;
        ofPoint _goalPos;
        ofPoint _beforeGoalPos;

        //親のメモリ確保
        vector < vector<int> >_parent;

        //n,mの確保
        vector<int>_n;
        vector<int>_m;		

        vector< vector<int> > _path;

    public:
        aStar();
        void init(ofPoint startPos, ofPoint goalPos);
        void setCostMap(gridField costMap);
        bool solveOneStep();
        vector< vector<int> > getOpen();
        vector< vector<int> > getClose();
        vector< vector<int> > getPath();
};