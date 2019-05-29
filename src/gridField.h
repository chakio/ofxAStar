#pragma once
#include "ofMain.h"
class gridField
{
    private:
        double _gridSize;// pix
        int _fieldHeightSize;//height grid number
        int _fieldWidthSize;//width grid number
        vector < vector <double> > _gridValues;

    public:
        gridField();
        void setup(double gridSize, int widthSize, int heightSize);
        void setValue(vector<vector<double>>values);
        void draw();
        
        double getGridSize();
        int getFieldWidthSize();
        int getFieldHeightSize();
        double getValue(int widthIndex, int heightIndex);
};