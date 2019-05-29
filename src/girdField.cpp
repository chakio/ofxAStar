#include "gridField.h"
//--------------------------------------------------------------
gridField::gridField() {}
//--------------------------------------------------------------
void gridField::setup(double gridSize, int widthSize, int heightSize) {
    _gridSize = gridSize;
    _fieldWidthSize = widthSize;
    _fieldHeightSize = heightSize;
    for (int widthNum = 0; widthNum < widthSize; widthNum++) {
        vector<double> column;
        column.resize(heightSize);
        _gridValues.push_back(column);
    }
};
//--------------------------------------------------------------
void gridField::draw() {
    // ocupancy
    ofMesh rectangles;
    rectangles.setMode(OF_PRIMITIVE_TRIANGLES);
    for (int rowNum = 0; rowNum < _fieldWidthSize; rowNum++) {
        for (int colNum = 0; colNum < _fieldHeightSize; colNum++) {
            ofPoint verties[4];
            // left upper
            verties[0].x = rowNum * _gridSize;
            verties[0].y = colNum * _gridSize;
            // left lower
            verties[1].x = verties[0].x;
            verties[1].y = verties[0].y + _gridSize;
            // right lower
            verties[2].x = verties[0].x + _gridSize;
            verties[2].y = verties[0].y + _gridSize;
            // right upper
            verties[3].x = verties[0].x + _gridSize;
            verties[3].y = verties[0].y;

            //_gridColors[rowNum][colNum]
            ofColor gridColor;
            if (_gridValues[rowNum][colNum] > 0) {
                gridColor = ofColor(255, 0, 0);
            } else {
                gridColor = ofColor(255, 255, 255);
            }
            // 0
            rectangles.addColor(gridColor);
            rectangles.addVertex(verties[0]);
            // 1
            rectangles.addColor(gridColor);
            rectangles.addVertex(verties[1]);
            // 2
            rectangles.addColor(gridColor);
            rectangles.addVertex(verties[2]);

            // 2
            rectangles.addColor(gridColor);
            rectangles.addVertex(verties[2]);
            // 3
            rectangles.addColor(gridColor);
            rectangles.addVertex(verties[3]);
            // 0
            rectangles.addColor(gridColor);
            rectangles.addVertex(verties[0]);
        }
    }
    rectangles.draw();

    ofMesh lines;
    lines.setMode(OF_PRIMITIVE_LINES);
    // column line
    for (int rowNum = 0; rowNum < _fieldWidthSize + 1; rowNum++) {
        double rowValue = rowNum * _gridSize;
        double rowLength = _fieldHeightSize * _gridSize;

        ofPoint startPos;
        ofPoint endPos;
        startPos.x = rowValue;
        startPos.y = 0;
        endPos.x = startPos.x;
        endPos.y = startPos.y + rowLength;

        lines.addVertex(startPos);
        lines.addVertex(endPos);
    }
    for (int colNum = 0; colNum < _fieldHeightSize + 1; colNum++) {
        double colValue = colNum * _gridSize;
        double colLength = _fieldWidthSize * _gridSize;

        ofPoint startPos;
        ofPoint endPos;
        startPos.x = 0;
        startPos.y = colValue;
        endPos.x = startPos.x + colLength;
        endPos.y = startPos.y;

        lines.addVertex(startPos);
        lines.addVertex(endPos);
    }
    lines.draw();
};
//--------------------------------------------------------------
void gridField::setValue(vector<vector<double>> values) {
    for (int widthNum = 0; widthNum < values.size(); widthNum++) {
        for (int heightNum = 0; heightNum < values[0].size(); heightNum++) {
            _gridValues[widthNum][heightNum] = values[widthNum][heightNum];
        }
    }
}
//--------------------------------------------------------------
double gridField::getValue(int widthIndex, int heightIndex) {
    return _gridValues[widthIndex][heightIndex];
}
//--------------------------------------------------------------
double gridField::getGridSize() { return _gridSize; };
//--------------------------------------------------------------
int gridField::getFieldHeightSize() { return _fieldHeightSize; };
//--------------------------------------------------------------
int gridField::getFieldWidthSize() { return _fieldWidthSize; };