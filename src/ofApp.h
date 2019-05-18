#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "aStar.h"

class ofApp : public ofBaseApp{
	public:
		void setup();
		void update();
		void draw();

		void clearObstacle();

		void keyPressed(int key);
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void windowResized(int w, int h);
		
		ofxPanel gui;
		ofxIntSlider switch_mode;
		ofxLabel mode;

		//画面の大きさの分割数
		double resolution = 20;
		double aspectRate[2] = {1,1};
		//ひとつのグリッドの大きさ
		double gridSize;
		
		vector< vector<int> > path;
		vector< vector<double> > obstacle;

		ofVec2f goalPos;
		ofVec2f startPos;

		int state;//0:セット 1:A*とく

		vector < vector<double> >totalCost;
    	vector < vector<double> >hCost;

    	//Open,Closeを用意
    	vector < vector<int> >open;
    	vector < vector<int> >close;

		aStar AStar;
};
