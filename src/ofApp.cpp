#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
    ofSetWindowShape((double)gridSize * (double)resolution / aspectRate[1] * aspectRate[0],
                     (double)gridSize * (double)resolution);
    for(int i=0;i<resolution/aspectRate[1]*aspectRate[0];i++)
    {
        vector<double> obs;
        for(int j=0;j<resolution;j++)
        {
            obs.push_back(0);
        }
        obstacle.push_back(obs);
    }
    costMap.setup(gridSize, resolution / aspectRate[1] * aspectRate[0], resolution);
    costMap.setValue(obstacle);
    startPos.x  = 0;
    startPos.y  = (int)(resolution/2)-1;
    goalPos.x   = (int)(resolution/aspectRate[1]*aspectRate[0])-1;
    goalPos.y   = (int)(resolution/2)-1;
    ofSetFrameRate(120);
	ofBackground(255);

    gui.setup("panel");
	gui.setPosition(0, 0);
    gui.add(switch_mode.setup("mode",0,2,0,200,20));
	gui.add(mode.setup("mode","obstacle",200,20));
}

//--------------------------------------------------------------
void ofApp::update()
{
    //セットアップ
    if(state==0)
    {
        if(switch_mode==0)
        {
            mode="obstacle";
        }
        else if(switch_mode==1)
        {
            mode="start";
        }
        else if(switch_mode==2)
        {
            mode="goal";
        }
    }
    //solve
    else if(state==1)
    {
        if(AStar.solveOneStep())
        {
            path = AStar.getPath();
            state=0;
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    costMap.setValue(obstacle);
    //guiの描画
    gui.draw();

    //gridの描画　
    open = AStar.getOpen();
    close = AStar.getClose();
    //open closeの描画
    for(int i=0;i<open.size();i++)
    {
        ofSetColor(255,0,255);
        ofFill();
        ofRect(ofPoint(open[i][0]*gridSize,open[i][1]*gridSize),gridSize,gridSize);
    }
    for(int i=0;i<close.size();i++)
    {
        ofSetColor(255,255,0);
        ofFill();
        ofRect(ofPoint(close[i][0]*gridSize,close[i][1]*gridSize),gridSize,gridSize);
    }

    //取得したpathの描画
    for(int i=0;i<path.size();i++)
    {
        ofSetColor(0,0,255);
        ofFill();
        ofRect(ofPoint(path[i][0]*gridSize,path[i][1]*gridSize),gridSize,gridSize);
    }

    //障害物は塗りつぶし
    ofSetColor(0);
    for(int i=0;i<resolution;i++)
    {
        for(int j=0;j<resolution/aspectRate[1]*aspectRate[0];j++)
        {
            if(obstacle[j][i]==0)
            {
                ofNoFill();
            }
            else
            {
	            ofFill();
            }
            ofRect(ofPoint(j*gridSize,i*gridSize),gridSize,gridSize);
        }
    }
    
    //start,goalの描画
    ofFill();
    //start
    ofSetColor(0,0,255);
    ofRect(ofPoint(startPos.x*gridSize,startPos.y*gridSize),gridSize,gridSize);
    //goal
    ofSetColor(255,0,0);
    ofRect(ofPoint(goalPos.x*gridSize,goalPos.y*gridSize),gridSize,gridSize);
}

void ofApp::clearObstacle()
{
    path.clear();
    for(int i=0;i<resolution/aspectRate[1]*aspectRate[0];i++)
    {
        for(int j=0;j<resolution;j++)
        {
            obstacle[i][j]=0;
        }   
    }
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key=='s')
    {
        path.clear();
        AStar.setCostMap(costMap);
        AStar.init(startPos,goalPos);
        state=1;
    }
    else if(key=='c')
    {
        clearObstacle();
    }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    //障害物配置モードの時
    //カーソルの位置に基づき障害物の配置
    if(switch_mode==0)
    {
        int xPos=(x/gridSize);
        int yPos=(y/gridSize);
        obstacle[xPos][yPos]=255;
    }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    //障害物配置モードの時
    //カーソルの位置に基づき障害物の配置
    if(switch_mode==0)
    {
        int xPos=(x/gridSize);
        int yPos=(y/gridSize);
        obstacle[xPos][yPos]=255;
    }
    //スタート配置モードの時
    //カーソルの位置に基づきスタートの配置
    else if(switch_mode==1)
    {
        int xPos=(x/gridSize);
        int yPos=(y/gridSize);
        startPos.x=xPos;
        startPos.y=yPos;
        cout<<startPos.x << " "<<startPos.y<<endl;
    }
    //ゴール配置モードの時
    //カーソルの位置に基づきゴールの配置
    else if(switch_mode==2)
    {
        int xPos=(x/gridSize);
        int yPos=(y/gridSize);
        goalPos.x=xPos;
        goalPos.y=yPos;
    }
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    //常に画面の大きさの変化に応じてgridsizeの変更
    gridSize=ofGetWindowHeight()/resolution;
    //縦横比16:9を維持
    ofSetWindowShape(ofGetWindowHeight()/aspectRate[1]*aspectRate[0],ofGetWindowHeight());
}