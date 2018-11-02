#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
    gridSize=ofGetWindowHeight()/resolution;
    ofSetWindowShape(ofGetWindowHeight()/9*16,ofGetWindowHeight());
    for(int i=0;i<resolution/9*16;i++)
    {
        vector<double> obs;
        for(int j=0;j<resolution;j++)
        {
            obs.push_back(0);
        }
        obstacle.push_back(obs);
    }
    
    startPos.x  = 0;
    startPos.y  = (int)(resolution/2)-1;
    goalPos.x   = (int)(resolution/9*16)-1;
    goalPos.y   = (int)(resolution/2)-1;
    ofSetFrameRate(60);
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
        if(solve())
        {
            getPath();
            state=0;
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    //guiの描画
    gui.draw();

    //gridの描画　

    //open closeの描画
    for(int i=0;i<Open.size();i++)
    {
        ofSetColor(255,0,255);
        ofFill();
        ofRect(ofPoint(Open[i][0]*gridSize,Open[i][1]*gridSize),gridSize,gridSize);
    }
    for(int i=0;i<Close.size();i++)
    {
        ofSetColor(255,255,0);
        ofFill();
        ofRect(ofPoint(Close[i][0]*gridSize,Close[i][1]*gridSize),gridSize,gridSize);
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
        for(int j=0;j<resolution/9*16;j++)
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



void ofApp::init()
{
     TotalCost.clear();
        for (int i=0;i<48;i++)
        {
            vector<double>totalcost;
            for (int j=0;j<27;j++)
            {
                totalcost.push_back(0);
            }
            TotalCost.push_back(totalcost);
        }

        HCost.clear();
        for (int i=0;i<48;i++)
        {
            vector<double>hcost;
            for (int j=0;j<27;j++)
            {
                hcost.push_back(0);
            }
            HCost.push_back(hcost);
        }

        //Open,Closeを用意
        Open.clear();
        open.clear();
        Close.clear();
        close.clear();

        //StartをOpenに入れる
        open.push_back(startPos.x);
        open.push_back(startPos.y);
        open.push_back(0);
        Open.push_back(open);
        open.clear();

        HCost[startPos.x][startPos.y]=sqrt(pow((Open[0][0]-goalPos.y),2)+pow((Open[0][1]-goalPos.y),2));
        TotalCost[Open[0][0]][Open[0][1]]=HCost[Open[0][0]][Open[0][1]];
    
        //親のメモリ確保
        Parent.clear();
        parent.clear();
        parent.push_back(Open[0][0]);
        parent.push_back(Open[0][1]);
        parent.push_back(Open[0][2]);
        parent.push_back(Open[0][0]);
        parent.push_back(Open[0][1]);
        parent.push_back(Open[0][2]);
        Parent.push_back(parent);
        parent.clear();

        //n,mの確保
        n.clear();
        m.clear();
        for(int i=0;i<3;i++)
        {
            n.push_back(0);
            m.push_back(0);
        }

        path.clear();
}

bool ofApp::solve()//解けた時true,途中の時false
{
    //cout<<"  1"<<endl;
    //①OPENリストの中身がなくなったら終了(失敗)
    if (Open.size()==0)
    {
        return true;
    }
    
    if (Parent.size()>3000)
    {
        return true;
    }
    
    //cout<<"  2"<<endl;
    //②OPENリストの中でTotalCostが最小のノードを探す
    double minimumCost=TotalCost[Open[0][0]][Open[0][1]];
    for(int i=0;i<Open.size();i++)
    {
        //cout<<"  "<<Open[i][0]<<" "<<Open[i][1]<<endl;
        if(minimumCost>=TotalCost[Open[i][0]][Open[i][1]])
        {
            minimumCost=TotalCost[Open[i][0]][Open[i][1]];
            n[0]=Open[i][0];
            n[1]=Open[i][1];
            n[2]=Open[i][2];
        }
    }
    //cout<<"    n"<<n[0]<<" "<<n[1]<<endl;
    //cout<<"  3"<<endl;
    //③nがGの時終了，それ以外の時はnをCloseへ
    //cout<<"    open"<<Open.size()<<endl;
    //cout<<"    close"<<Close.size()<<endl;
    if(n[0]==goalPos.x && n[1]==goalPos.y)
    {
        //cout<<"  finish!!!!!"<<endl;
        return true;
    }
    else
    {
        Close.push_back(n);
        for(int i=0;i<Open.size();i++)
        {
            if(Open[i][0]==n[0] && Open[i][1]==n[1])
            {
                Open.erase(Open.begin()+i);
                //return false;
            }
        }
    }
    //cout<<"    open"<<Open.size()<<endl;
    //cout<<"    close"<<Close.size()<<endl;
    //cout<<"  4"<<endl;
    //④nに隣接するノードに対してアクセス
    for(int i=-1;i<2;i++)
    {
        for(int j=-1;j<2;j++)
        {
            if(i==0 && j==0)
            {

            }
            else
            {
                
                m[0]=n[0]+i;
                m[1]=n[1]+j;
                //cout<<"    a  "<<m[0]<<" "<<m[1]<<endl;
                if(m[0]>=0 && m[0]<=obstacle.size()-1 && m[1]>=0 && m[1]<=obstacle[0].size()-1)
                {
                    //cout<<"    b"<<endl;
                    //仮のコストtotalcostの計算
                    HCost[m[0]][m[1]]=sqrt(pow((m[0]-goalPos.x),2)+pow((m[1]-goalPos.y),2));
                    //HCost[m[0]][m[1]]=pow((m[0]-Goal[0]),2)+pow((m[1]-Goal[1]),2);
                    double Cost=obstacle[m[0]][m[1]]-obstacle[n[0]][n[1]];
                    //cout<<"Cost  "<<Cost<<endl;
                    double totalcost=TotalCost[n[0]][n[1]]-HCost[n[0]][n[1]]+HCost[m[0]][m[1]]+Cost+sqrt(i*i+j*j);

                    //cout<<"    c"<<endl;
                    //ステータスの判定
                    int status=0; //open:1,close:2,それ以外:0
                    //cout<<"    opensize"<<Open.size()<<endl;
                    for(int k=0;k<Open.size();k++)
                    {
                        //cout<<"    open"<<Open[0][0]<<" "<<Open[0][1]<<endl;
                        if(Open[k][0]==m[0] && Open[k][1]==m[1])
                        {
                            status=1;
                        }
                    }
                    if(status==0)
                    {
                        //cout<<"    c0"<<endl;
                        for(int k=0;k<Close.size();k++)
                        {
                            if(Close[k][0]==m[0] && Close[k][1]==m[1])
                            {
                                status=2;
                            }
                        }
                    }
                    //cout<<"    d"<<endl;
                    //ステータスに応じて操作
                    if(status==0)
                    {
                        //cout<<"    0"<<endl;
                        //cout<<"       1"<<endl;
                        TotalCost[m[0]][m[1]]=totalcost;
                        Open.push_back(m);
                        //cout<<"       2"<<endl;
                        parent.push_back(m[0]);
                        parent.push_back(m[1]);
                        parent.push_back(m[2]);
                        parent.push_back(n[0]);
                        parent.push_back(n[1]);
                        parent.push_back(n[2]);
                        Parent.push_back(parent);
                        parent.clear();
                        //cout<<"       4"<<endl;
                        //cout<<"    m"<<m.size()<<"  "<<m[0]<<" "<<m[1]<<" "<<m[2]<<endl;
                        //cout<<"    open"<<Open.size()<<endl;
                        //cout<<"    open"<<Open[0].size()<<endl;
                        //cout<<"    open"<<Open.size()<<"  "<<Open[0][0]<<" "<<Open[0][1]<<endl;
                    }
                    else if(status==1)
                    {
                        //cout<<"    1"<<endl;
                        if(TotalCost[m[0]][m[1]]>totalcost)
                        {
                            TotalCost[m[0]][m[1]]=totalcost;
                            parent.push_back(m[0]);
                            parent.push_back(m[1]);
                            parent.push_back(m[2]);
                            parent.push_back(n[0]);
                            parent.push_back(n[1]);
                            parent.push_back(n[2]);
                            Parent.push_back(parent);
                            parent.clear();
                        }
                    }
                    else
                    {
                        //cout<<"    2"<<endl;
                        if(TotalCost[m[0]][m[1]]>totalcost)
                        {
                            TotalCost[m[0]][m[1]]=totalcost;
                            parent.push_back(m[0]);
                            parent.push_back(m[1]);
                            parent.push_back(m[2]);
                            parent.push_back(n[0]);
                            parent.push_back(n[1]);
                            parent.push_back(n[2]);
                            Parent.push_back(parent);
                            parent.clear();
                        }
                    }
                }
            }
        }
    }
    return false;
}

void ofApp::getPath()
{
    //Parentをたどっていく
    //pathlistの確保
    vector < vector<int> >Pathlist;
    vector<int> parent2;
    parent2.push_back(goalPos.x);
    parent2.push_back(goalPos.y);
    parent2.push_back(4);
    
    double Child[3];
    double minimumCost=obstacle[Parent[0][0]][Parent[0][1]];

    Child[0]=goalPos.x;
    Child[1]=goalPos.y;
    Child[2]=4;
    Pathlist.push_back(parent2);
   
    while(1)
    {
        for(int i=0;i<Parent.size();i++)
        {
            int p=Parent.size()-i-1;
            //cout<<"11  "<<Parent[i][0]<<" "<<Parent[i][1]<<endl;
            if(Parent[i][0]==Child[0] && Parent[i][1]==Child[1])
            {
                //cout<<"111"<<endl;
                parent2[0]=Parent[i][0];
                parent2[1]=Parent[i][1];
                parent2[2]=Parent[i][2];
                Pathlist.push_back(parent2);
                Child[0]=Parent[i][3];
                Child[1]=Parent[i][4];
                Child[2]=Parent[i][5];
                //cout<<"112"<<endl;
                break;
            }
        }
        //cout<<"2"<<endl;
        if(startPos.x==Child[0] && startPos.y==Child[1])
        {
            //cout<<"21"<<endl;
            parent2[0]=startPos.x;
            parent2[1]=startPos.y;
            parent2[2]=0;
            //cout<<"22"<<endl;
            Pathlist.push_back(parent2);
            break;
        }
        //cout<<"3"<<endl;
    }
    //cout<<"4"<<endl;
    reverse(Pathlist.begin(), Pathlist.end());  
    //cout<<"5"<<endl;
    cout<<"generated Path"<<endl;
    path=Pathlist;
}

void ofApp::clearObstacle()
{
    path.clear();
    for(int i=0;i<48;i++)
    {
        for(int j=0;j<27;j++)
        {
            obstacle[i][j]=0;
        }   
    }
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key=='s')
    {
        init();
        state=1;
    }
    else if(key=='c')
    {
        init();
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
    ofSetWindowShape(ofGetWindowHeight()/9*16,ofGetWindowHeight());
}



