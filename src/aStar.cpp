#include "aStar.h"
aStar::aStar() {
    _open.clear();
    _open.assign(1, vector<int>(3, 0));

    _close.clear();
    _close.assign(1, vector<int>(3, 0));
}
//--------------------------------------------------------------
void aStar::setCostMap(gridField costMap) { _costMap = costMap; }
//--------------------------------------------------------------
void aStar::init(ofPoint startPos, ofPoint goalPos) {
    // set start and goal
    _startPos = startPos;
    _goalPos = goalPos;
    
    // Open,Close init

    // n,mの確保
    _n.clear();
    _n.resize(3);
    _m.clear();
    _m.resize(3);
    // 二回目以降はコストを保持する
    cout << "a" << endl;
    _totalCost.clear();
    _totalCost.assign(_costMap.getFieldWidthSize(),
                        vector<double>(_costMap.getFieldHeightSize(), 12000000));

    _hCost.clear();
    _hCost.assign(_costMap.getFieldWidthSize(),
                    vector<double>(_costMap.getFieldHeightSize(), 12000000));
    // Cost init
    if (_close.size() == 1) {
        

        //親のメモリ確保
        _parent.clear();
        _parent.assign(1, vector<int>(6, 0));
        _parent[0][0] = _open[0][0];
        _parent[0][1] = _open[0][1];
        _parent[0][2] = _open[0][2];
        _parent[0][3] = _open[0][0];
        _parent[0][4] = _open[0][1];
        _parent[0][5] = _open[0][2];

        _n[0] = _startPos.x;
        _n[1] = _startPos.y;
        _n[2] = 0;
        _open.clear();
        _open.push_back(_n);
        _beforeGoalPos = goalPos;
    } else {
        double nowDistance =
            sqrt(pow((_startPos.x - goalPos.x), 2) + pow((_startPos.y - goalPos.y), 2));
        double pastDistance = sqrt(pow((_startPos.x - _beforeGoalPos.x), 2) +
                                   pow((_startPos.y - _beforeGoalPos.y), 2));
        if (nowDistance > pastDistance) {

            _open.clear();
            _close.clear();
            _beforeGoalPos = goalPos;
        }
    }

    /*cout<<"a"<<endl;


    */
    // StartをOpenに入れる

    bool find = false;
    int findOutIndex = 0;
    _close.insert(_close.end(),_open.begin(),_open.end());
    for (int k = 0; k < _close.size(); k++) {
        if (_close[k][0] == _goalPos.x && _close[k][1] == _goalPos.y) {
            find = true;
            findOutIndex = k;
        }
    }
    if (find) {
        _open.clear();
        /*if(!openCleared)
        {
            _beforeOpen =  _open;
            openCleared = true;
        }*/
        
        //_close.erase(_close.begin() + findOutIndex);
    } else {

        cout << "add open" << endl;
    
        _open.clear();
        /*if(openCleared)
        {
            _open = _beforeOpen;
            openCleared = false;
        }*/
        _n[0] = _startPos.x;
        _n[1] = _startPos.y;
        _n[2] = 0;
        _open.push_back(_n);
        _hCost[_startPos.x][_startPos.y] =
            sqrt(pow((_open[0][0] - goalPos.x), 2) + pow((_open[0][1] - goalPos.y), 2));
        _totalCost[_open[0][0]][_open[0][1]] = _hCost[_open[0][0]][_open[0][1]];
        
    }
    // pathの確保
    _path.clear();
}
//--------------------------------------------------------------
bool aStar::solveOneStep() {
    cout << "  1" << endl;
    //①OPENリストの中身がなくなったら終了(失敗)
    cout << "    open" << _open.size() << endl;
    if (_open.size() == 0) {
        return true;
    }

    /*if (_parent.size() > 3000) {
      return true;
    }*/

    cout << "  2" << endl;
    //②OPENリストの中でTotalCostが最小のノードを探す
    double minimumCost = _totalCost[_open[0][0]][_open[0][1]];
    for (int i = 0; i < _open.size(); i++) {
        // cout<<"  "<<Open[i][0]<<" "<<Open[i][1]<<endl;
        if (minimumCost >= _totalCost[_open[i][0]][_open[i][1]]) {
            minimumCost = _totalCost[_open[i][0]][_open[i][1]];
            _n[0] = _open[i][0];
            _n[1] = _open[i][1];
            _n[2] = _open[i][2];
        }
    }

    // cout<<"    n"<<n[0]<<" "<<n[1]<<endl;
    cout << "  3" << endl;
    //③nがGの時終了，それ以外の時はnをCloseへ
    cout << "    open" << _open.size() << endl;
    cout << "    close" << _close.size() << endl;
    if (_n[0] == _goalPos.x && _n[1] == _goalPos.y) {
        // cout<<"  finish!!!!!"<<endl;
        return true;
    } else {
        _close.push_back(_n);
        for (int i = 0; i < _open.size(); i++) {
            if (_open[i][0] == _n[0] && _open[i][1] == _n[1]) {
                _open.erase(_open.begin() + i);
                // return false;
            }
        }
    }

    // cout<<"    close"<<Close.size()<<endl;
    cout << "  4" << endl;
    //④nに隣接するノードに対してアクセス
    for (int i = -1; i < 2; i++) {
        for (int j = -1; j < 2; j++) {
            if (i == 0 && j == 0) {
            } else {
                _m[0] = _n[0] + i;
                _m[1] = _n[1] + j;
                // cout<<"    a  "<<m[0]<<" "<<m[1]<<endl;
                if (_m[0] >= 0 && _m[0] <= _costMap.getFieldWidthSize() - 1 && _m[1] >= 0 &&
                    _m[1] <= _costMap.getFieldHeightSize() - 1) {
                    // cout<<"    b"<<endl;
                    //仮のコストtotalcostの計算
                    _hCost[_m[0]][_m[1]] =
                        sqrt(pow((_m[0] - _goalPos.x), 2) + pow((_m[1] - _goalPos.y), 2)) *
                        _costMap.getGridSize();
                    // HCost[m[0]][m[1]]=pow((m[0]-Goal[0]),2)+pow((m[1]-Goal[1]),2);
                    double cost = _costMap.getValue(_m[0], _m[1]) - _costMap.getValue(_n[0], _n[1]);
                    // cout<<"Cost  "<<Cost<<endl;
                    double totalcost = _totalCost[_n[0]][_n[1]] - _hCost[_n[0]][_n[1]] +
                                       _hCost[_m[0]][_m[1]] + cost +
                                       (double)sqrt(i * i + j * j) * _costMap.getGridSize();

                    // cout<<"    c"<<endl;
                    //ステータスの判定
                    int status = 0;  // open:1,close:2,それ以外:0
                    // cout<<"    opensize"<<Open.size()<<endl;
                    int findOutIndex;
                    for (int k = 0; k < _open.size(); k++) {
                        // cout<<"    open"<<Open[0][0]<<" "<<Open[0][1]<<endl;
                        if (_open[k][0] == _m[0] && _open[k][1] == _m[1]) {
                            status = 1;
                            findOutIndex = k;
                        }
                    }
                    if (status == 0) {
                        // cout<<"    c0"<<endl;
                        for (int k = 0; k < _close.size(); k++) {
                            if (_close[k][0] == _m[0] && _close[k][1] == _m[1]) {
                                status = 2;
                                findOutIndex = k;
                            }
                        }
                    }

                    // cout<<"    d"<<endl;
                    //ステータスに応じて操作
                    vector<int> parent;
                    parent.resize(6);
                    if (status == 0) {
                        cout << "    0" << endl;
                        // cout<<"       1"<<endl;
                        _totalCost[_m[0]][_m[1]] = totalcost;
                        _open.push_back(_m);
                        // cout<<"       2"<<endl;
                        parent[0] = _m[0];
                        parent[1] = _m[1];
                        parent[2] = _m[2];
                        parent[3] = _n[0];
                        parent[4] = _n[1];
                        parent[5] = _n[2];
                        _parent.push_back(parent);

                        // cout<<"       4"<<endl;
                        // cout<<"    m"<<m.size()<<"  "<<m[0]<<" "<<m[1]<<" "<<m[2]<<endl;
                        // cout<<"    open"<<Open.size()<<endl;
                        // cout<<"    open"<<Open[0].size()<<endl;
                        // cout<<"    open"<<Open.size()<<"  "<<Open[0][0]<<" "<<Open[0][1]<<endl;
                    } else if (status == 1) {
                        cout << "    1" << endl;
                        if (_totalCost[_m[0]][_m[1]] > totalcost) {
                            _totalCost[_m[0]][_m[1]] = totalcost;
                            parent[0] = _m[0];
                            parent[1] = _m[1];
                            parent[2] = _m[2];
                            parent[3] = _n[0];
                            parent[4] = _n[1];
                            parent[5] = _n[2];
                            _parent.push_back(parent);
                        }
                    } else {
                        cout << "    2" << endl;
                        cout << _totalCost[_m[0]][_m[1]] << "  " << totalcost << endl;
                        if (_totalCost[_m[0]][_m[1]] > totalcost) {
                            _totalCost[_m[0]][_m[1]] = totalcost;

                            _close.erase(_close.begin() + findOutIndex);
                            _open.push_back(_m);
                            parent[0] = _m[0];
                            parent[1] = _m[1];
                            parent[2] = _m[2];
                            parent[3] = _n[0];
                            parent[4] = _n[1];
                            parent[5] = _n[2];
                            _parent.push_back(parent);
                        }
                    }
                }
            }
        }
    }
    return false;
}
//--------------------------------------------------------------
vector<vector<int> > aStar::getPath() {
    // Parentをたどっていく
    // pathlistの確保
    vector<vector<int> > pathList;
    vector<int> parent2;
    parent2.resize(3);
    parent2[0] = _goalPos.x;
    parent2[1] = _goalPos.y;
    parent2[2] = 4;
    pathList.push_back(parent2);

    double child[3];
    child[0] = _goalPos.x;
    child[1] = _goalPos.y;
    child[2] = 4;

    while (1) {
        for (int i = 0; i < _parent.size(); i++) {
            int p = _parent.size() - i - 1;
            // cout<<"11  "<<Parent[i][0]<<" "<<Parent[i][1]<<endl;
            if (_parent[i][0] == child[0] && _parent[i][1] == child[1]) {
                // cout<<"111"<<endl;
                parent2[0] = _parent[i][0];
                parent2[1] = _parent[i][1];
                parent2[2] = _parent[i][2];
                pathList.push_back(parent2);
                child[0] = _parent[i][3];
                child[1] = _parent[i][4];
                child[2] = _parent[i][5];
                // cout<<"112"<<endl;
                break;
            }
        }
        // cout<<"2"<<endl;
        if (_startPos.x == child[0] && _startPos.y == child[1]) {
            // cout<<"21"<<endl;
            parent2[0] = _startPos.x;
            parent2[1] = _startPos.y;
            parent2[2] = 0;
            // cout<<"22"<<endl;
            pathList.push_back(parent2);
            break;
        }
        // cout<<"3"<<endl;
    }
    // cout<<"4"<<endl;
    reverse(pathList.begin(), pathList.end());
    // cout<<"5"<<endl;
    cout << "generated Path" << endl;
    return (pathList);
}

//--------------------------------------------------------------
vector<vector<int> > aStar::getOpen() { return _open; }
//--------------------------------------------------------------
vector<vector<int> > aStar::getClose() { return _close; }