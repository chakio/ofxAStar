[![MIT licensed](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
ofxAStar
====

* A*SearchAlgorithmの実装   
![result](https://github.com/chakio/ofxAStar/blob/master/media/demo.gif)  
![result](https://github.com/chakio/ofxAStar/blob/master/media/demo2.gif)
## Description
ロボットを移動させる際、障害物などを考慮した経路計画（ロボットのためのルートの決定）が不可欠です。  
そこで、経路計画手法の一つである探索アルゴリズムのA* Search Algorithmを実装しました。  
迷路などにおけるスタートからゴールまでの経路を探索します。  
自分で好きな迷路を作成し、解くことが可能なソフトウエアです。  

## Requirement 
* Ubuntu16.04
* openFrameworks v0.10.0
* visualstudio code

## Useage
マウスクリックやドラッグで障害物を設計した後
キーボードのsで探索を開始します。  
探索後や障害物を再設計する場合にはキーボードでcを入力するとクリアできます。

## Originality
* 障害物を簡単に設計可能
* 探索途中の描画

## Lisence
[MIT](https://github.com/chakio/ofxAStar/blob/master/LICENSE)

## Author
[chakio](https://github.com/chakio)