#pragma once
#include <vector>
#include <queue>
#include "basePathPlanning.h"

using namespace std;

class BFSPathPlanning : public basePathPlanning {

private:

public:

	Path BFS(double startX, double startY, double endX, double endY, bool haveProduct) {

		int startIdx = -1, startIdy = -1, endIdx = -1, endIdy = -1;
		getLatticeIndex(startX, startY, startIdx, startIdy);
		getLatticeIndex(endX, endY, endIdx, endIdy);
		Path path2;	// 用于存储从end到start的路径

		if (startIdx == endIdx && startIdy == endIdy) {
			path2.push_back(fesible_point_haveProd[startIdy][startIdx]);
			return path2;
		}
		
		lattice path1[100][100];	// 用于存储在BFS过程中所遍历到的点的父节点
		lattice start;
		start.x = startIdx;
		start.y = startIdy;
		queue<lattice> q;
		q.push(start);
		int dx[8] = { 0,1,0,-1,-1,1,1,-1 };
		int dy[8] = { 1,0,-1,0,1,1,-1,-1 };

		bool visit[100][100] = { false };	// 标记是否被访问过
		visit[startIdy][startIdx] = true;

		while (!q.empty()) {

			lattice curNode = q.front();
			q.pop();

			for (int i = 0; i < 8; ++i) {

				lattice newNode;
				int x = curNode.x + dx[i], y = curNode.y + dy[i];

				if (haveProduct) {
					if (x < 0 || x > 99 || y < 0 || y > 99 || visit[y][x] || !fesible_lattice_haveProd[y][x]) {
						continue;
					}
				}
				else {
					if (x < 0 || x > 99 || y < 0 || y > 99 || visit[y][x] || !fesible_lattice_noProd[y][x]) {
						continue;
					}
				}

				if (haveProduct) {
					if (blockDectect(fesible_point_haveProd[curNode.y][curNode.x],
						fesible_point_haveProd[y][x], 0.53))
						continue;
				}
				else {
					if (blockDectect(fesible_point_noProd[curNode.y][curNode.x],
						fesible_point_noProd[y][x], 0.45))
						continue;
				}

				newNode.x = x;
				newNode.y = y;
				q.push(newNode);
				path1[y][x].x = curNode.x;
				path1[y][x].y = curNode.y;
				visit[y][x] = true;

				if (x == endIdx && y == endIdy) {

					if (haveProduct) 
						path2.push_back(fesible_point_haveProd[endIdy][endIdx]);
					else
						path2.push_back(fesible_point_noProd[endIdy][endIdx]);

					int father_x = path1[endIdy][endIdx].x;
					int father_y = path1[endIdy][endIdx].y;
					int temp_x = father_x, temp_y = father_y;
					while (father_x != startIdx || father_y != startIdy) {

						if (haveProduct)
							path2.push_back(fesible_point_haveProd[father_y][father_x]);
						else
							path2.push_back(fesible_point_noProd[father_y][father_x]);

						father_x = path1[temp_y][temp_x].x;
						father_y = path1[temp_y][temp_x].y;
						temp_x = father_x, temp_y = father_y;
					}

					if (haveProduct)
						path2.push_back(fesible_point_haveProd[startIdy][startIdx]);
					else
						path2.push_back(fesible_point_noProd[startIdy][startIdx]);

					return path2;	// 返回从end到start的路径
				}
			}
		}

		return {};		// 找不到路径，返回空数组
	}

	// 在readMap中调用, 传入地图数组的引用, 用来计算路径; 
	void initPaths(vector<vector<char>>& h_map) {

		Path_Wbi2Wbj_HaveProd.resize(H_GlobalContext.wbNum);
		for (auto& one : Path_Wbi2Wbj_HaveProd)
			one.resize(H_GlobalContext.wbNum);
		Path_Wbi2Wbj_NoProd.resize(H_GlobalContext.wbNum);
		for (auto& one : Path_Wbi2Wbj_NoProd)
			one.resize(H_GlobalContext.wbNum);
		Path_Boti2Wbj.resize(4);
		for (auto& one : Path_Boti2Wbj)
			one.resize(H_GlobalContext.wbNum);

		for (int y = 99; y >= 0; --y)
			for (int x = 0; x < 100; ++x)
				if (h_map[y][x] == '#')
					block_lattice[y][x] = true;

		int dx[8] = { -1,0,1,1,1,0,-1,-1 };		// 顺序是，从左上角，顺时针方向
		int dy[8] = { 1,1,1,0,-1,-1,-1,0 };		// 顺序是，从左上角，顺时针方向
		int x_i, y_i;
		for (int y = 99; y >= 0; --y) {
			for (int x = 0; x < 100; ++x) {
				if (block_lattice[y][x])        // 判断该点是否就是障碍物
					continue;

				int allocation = 0;

				for (int i = 0; i < 8; ++i) {   // 判断该点的上下左右和四个角是否有障碍物
					x_i = x + dx[i];
					y_i = y + dy[i];
					if (x_i >= 0 && x_i < 100 && y_i >= 0 && y_i < 100) {
						if (block_lattice[y_i][x_i]) {
							allocation = allocation + (1 << i);
						}
					}
					else {		// 把越界的格子视为障碍物
						allocation = allocation + (1 << i);
					}
				}

				// 使得带货品或不带货品的机器人可通行的81种格子排列--河野万里
				///////////////////////////////////////////////////////////
				// 1
				//                  ...
				// allocation == -> ...
				// 0		        ...
				// 
				// /////////////////////////////////////////
				// 2
				//                  #..                  ..#
				// allocation == -> ... allocation == -> ...
				// 0b1              ... 0b100            ...
				// 
				//                  ...                  ...
				// allocation == -> ... allocation == -> ...
				// 0b10000  	    ..# 0b1000000        #..
				// 
				// /////////////////////////////////////////
				// 3
				//                  .#.                  ...
				// allocation == -> ... allocation == -> ..#
				// 0b10             ... 0b1000           ...
				// 
				//                  ...                  ...
				// allocation == -> ... allocation == -> #..
				// 0b100000		    .#. 0b10000000       ...
				// 
				// /////////////////////////////////////////
				// 4
				//                  #.#                  ..#
				// allocation == -> ... allocation == -> ...
				// 0b101            ... 0b10100          ..#
				// 
				//                  ...                  #..
				// allocation == -> ... allocation == -> ...
				// 0b1010000	    #.# 0b1000001        #..
				// 
				// /////////////////////////////////////////
				// 5
				//                  .#.                  ...
				// allocation == -> ..# allocation == -> ..#
				// 0b1010           ... 0b101000         .#.
				// 
				//                  ...                  .#.
				// allocation == -> #.. allocation == -> #..
				// 0b10100000       .#. 0b10000010       ...
				// 
				// /////////////////////////////////////////
				// 6
				//                  ##.                  .##
				// allocation == -> ... allocation == -> ...
				// 0b11             ... 0b110            ...
				// 
				//                  ..#                  ...
				// allocation == -> ..# allocation == -> ..#
				// 0b1100   	    ... 0b11000          ..#
				// 
				//                  ...                  ...
				// allocation == -> ... allocation == -> ...
				// 0b110000         .## 0b1100000        ##.
				// 
				//                  ...                  #..
				// allocation == -> #.. allocation == -> #..
				// 0b11000000	    #.. 0b10000001       ...
				// 
				// /////////////////////////////////////////
				// 7
				//                  .#.                  .#.
				// allocation == -> ... allocation == -> ...
				// 0b10010          ..# 0b1000010        #..
				// 
				//                  ...                  #..
				// allocation == -> ..# allocation == -> ..#
				// 0b1001000   	    #.. 0b1001           ...
				// 
				//                  #..                  ..#
				// allocation == -> ... allocation == -> ...
				// 0b100001         .#. 0b100100         .#.
				// 
				//                  ..#                  ...
				// allocation == -> #.. allocation == -> #..
				// 0b10000100	    ... 0b10010000       ..#
				// 
				// /////////////////////////////////////////
				// 8
				//                  ###                  ..#
				// allocation == -> ... allocation == -> ..#
				// 0b111            ... 0b11100          ..#
				// 
				//                  ...                  #..
				// allocation == -> ... allocation == -> #..
				// 0b1110000   	    ### 0b11000001       #..
				// 
				// /////////////////////////////////////////
				// 9
				//                  .##                  ...
				// allocation == -> ..# allocation == -> ..#
				// 0b1110           ... 0b111000         .##
				// 
				//                  ...                  ##.
				// allocation == -> #.. allocation == -> #..
				// 0b11100000  	    ##. 0b10000011       ...
				// 
				// /////////////////////////////////////////
				// 10
				//                  ##.                  .##
				// allocation == -> ..# allocation == -> #..
				// 0b1011           ... 0b10000110       ...
				// 
				//                  ..#                  .#.
				// allocation == -> ..# allocation == -> ..#
				// 0b101100   	    .#. 0b11010          ..#
				// 
				//                  ...                  ...
				// allocation == -> #.. allocation == -> ..#
				// 0b10110000       .## 0b1101000        ##.
				// 
				//                  .#.                  #..
				// allocation == -> #.. allocation == -> #..
				// 0b11000010       #.. 0b10100001       .#.
				// 
				// /////////////////////////////////////////
				// 11
				//                  ##.                  .##
				// allocation == -> ... allocation == -> ...
				// 0b1000011        #.. 0b10110          ..#
				// 
				//                  #.#                  ...
				// allocation == -> ..# allocation == -> ..#
				// 0b1101   	    ... 0b1011000        #.#
				// 
				//                  ..#                  #..
				// allocation == -> ... allocation == -> ...
				// 0b110100         .## 0b1100001        ##.
				// 
				//                  ...                  #.#
				// allocation == -> #.. allocation == -> #..
				// 0b11010000	    #.# 0b10000101       ...
				// 
				// /////////////////////////////////////////
				// 12
				//                  ##.                  ..#
				// allocation == -> ..# allocation == -> ..#
				// 0b11011          ..# 0b1101100        ##.
				// 
				//                  #..                  .##
				// allocation == -> #.. allocation == -> #..
				// 0b10110001       .## 0b11000110       #..
				// 
				// /////////////////////////////////////////
				// 13
				//                  ###                  .##
				// allocation == -> ..# allocation == -> ..#
				// 0b1111           ... 0b11110          ..#
				// 
				//                  ..#                  ...
				// allocation == -> ..# allocation == -> ..#
				// 0b111100         .## 0b1111000        ###
				// 
				//                  ...                  #..
				// allocation == -> #.. allocation == -> #..
				// 0b11110000       ### 0b11100001       ##.
				// 
				//                  ##.                  ###
				// allocation == -> #.. allocation == -> #..
				// 0b11000011       #.. 0b10000111       ...
				// 
				// /////////////////////////////////////////
				// 14
				//                  ###                  ###
				// allocation == -> ... allocation == -> ...
				// 0b10111          ..# 0b1000111        #..
				// 
				//                  ..#                  #.#
				// allocation == -> ..# allocation == -> ..#
				// 0b1011100        #.# 0b11101          ..#
				// 
				//                  ..#                  #..
				// allocation == -> ... allocation == -> ...
				// 0b1110100        ### 0b1110001        ###
				// 
				//                  #.#                  #..
				// allocation == -> #.. allocation == -> #..
				// 0b11000101       #.. 0b11010001       #.#
				// 
				// /////////////////////////////////////////
				// 15
				//                  ###                  ..#
				// allocation == -> ..# allocation == -> ..#
				// 0b11111          ..# 0b1111100        ###
				// 
				//                  #..                  ###
				// allocation == -> #.. allocation == -> #..
				// 0b11110001       ### 0b11000111       #..
				// 
				// /////////////////////////////////////////

				switch (allocation)
				{ 
				case 0:	// 1
					fesible_lattice_noProd[y][x] = true;
					fesible_lattice_haveProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.25, y * 0.5 + 0.25);
					fesible_point_haveProd[y][x] = Point(x * 0.5 + 0.25, y * 0.5 + 0.25);
					break;

				case 0b1:	// 2
					fesible_lattice_noProd[y][x] = true;
					fesible_lattice_haveProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					fesible_point_haveProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;
				case 0b100:
					fesible_lattice_noProd[y][x] = true;
					fesible_lattice_haveProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					fesible_point_haveProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b10000:
					fesible_lattice_noProd[y][x] = true;
					fesible_lattice_haveProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					fesible_point_haveProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b1000000:
					fesible_lattice_noProd[y][x] = true;
					fesible_lattice_haveProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					fesible_point_haveProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;

				case 0b10:	// 3
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.25, y * 0.5);
					break;
				case 0b1000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.25);
					break;
				case 0b100000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.25, y * 0.5 + 0.5);
					break;
				case 0b10000000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.25);
					break;

				case 0b101:	// 4
					fesible_lattice_noProd[y][x] = true;
					fesible_lattice_haveProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.25, y * 0.5);
					fesible_point_haveProd[y][x] = Point(x * 0.5 + 0.25, y * 0.5);
					break;
				case 0b10100:
					fesible_lattice_noProd[y][x] = true;
					fesible_lattice_haveProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.25);
					fesible_point_haveProd[y][x] = Point(x * 0.5, y * 0.5 + 0.25);
					break;
				case 0b1010000:
					fesible_lattice_noProd[y][x] = true;
					fesible_lattice_haveProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.25, y * 0.5 + 0.5);
					fesible_point_haveProd[y][x] = Point(x * 0.5 + 0.25, y * 0.5 + 0.5);
					break;
				case 0b1000001:
					fesible_lattice_noProd[y][x] = true;
					fesible_lattice_haveProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.25);
					fesible_point_haveProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.25);
					break;

				case 0b1010:	// 5
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b101000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b10100000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;
				case 0b10000010:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;

				case 0b11:	// 6
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.25, y * 0.5);
					break;
				case 0b110:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.25, y * 0.5);
					break;
				case 0b1100:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.25);
					break;
				case 0b11000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.25);
					break;
				case 0b110000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.25, y * 0.5 + 0.5);
					break;
				case 0b1100000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.25, y * 0.5 + 0.5);
					break;
				case 0b11000000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.25);
					break;
				case 0b10000001:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.25);
					break;

				case 0b10010:	// 7
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b1000010:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;
				case 0b1001000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b1001:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b100001:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;
				case 0b100100:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b10000100:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;
				case 0b10010000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;

				case 0b111:	// 8
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.25, y * 0.5);
					break;
				case 0b11100:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.25);
					break;
				case 0b1110000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.25, y * 0.5 + 0.5);
					break;
				case 0b11000001:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.25);
					break;

				case 0b1110:	// 9
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b111000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b11100000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;
				case 0b10000011:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;

				case 0b1011:	// 10
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b10000110:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;
				case 0b101100:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b11010:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b10110000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;
				case 0b1101000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b11000010:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;
				case 0b10100001:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;

				case 0b1000011:	// 11
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;
				case 0b10110:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b1101:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b1011000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b110100:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b1100001:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;
				case 0b11010000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;
				case 0b10000101:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;

				case 0b11011:	// 12
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b1101100:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b10110001:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;
				case 0b11000110:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;

				case 0b1111:	// 13
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b11110:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b111100:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b1111000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b11110000:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;
				case 0b11100001:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;
				case 0b11000011:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;
				case 0b10000111:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;

				case 0b10111:	// 14
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b1000111:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;
				case 0b1011100:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b11101:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b1110100:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b1110001:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;
				case 0b11000101:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;
				case 0b11010001:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;

				case 0b11111:	// 15
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5);
					break;
				case 0b1111100:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5, y * 0.5 + 0.5);
					break;
				case 0b11110001:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5 + 0.5);
					break;
				case 0b11000111:
					fesible_lattice_noProd[y][x] = true;
					fesible_point_noProd[y][x] = Point(x * 0.5 + 0.5, y * 0.5);
					break;

				default:
					break;
				}
				
			}
		}

		for (int i = 0; i < H_GlobalContext.wbNum; ++i) {
			for (int j = i + 1; j < H_GlobalContext.wbNum; ++j) {
				
				Path PathijHaveProd = // 目前BFS返回来的路径是反过来的
					BFS(H_GlobalContext.wbArr[j].x, H_GlobalContext.wbArr[j].y, 
						H_GlobalContext.wbArr[i].x, H_GlobalContext.wbArr[i].y, true);
				if (PathijHaveProd.pointList.size() == 0)
					continue;
				Path_Wbi2Wbj_HaveProd[i][j] = PathijHaveProd;
				Path_Wbi2Wbj_HaveProd[j][i] = PathijHaveProd.reverse();

				Path PathijNoProd = // 目前BFS返回来的路径是反过来的
					BFS(H_GlobalContext.wbArr[j].x, H_GlobalContext.wbArr[j].y, 
						H_GlobalContext.wbArr[i].x, H_GlobalContext.wbArr[i].y, false);
				if (PathijNoProd.pointList.size() == 0)
					continue;
				Path_Wbi2Wbj_NoProd[i][j] = PathijNoProd;
				Path_Wbi2Wbj_NoProd[j][i] = PathijNoProd.reverse();
	
			}
		}

		for (int i = 0; i < H_GlobalContext.wbNum; ++i) {
			Path p;
			p.push_back(H_GlobalContext.wbArr[i].x, H_GlobalContext.wbArr[i].y);
			Path_Wbi2Wbj_HaveProd[i][i] = p;
			Path_Wbi2Wbj_NoProd[i][i] = p;
		}

		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < H_GlobalContext.wbNum; ++j) {
				Path Pathij = // 目前BFS返回来的路径是反过来的
					BFS(H_GlobalContext.wbArr[j].x, H_GlobalContext.wbArr[j].y,
						H_GlobalContext.botArr[i].x, H_GlobalContext.botArr[i].y, false);
				if (Pathij.pointList.size() == 0)
					continue;
				Path_Boti2Wbj[i][j] = Pathij;

			}
		}
	}

	bool blockDectect(const Point& p1, const Point& p2, double r) {	// 检测到障碍物，返回true
		
		double delta = 0.2;
		double Ax = p1.x, Ay = p1.y, Bx = p2.x, By = p2.y;
		double Cx, Cy, Dx, Dy, Ex, Ey, Fx, Fy;
		double deltaX = Ax - Bx, deltaY = Ay - By;
		double dist = sqrt(deltaX * deltaX + deltaY * deltaY);

		if (Ax == Bx) {		// 特殊情况特殊处理
			Cx = Ax - r, Cy = Ay;
			Dx = Ax + r, Dy = Ay;
			Ex = Bx - r, Ey = By;
			Fx = Bx + r, Fy = By;
		}
		else if (Ay == By) { // 特殊情况特殊处理
			Cx = Ax, Cy = Ay - r;
			Dx = Ax, Dy = Ay + r;
			Ex = Bx, Ey = By - r;
			Fx = Bx, Fy = By + r;
		}
		else {
			double theta1 = fabs(atan2(deltaY, deltaX));
			if (theta1 > M_PI_2)	// 强行扳回锐角
				theta1 = M_PI - theta1;
			double theta2 = M_PI_2 - theta1;	// 保证theta2是正的锐角
			if (deltaY * deltaX > 0) { // 同号，斜率为正
				Cx = Ax - r * cos(theta2), Cy = Ay + r * sin(theta2);
				Dx = Ax + r * cos(theta2), Dy = Ay - r * sin(theta2);
				Ex = Bx - r * cos(theta2), Ey = By + r * sin(theta2);
				Fx = Bx + r * cos(theta2), Fy = By - r * sin(theta2);
			}
			else {	// 异号，斜率为负
				Cx = Ax - r * cos(theta2), Cy = Ay - r * sin(theta2);
				Dx = Ax + r * cos(theta2), Dy = Ay + r * sin(theta2);
				Ex = Bx - r * cos(theta2), Ey = By - r * sin(theta2);
				Fx = Bx + r * cos(theta2), Fy = By + r * sin(theta2);
			}
		}
		double np = ceil(dist / delta);	// 往上取整，使得点更密集
		delta = dist / np;	// 得到一个能均分dist的delta
		double tmpX1, tmpY1, tmpX2, tmpY2;
		int idx1, idy1, idx2, idy2;
		for (int i = 1; i < np; ++i) {
			tmpX1 = Cx - deltaX * (i / np), tmpY1 = Cy - deltaY * (i / np);
			tmpX2 = Dx - deltaX * (i / np), tmpY2 = Dy - deltaY * (i / np);
			getLatticeIndex(tmpX1, tmpY1, idx1, idy1);
			getLatticeIndex(tmpX2, tmpY2, idx2, idy2);
			if (block_lattice[idy1][idx1] || block_lattice[idy2][idx2])
				return true;
		}
		return false;
	}

	// 获得从 fromWb 到 toWb的路径
	Path getPath(const hwWorkbench& fromWb, const hwWorkbench& toWb, bool haveProd) {
		if (haveProd)
			return Path_Wbi2Wbj_HaveProd[fromWb.wbId][toWb.wbId];
		else
			return Path_Wbi2Wbj_NoProd[fromWb.wbId][toWb.wbId];
	}

	// 获得从 fromPoint 到 toWb的路径
	Path getPath(const Point& fromPoint, const hwWorkbench& toWb, bool haveProd) {
		if (haveProd)
			return BFS(toWb.x, toWb.y, fromPoint.x, fromPoint.y, true);
		else
			return BFS(toWb.x, toWb.y, fromPoint.x, fromPoint.y, false);
	}

	// 获得从 fromPoint 到 toPoint的路径
	Path getPath(const Point& fromPoint, const Point& toPoint, bool haveProd) {
		if (haveProd)
			return BFS(toPoint.x, toPoint.y, fromPoint.x, fromPoint.y, true);
		else
			return BFS(toPoint.x, toPoint.y, fromPoint.x, fromPoint.y, false);
	}

	// 获得从 fromWb 到 toPoint的路径
	Path getPath(const hwWorkbench& fromWb, const Point& toPoint) {
		Path p;
		return p;
	}

	// 获得从 fromBot 到 toWb的路径
	Path getPath(const hwRobot_2& fromBot, const hwWorkbench& toWb) {
		return Path_Boti2Wbj[fromBot.botId][toWb.wbId];
	}

};
