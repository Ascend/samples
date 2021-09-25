/**
 * Copyright 2021 Huawei Technologies Co., Ltd

 * XiangQi Wizard Light - A Very Simple Chinese Chess Program
 * Designed by Morning Yellow, Version: 0.6, Last Modified: Mar. 2008
 * Copyright (C) 2004-2008 www.xqbase.com
**/
using namespace std;
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <cstring>
#include <vector>
#include <bits/stdc++.h>
#include <cstdio>
#include <cerrno>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#define MAXLINE 4096
#define FALSE 0
#define TRUE 1
typedef unsigned short WORD;
typedef unsigned long DWORD;
typedef unsigned char BYTE;
typedef int WINBOOL,*PWINBOOL,*LPWINBOOL;
typedef WINBOOL BOOL;

// chessboard margin
const int RANK_TOP = 3;
const int RANK_BOTTOM = 12;
const int FILE_LEFT = 3;
const int FILE_RIGHT = 11;

// chess ID
const int PIECE_KING = 0;
const int PIECE_ADVISOR = 1;
const int PIECE_BISHOP = 2;
const int PIECE_KNIGHT = 3;
const int PIECE_ROOK = 4;
const int PIECE_CANNON = 5;
const int PIECE_PAWN = 6;

// other cons
const int MAX_GEN_MOVES = 128; // max generated moves
const int MAX_MOVES = 256;     // max history moves
const int LIMIT_DEPTH = 64;    // search depth
const int MATE_VALUE = 10000;  // value of MATE
const int BAN_VALUE = MATE_VALUE - 100; // continuous check = lose
const int WIN_VALUE = MATE_VALUE - 200; // value bound (exceeding this value means that kill move is found)
const int DRAW_VALUE = 20;     // draw
const int ADVANCED_VALUE = 3;  //
const int RANDOM_MASK = 7;     //
const int NULL_MARGIN = 400;   // empty step
const int NULL_DEPTH = 2;      //
const int HASH_SIZE = 1 << 20; // replacement table
const int HASH_ALPHA = 1;      // alpha
const int HASH_BETA = 2;       // beta
const int HASH_PV = 3;         // PV
const int BOOK_SIZE = 16384;   // start book

// check if chess is in chessboard
static const char ccInBoard[256] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// check if chess is in fort
static const char ccInFort[256] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// check legal span of some certain classes
static const char ccLegalSpan[512] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 3, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 2, 1, 2, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 2, 1, 2, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 3, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0
};

// check knight
static const char ccKnightPin[512] = {
        0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,-16,  0,-16,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0, -1,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0, -1,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0, 16,  0, 16,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0
};

// shuai
static const char ccKingDelta[4] = {-16, -1, 1, 16};
// shi
static const char ccAdvisorDelta[4] = {-17, -15, 15, 17};
// ma
static const char ccKnightDelta[4][2] = {{-33, -31}, {-18, 14}, {-14, 18}, {31, 33}};
// ma check
static const char ccKnightCheckDelta[4][2] = {{-33, -18}, {-31, -14}, {14, 31}, {18, 33}};

static BYTE cucpcStartup_0[256] = {
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0, 20, 19, 18, 17, 16, 17, 18, 19, 20,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0, 21,  0,  0,  0,  0,  0, 21,  0,  0,  0,  0,  0,
        0,  0,  0, 22,  0, 22,  0, 22,  0, 22,  0, 22,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0, 14,  0, 14,  0, 14,  0, 14,  0, 14,  0,  0,  0,  0,
        0,  0,  0,  0, 13,  0,  0,  0,  0,  0, 13,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0, 12, 11, 10,  9,  8,  9, 10, 11, 12,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};

static BYTE cucpcStartup_1[256] = {
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0, 16,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0, 14,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0, 14,  0, 13,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0, 10,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0, 20,  0,  0, 19,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  8,  9,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};

// value of chess
static const BYTE cucvlPiecePos[7][256] = {
        { // shuai
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  1,  1,  1,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  2,  2,  2,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0, 11, 15, 11,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
        }, { // shi
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0, 20,  0, 20,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0, 23,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0, 20,  0, 20,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
        }, { // xiang
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0, 20,  0,  0,  0, 20,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0, 18,  0,  0,  0, 23,  0,  0,  0, 18,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0, 20,  0,  0,  0, 20,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
        }, { // ma
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0, 90, 90, 90, 96, 90, 96, 90, 90, 90,  0,  0,  0,  0,
                0,  0,  0, 90, 96,103, 97, 94, 97,103, 96, 90,  0,  0,  0,  0,
                0,  0,  0, 92, 98, 99,103, 99,103, 99, 98, 92,  0,  0,  0,  0,
                0,  0,  0, 93,108,100,107,100,107,100,108, 93,  0,  0,  0,  0,
                0,  0,  0, 90,100, 99,103,104,103, 99,100, 90,  0,  0,  0,  0,
                0,  0,  0, 90, 98,101,102,103,102,101, 98, 90,  0,  0,  0,  0,
                0,  0,  0, 92, 94, 98, 95, 98, 95, 98, 94, 92,  0,  0,  0,  0,
                0,  0,  0, 93, 92, 94, 95, 92, 95, 94, 92, 93,  0,  0,  0,  0,
                0,  0,  0, 85, 90, 92, 93, 78, 93, 92, 90, 85,  0,  0,  0,  0,
                0,  0,  0, 88, 85, 90, 88, 90, 88, 90, 85, 88,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
        }, { // che
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,206,208,207,213,214,213,207,208,206,  0,  0,  0,  0,
                0,  0,  0,206,212,209,216,233,216,209,212,206,  0,  0,  0,  0,
                0,  0,  0,206,208,207,214,216,214,207,208,206,  0,  0,  0,  0,
                0,  0,  0,206,213,213,216,216,216,213,213,206,  0,  0,  0,  0,
                0,  0,  0,208,211,211,214,215,214,211,211,208,  0,  0,  0,  0,
                0,  0,  0,208,212,212,214,215,214,212,212,208,  0,  0,  0,  0,
                0,  0,  0,204,209,204,212,214,212,204,209,204,  0,  0,  0,  0,
                0,  0,  0,198,208,204,212,212,212,204,208,198,  0,  0,  0,  0,
                0,  0,  0,200,208,206,212,200,212,206,208,200,  0,  0,  0,  0,
                0,  0,  0,194,206,204,212,200,212,204,206,194,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
        }, { // pao
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,100,100, 96, 91, 90, 91, 96,100,100,  0,  0,  0,  0,
                0,  0,  0, 98, 98, 96, 92, 89, 92, 96, 98, 98,  0,  0,  0,  0,
                0,  0,  0, 97, 97, 96, 91, 92, 91, 96, 97, 97,  0,  0,  0,  0,
                0,  0,  0, 96, 99, 99, 98,100, 98, 99, 99, 96,  0,  0,  0,  0,
                0,  0,  0, 96, 96, 96, 96,100, 96, 96, 96, 96,  0,  0,  0,  0,
                0,  0,  0, 95, 96, 99, 96,100, 96, 99, 96, 95,  0,  0,  0,  0,
                0,  0,  0, 96, 96, 96, 96, 96, 96, 96, 96, 96,  0,  0,  0,  0,
                0,  0,  0, 97, 96,100, 99,101, 99,100, 96, 97,  0,  0,  0,  0,
                0,  0,  0, 96, 97, 98, 98, 98, 98, 98, 97, 96,  0,  0,  0,  0,
                0,  0,  0, 96, 96, 97, 99, 99, 99, 97, 96, 96,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
        }, { // bing zu
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  9,  9,  9, 11, 13, 11,  9,  9,  9,  0,  0,  0,  0,
                0,  0,  0, 19, 24, 34, 42, 44, 42, 34, 24, 19,  0,  0,  0,  0,
                0,  0,  0, 19, 24, 32, 37, 37, 37, 32, 24, 19,  0,  0,  0,  0,
                0,  0,  0, 19, 23, 27, 29, 30, 29, 27, 23, 19,  0,  0,  0,  0,
                0,  0,  0, 14, 18, 20, 27, 29, 27, 20, 18, 14,  0,  0,  0,  0,
                0,  0,  0,  7,  0, 13,  0, 16,  0, 13,  0,  7,  0,  0,  0,  0,
                0,  0,  0,  7,  0,  7,  0, 15,  0,  7,  0,  7,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
        }
};

// check if in board
inline BOOL IN_BOARD(int sq) {
    return ccInBoard[sq] != 0;
}

// check if in fort
inline BOOL IN_FORT(int sq) {
    return ccInFort[sq] != 0;
}

// get x
inline int RANK_Y(int sq) {
    return sq /16;
//    return sq >> 4;
}

// get y
inline int FILE_X(int sq) {
    return sq % 15;
    //return sq & 15;
}

// get square
inline int COORD_XY(int x, int y) {
    return x * 16 + y;
}

// flip square
inline int SQUARE_FLIP(int sq) {
    return 254 - sq;
}

// vertical flip
inline int FILE_FLIP(int x) {
    return 14 - x;
}

// horizontal flip
inline int RANK_FLIP(int y) {
    return 15 - y;
}

//
inline int MIRROR_SQUARE(int sq) {
    return COORD_XY(FILE_FLIP(FILE_X(sq)), RANK_Y(sq));
}

//
inline int SQUARE_FORWARD(int sq, int sd) {
    return sq - 16 + (sd << 5);
}

//
inline BOOL KING_SPAN(int sqSrc, int sqDst) {
    return ccLegalSpan[sqDst - sqSrc + 256] == 1;
}

//
inline BOOL ADVISOR_SPAN(int sqSrc, int sqDst) {
    return ccLegalSpan[sqDst - sqSrc + 256] == 2;
}

//
inline BOOL BISHOP_SPAN(int sqSrc, int sqDst) {
    return ccLegalSpan[sqDst - sqSrc + 256] == 3;
}

//
inline int BISHOP_PIN(int sqSrc, int sqDst) {
    return (sqSrc + sqDst) >> 1;
}

//
inline int KNIGHT_PIN(int sqSrc, int sqDst) {
    return sqSrc + ccKnightPin[sqDst - sqSrc + 256];
}

//
inline BOOL HOME_HALF(int sq, int sd) {
    return (sq & 0x80) != (sd << 7);
}

//
inline BOOL AWAY_HALF(int sq, int sd) {
    return (sq & 0x80) == (sd << 7);
}

//
inline BOOL SAME_HALF(int sqSrc, int sqDst) {
    return ((sqSrc ^ sqDst) & 0x80) == 0;
}

// same x
inline BOOL SAME_RANK(int sqSrc, int sqDst) {
    return ((sqSrc ^ sqDst) & 0xf0) == 0;
}

// same y
inline BOOL SAME_FILE(int sqSrc, int sqDst) {
    return ((sqSrc ^ sqDst) & 0x0f) == 0;
}

// red or black
inline int SIDE_TAG(int sd) {
    return 8 + (sd << 3);
}

// opposite red or black
inline int OPP_SIDE_TAG(int sd) {
    return 16 - (sd << 3);
}

// start point
inline int SRC(int mv) {
    return mv & 255;
}

// end point
inline int DST(int mv) {
    return mv >> 8;
}

// move a step
inline int MOVE(int sqSrc, int sqDst) {
    return sqSrc + sqDst * 256;
}

// mirror move info
inline int MIRROR_MOVE(int mv) {
    return MOVE(MIRROR_SQUARE(SRC(mv)), MIRROR_SQUARE(DST(mv)));
}

// get the square index of chess
inline int GetSquare(int x, int y) {
    return ((x+3) * 16 + y + 3);
}

// get the class index of chess
inline BYTE GetClass(string class_) {
    vector<BYTE> map_list = {16, 20, 19, 21, 17, 18, 22, 8, 12, 11, 13, 9, 10, 14};
    int map_index = atoi(class_.c_str());
    return map_list[map_index];
}


std::vector<std::string> split(std::string str, std::string pattern)
{
    std::string::size_type pos;
    std::vector<std::string> result;
    str += pattern;
    int size = str.size();
    for (int i = 0; i < size; i++)
    {
        pos = str.find(pattern, i);
        if (pos < size)
        {
            std::string s = str.substr(i, pos - i);
            result.push_back(s);
            i = pos + pattern.size() - 1;
        }
    }
    return result;
}


// RC4 password stream generator
struct RC4Struct {
    BYTE s[256];
    int x, y;

    void InitZero(void);   // init with empty pwd
    BYTE NextByte(void) {  // generate next byte
        BYTE uc;
        x = (x + 1) & 255;
        y = (y + s[x]) & 255;
        uc = s[x];
        s[x] = s[y];
        s[y] = uc;
        return s[(s[x] + s[y]) & 255];
    }
    DWORD NextLong(void) { // next 4 byte
        BYTE uc0, uc1, uc2, uc3;
        uc0 = NextByte();
        uc1 = NextByte();
        uc2 = NextByte();
        uc3 = NextByte();
        return uc0 + (uc1 << 8) + (uc2 << 16) + (uc3 << 24);
    }
};

// init
void RC4Struct::InitZero(void) {
    int i, j;
    BYTE uc;

    x = y = j = 0;
    for (i = 0; i < 256; i ++) {
        s[i] = i;
    }
    for (i = 0; i < 256; i ++) {
        j = (j + s[i]) & 255;
        uc = s[i];
        s[i] = s[j];
        s[j] = uc;
    }
}

// Zobrist
struct ZobristStruct {
    DWORD dwKey, dwLock0, dwLock1;

    void InitZero(void) {                 // fill in zobrist with 0
        dwKey = dwLock0 = dwLock1 = 0;
    }
    void InitRC4(RC4Struct &rc4) {        // fill in Zobrist with rc4
        dwKey = rc4.NextLong();
        dwLock0 = rc4.NextLong();
        dwLock1 = rc4.NextLong();
    }
    void Xor(const ZobristStruct &zobr) { // XOR
        dwKey ^= zobr.dwKey;
        dwLock0 ^= zobr.dwLock0;
        dwLock1 ^= zobr.dwLock1;
    }
    void Xor(const ZobristStruct &zobr1, const ZobristStruct &zobr2) {
        dwKey ^= zobr1.dwKey ^ zobr2.dwKey;
        dwLock0 ^= zobr1.dwLock0 ^ zobr2.dwLock0;
        dwLock1 ^= zobr1.dwLock1 ^ zobr2.dwLock1;
    }
};

// Zobrist table
static struct {
    ZobristStruct Player;
    ZobristStruct Table[14][256];
} Zobrist;

// Zobrist table init
static void InitZobrist(void) {
    int i, j;
    RC4Struct rc4;

    rc4.InitZero();
    Zobrist.Player.InitRC4(rc4);
    for (i = 0; i < 14; i ++) {
        for (j = 0; j < 256; j ++) {
            Zobrist.Table[i][j].InitRC4(rc4);
        }
    }
}

// history move info (4 byte)
struct MoveStruct {
    WORD wmv;
    BYTE ucpcCaptured, ucbCheck;
    DWORD dwKey;

    void Set(int mv, int pcCaptured, BOOL bCheck, DWORD dwKey_) {
        wmv = mv;
        ucpcCaptured = pcCaptured;
        ucbCheck = bCheck;
        dwKey = dwKey_;
    }
}; // mvs

// current chessboard info
struct PositionStruct {
    int sdPlayer;                   // whose turn
    BYTE ucpcSquares[256];          // pc on chessboard
    int vlWhite, vlBlack;           // pc values of 2 sides
    int nDistance, nMoveNum;        // steps from root, history steps number
    MoveStruct mvsList[MAX_MOVES];  // history steps info
    ZobristStruct zobr;             // Zobrist
    int AI_mv;                      // AI mv record

    // read in chessboard info according to the input string
    void ReadInChess(string buff, BYTE * ucpcSquares, int size) {
        *ucpcSquares = { };

        vector<string> res;
        vector< vector<string> > ans;
        res = split(buff, "#");

        for (int i=0; i < res.size(); i++){
            string tmp;
            for (int j=1; j< res[i].size()-1;j++) {
                tmp += res[i][j];
            }
            vector <string> tmp_2;
            tmp_2 = split(tmp, ", ");
            ans.push_back(tmp_2);

        }

        for (int i=0; i<ans.size();i++) {
            int x = atoi(ans[i][0].c_str());
            int y = atoi(ans[i][1].c_str());
            int new_index = GetSquare(x, y);
            BYTE new_class = GetClass(ans[i][2]);
            ucpcSquares[new_index] = new_class;
        }
    }


    void PrintBoard(void) {         // print chess board.
        //system("cls");
        for (int i=0; i<16; i++){
            for(int j=0;j<16;j++){
                cout.width(8);
                switch (ucpcSquares[ (16*i + j)]-0) {
                    case 8 : cout << "r_shuai"; break;
                    case 9 : cout << " r_shi "; break;
                    case 10 : cout << "r_xiang"; break;
                    case 11 : cout << " r_ma  "; break;
                    case 12 : cout << " r_ju  "; break;
                    case 13 : cout << " r_pao "; break;
                    case 14 : cout << " r_bing"; break;
                    case 16 : cout << "b_jiang"; break;
                    case 17 : cout << " b_shi "; break;
                    case 18 : cout << "b_xiang"; break;
                    case 19 : cout << " b_ma  "; break;
                    case 20 : cout << " b_ju  "; break;
                    case 21 : cout << " b_pao "; break;
                    case 22 : cout << " b_zu  "; break;
                    default : cout << "       ";
                }
            }
            cout << endl;
        }
    }

    void ClearBoard(void) {         // clear board
        sdPlayer = vlWhite = vlBlack = nDistance = 0;
        memset(ucpcSquares, 0, 256);
        zobr.InitZero();
    }
    void SetIrrev(void) {           // init history step info
        mvsList[0].Set(0, 0, Checked(), zobr.dwKey);
        nMoveNum = 1;
    }
    void Startup(int idx);             // chessboard init
//  void Startup_halfway(void);
    void ChangeSide(void) {         // change side
        sdPlayer = 1 - sdPlayer;
        zobr.Xor(Zobrist.Player);
    }
    void AddPiece(int sq, int pc) { // put a pc on chessboard
        ucpcSquares[sq] = pc;
        // red value +, black value -   (Attention: check if cucvlPiecePos)
        if (pc < 16) {
            vlWhite += cucvlPiecePos[pc - 8][sq];
            zobr.Xor(Zobrist.Table[pc - 8][sq]);
        } else {
            vlBlack += cucvlPiecePos[pc - 16][SQUARE_FLIP(sq)];
            zobr.Xor(Zobrist.Table[pc - 9][sq]);
        }
    }
    void DelPiece(int sq, int pc) { // remove a pc on chessboard
        ucpcSquares[sq] = 0;
        // red value -, black value +   (Attention: check if cucvlPiecePos)
        if (pc < 16) {
            vlWhite -= cucvlPiecePos[pc - 8][sq];
            zobr.Xor(Zobrist.Table[pc - 8][sq]);
        } else {
            vlBlack -= cucvlPiecePos[pc - 16][SQUARE_FLIP(sq)];
            zobr.Xor(Zobrist.Table[pc - 9][sq]);
        }
    }
    int Evaluate(void) const {      // evaluate value
        return (sdPlayer == 0 ? vlWhite - vlBlack : vlBlack - vlWhite) + ADVANCED_VALUE;
    }
    BOOL InCheck(void) const {      // if is checked.
        return mvsList[nMoveNum - 1].ucbCheck;
    }
    BOOL Captured(void) const {     // if a pc is killed in last step.
        return mvsList[nMoveNum - 1].ucpcCaptured != 0;
    }
    int MovePiece(int mv);                      // move a pc
    void UndoMovePiece(int mv, int pcCaptured); // undo a move
    BOOL MakeMove(int mv);                      // make a move
    void UndoMakeMove(void) {                   // undo make a move
        nDistance --;
        nMoveNum --;
        ChangeSide();
        UndoMovePiece(mvsList[nMoveNum].wmv, mvsList[nMoveNum].ucpcCaptured);
    }
    void NullMove(void) {                       // empty move
        DWORD dwKey;
        dwKey = zobr.dwKey;
        ChangeSide();
        mvsList[nMoveNum].Set(0, 0, FALSE, dwKey);
        nMoveNum ++;
        nDistance ++;
    }
    void UndoNullMove(void) {                   // undo an empty move
        nDistance --;
        nMoveNum --;
        ChangeSide();
    }
    // Genarate all moves, if bCapture is True, generate kill move only.
    int GenerateMoves(int *mvs, BOOL bCapture = FALSE) const;
    BOOL LegalMove(int mv) const;               // check if move is legal
    BOOL Checked(void) const;                   // if is checked
    BOOL IsMate(void);                          // if is gg
    int DrawValue(void) const {                 // draw value
        return (nDistance & 1) == 0 ? -DRAW_VALUE : DRAW_VALUE;
    }
    int RepStatus(int nRecur = 1) const;        // check repeat status
    int RepValue(int nRepStatus) const {        // repeat status values
        int vlReturn;
        vlReturn = ((nRepStatus & 2) == 0 ? 0 : nDistance - BAN_VALUE) +
                   ((nRepStatus & 4) == 0 ? 0 : BAN_VALUE - nDistance);
        return vlReturn == 0 ? DrawValue() : vlReturn;
    }
    BOOL NullOkay(void) const {                 // if null cut is allowed
        return (sdPlayer == 0 ? vlWhite : vlBlack) > NULL_MARGIN;
    }
    void Mirror(PositionStruct &posMirror) const; //
};

// initialization
void PositionStruct::Startup(int idx) {
    int sq, pc;
    ClearBoard();
    for (sq = 0; sq < 256; sq ++) {
        // different start status, default to zero
        switch (idx) {
            case 0 :
                pc = cucpcStartup_0[sq];
                break;
            case 1 :
                pc = cucpcStartup_1[sq];
                break;
            default :
                pc = cucpcStartup_0[sq];
                break;
        }

        if (pc != 0) {
            AddPiece(sq, pc);
        }
    }
    SetIrrev();
}


// move a pc on chessboard
int PositionStruct::MovePiece(int mv) {
    int sqSrc, sqDst, pc, pcCaptured;
    sqSrc = SRC(mv);
    sqDst = DST(mv);
    pcCaptured = ucpcSquares[sqDst];
    if (pcCaptured != 0) {
        DelPiece(sqDst, pcCaptured);
    }
    pc = ucpcSquares[sqSrc];
    DelPiece(sqSrc, pc);
    AddPiece(sqDst, pc);
    return pcCaptured;
}

// undo a move
void PositionStruct::UndoMovePiece(int mv, int pcCaptured) {
    int sqSrc, sqDst, pc;
    sqSrc = SRC(mv);
    sqDst = DST(mv);
    pc = ucpcSquares[sqDst];
    DelPiece(sqDst, pc);
    AddPiece(sqSrc, pc);
    if (pcCaptured != 0) {
        AddPiece(sqDst, pcCaptured);
    }
}

// make move
BOOL PositionStruct::MakeMove(int mv) {
    int pcCaptured;
    DWORD dwKey;

    dwKey = zobr.dwKey;
    pcCaptured = MovePiece(mv);
    if (Checked()) {
        UndoMovePiece(mv, pcCaptured);
        return FALSE;
    }
    ChangeSide();
    mvsList[nMoveNum].Set(mv, pcCaptured, Checked(), dwKey);
    nMoveNum ++;
    nDistance ++;
    return TRUE;
}

// GenerateMoves
const BOOL GEN_CAPTURE = TRUE;

//
int PositionStruct::GenerateMoves(int *mvs, BOOL bCapture) const {
    int i, j, nGenMoves, nDelta, sqSrc, sqDst;
    int pcSelfSide, pcOppSide, pcSrc, pcDst;
    // generating all move needs following steps

    nGenMoves = 0;
    pcSelfSide = SIDE_TAG(sdPlayer);
    pcOppSide = OPP_SIDE_TAG(sdPlayer);
    for (sqSrc = 0; sqSrc < 256; sqSrc ++) {

        // 1. find one of pc of my side.
        pcSrc = ucpcSquares[sqSrc];
        if ((pcSrc & pcSelfSide) == 0) {
            continue;
        }

        // 2. decide move steps according to class of pc
        switch (pcSrc - pcSelfSide) {
            case PIECE_KING:
                for (i = 0; i < 4; i ++) {
                    sqDst = sqSrc + ccKingDelta[i];
                    if (!IN_FORT(sqDst)) {
                        continue;
                    }
                    pcDst = ucpcSquares[sqDst];
                    if (bCapture ? (pcDst & pcOppSide) != 0 : (pcDst & pcSelfSide) == 0) {
                        mvs[nGenMoves] = MOVE(sqSrc, sqDst);
                        nGenMoves ++;
                    }
                }
                break;
            case PIECE_ADVISOR:
                for (i = 0; i < 4; i ++) {
                    sqDst = sqSrc + ccAdvisorDelta[i];
                    if (!IN_FORT(sqDst)) {
                        continue;
                    }
                    pcDst = ucpcSquares[sqDst];
                    if (bCapture ? (pcDst & pcOppSide) != 0 : (pcDst & pcSelfSide) == 0) {
                        mvs[nGenMoves] = MOVE(sqSrc, sqDst);
                        nGenMoves ++;
                    }
                }
                break;
            case PIECE_BISHOP:
                for (i = 0; i < 4; i ++) {
                    sqDst = sqSrc + ccAdvisorDelta[i];
                    if (!(IN_BOARD(sqDst) && HOME_HALF(sqDst, sdPlayer) && ucpcSquares[sqDst] == 0)) {
                        continue;
                    }
                    sqDst += ccAdvisorDelta[i];
                    pcDst = ucpcSquares[sqDst];
                    if (bCapture ? (pcDst & pcOppSide) != 0 : (pcDst & pcSelfSide) == 0) {
                        mvs[nGenMoves] = MOVE(sqSrc, sqDst);
                        nGenMoves ++;
                    }
                }
                break;
            case PIECE_KNIGHT:
                for (i = 0; i < 4; i ++) {
                    sqDst = sqSrc + ccKingDelta[i];
                    if (ucpcSquares[sqDst] != 0) {
                        continue;
                    }
                    for (j = 0; j < 2; j ++) {
                        sqDst = sqSrc + ccKnightDelta[i][j];
                        if (!IN_BOARD(sqDst)) {
                            continue;
                        }
                        pcDst = ucpcSquares[sqDst];
                        if (bCapture ? (pcDst & pcOppSide) != 0 : (pcDst & pcSelfSide) == 0) {
                            mvs[nGenMoves] = MOVE(sqSrc, sqDst);
                            nGenMoves ++;
                        }
                    }
                }
                break;
            case PIECE_ROOK:
                for (i = 0; i < 4; i ++) {
                    nDelta = ccKingDelta[i];
                    sqDst = sqSrc + nDelta;
                    while (IN_BOARD(sqDst)) {
                        pcDst = ucpcSquares[sqDst];
                        if (pcDst == 0) {
                            if (!bCapture) {
                                mvs[nGenMoves] = MOVE(sqSrc, sqDst);
                                nGenMoves ++;
                            }
                        } else {
                            if ((pcDst & pcOppSide) != 0) {
                                mvs[nGenMoves] = MOVE(sqSrc, sqDst);
                                nGenMoves ++;
                            }
                            break;
                        }
                        sqDst += nDelta;
                    }
                }
                break;
            case PIECE_CANNON:
                for (i = 0; i < 4; i ++) {
                    nDelta = ccKingDelta[i];
                    sqDst = sqSrc + nDelta;
                    while (IN_BOARD(sqDst)) {
                        pcDst = ucpcSquares[sqDst];
                        if (pcDst == 0) {
                            if (!bCapture) {
                                mvs[nGenMoves] = MOVE(sqSrc, sqDst);
                                nGenMoves ++;
                            }
                        } else {
                            break;
                        }
                        sqDst += nDelta;
                    }
                    sqDst += nDelta;
                    while (IN_BOARD(sqDst)) {
                        pcDst = ucpcSquares[sqDst];
                        if (pcDst != 0) {
                            if ((pcDst & pcOppSide) != 0) {
                                mvs[nGenMoves] = MOVE(sqSrc, sqDst);
                                nGenMoves ++;
                            }
                            break;
                        }
                        sqDst += nDelta;
                    }
                }
                break;
            case PIECE_PAWN:
                sqDst = SQUARE_FORWARD(sqSrc, sdPlayer);
                if (IN_BOARD(sqDst)) {
                    pcDst = ucpcSquares[sqDst];
                    if (bCapture ? (pcDst & pcOppSide) != 0 : (pcDst & pcSelfSide) == 0) {
                        mvs[nGenMoves] = MOVE(sqSrc, sqDst);
                        nGenMoves ++;
                    }
                }
                if (AWAY_HALF(sqSrc, sdPlayer)) {
                    for (nDelta = -1; nDelta <= 1; nDelta += 2) {
                        sqDst = sqSrc + nDelta;
                        if (IN_BOARD(sqDst)) {
                            pcDst = ucpcSquares[sqDst];
                            if (bCapture ? (pcDst & pcOppSide) != 0 : (pcDst & pcSelfSide) == 0) {
                                mvs[nGenMoves] = MOVE(sqSrc, sqDst);
                                nGenMoves ++;
                            }
                        }
                    }
                }
                break;
        }
    }
    return nGenMoves;
}


// check if the move is legal.
BOOL PositionStruct::LegalMove(int mv) const {
    int sqSrc, sqDst, sqPin;
    int pcSelfSide, pcSrc, pcDst, nDelta;
    // takes the following steps:

    // 1. if there is my chess at the start point
    sqSrc = SRC(mv);
    pcSrc = ucpcSquares[sqSrc];
    pcSelfSide = SIDE_TAG(sdPlayer);
    if ((pcSrc & pcSelfSide) == 0) {

        return FALSE;
    }

    // 2. if there is my chess at the end point
    sqDst = DST(mv);
    pcDst = ucpcSquares[sqDst];
    if ((pcDst & pcSelfSide) != 0) {
        return FALSE;
    }

    // 3. check rules
    switch (pcSrc - pcSelfSide) {
        case PIECE_KING:
            return IN_FORT(sqDst) && KING_SPAN(sqSrc, sqDst);
        case PIECE_ADVISOR:
            return IN_FORT(sqDst) && ADVISOR_SPAN(sqSrc, sqDst);
        case PIECE_BISHOP:
            return SAME_HALF(sqSrc, sqDst) && BISHOP_SPAN(sqSrc, sqDst) &&
                   ucpcSquares[BISHOP_PIN(sqSrc, sqDst)] == 0;
        case PIECE_KNIGHT:
            sqPin = KNIGHT_PIN(sqSrc, sqDst);
            return sqPin != sqSrc && ucpcSquares[sqPin] == 0;
        case PIECE_ROOK:
        case PIECE_CANNON:
            if (SAME_RANK(sqSrc, sqDst)) {
                nDelta = (sqDst < sqSrc ? -1 : 1);
            } else if (SAME_FILE(sqSrc, sqDst)) {
                nDelta = (sqDst < sqSrc ? -16 : 16);
            } else {
                return FALSE;
            }
            sqPin = sqSrc + nDelta;
            while (sqPin != sqDst && ucpcSquares[sqPin] == 0) {
                sqPin += nDelta;
            }
            if (sqPin == sqDst) {
                return pcDst == 0 || pcSrc - pcSelfSide == PIECE_ROOK;
            } else if (pcDst != 0 && pcSrc - pcSelfSide == PIECE_CANNON) {
                sqPin += nDelta;
                while (sqPin != sqDst && ucpcSquares[sqPin] == 0) {
                    sqPin += nDelta;
                }
                return sqPin == sqDst;
            } else {
                return FALSE;
            }
        case PIECE_PAWN:
            if (AWAY_HALF(sqDst, sdPlayer) && (sqDst == sqSrc - 1 || sqDst == sqSrc + 1)) {
                return TRUE;
            }
            return sqDst == SQUARE_FORWARD(sqSrc, sdPlayer);
        default:
            return FALSE;
    }
}

// if is checked
BOOL PositionStruct::Checked() const {
    int i, j, sqSrc, sqDst;
    int pcSelfSide, pcOppSide, pcDst, nDelta;
    pcSelfSide = SIDE_TAG(sdPlayer);
    pcOppSide = OPP_SIDE_TAG(sdPlayer);
    // find king on the chessboard

    for (sqSrc = 0; sqSrc < 256; sqSrc ++) {
        if (ucpcSquares[sqSrc] != pcSelfSide + PIECE_KING) {
            continue;
        }

        // 1. if check by pawn
        if (ucpcSquares[SQUARE_FORWARD(sqSrc, sdPlayer)] == pcOppSide + PIECE_PAWN) {
            return TRUE;
        }
        for (nDelta = -1; nDelta <= 1; nDelta += 2) {
            if (ucpcSquares[sqSrc + nDelta] == pcOppSide + PIECE_PAWN) {
                return TRUE;
            }
        }

        // 2. if checked by knight,
        for (i = 0; i < 4; i ++) {
            if (ucpcSquares[sqSrc + ccAdvisorDelta[i]] != 0) {
                continue;
            }
            for (j = 0; j < 2; j ++) {
                pcDst = ucpcSquares[sqSrc + ccKnightCheckDelta[i][j]];
                if (pcDst == pcOppSide + PIECE_KNIGHT) {
                    return TRUE;
                }
            }
        }

        // 3. if is checked by rook or cannon
        for (i = 0; i < 4; i ++) {
            nDelta = ccKingDelta[i];
            sqDst = sqSrc + nDelta;
            while (IN_BOARD(sqDst)) {
                pcDst = ucpcSquares[sqDst];
                if (pcDst != 0) {
                    if (pcDst == pcOppSide + PIECE_ROOK || pcDst == pcOppSide + PIECE_KING) {
                        return TRUE;
                    }
                    break;
                }
                sqDst += nDelta;
            }
            sqDst += nDelta;
            while (IN_BOARD(sqDst)) {
                int pcDst = ucpcSquares[sqDst];
                if (pcDst != 0) {
                    if (pcDst == pcOppSide + PIECE_CANNON) {
                        return TRUE;
                    }
                    break;
                }
                sqDst += nDelta;
            }
        }
        return FALSE;
    }
    return FALSE;
}

// check if game is over
BOOL PositionStruct::IsMate(void) {
    int i, nGenMoveNum, pcCaptured;
    int mvs[MAX_GEN_MOVES];

    nGenMoveNum = GenerateMoves(mvs);
    for (i = 0; i < nGenMoveNum; i ++) {
        pcCaptured = MovePiece(mvs[i]);
        if (!Checked()) {
            UndoMovePiece(mvs[i], pcCaptured);
            return FALSE;
        } else {
            UndoMovePiece(mvs[i], pcCaptured);
        }
    }
    return TRUE;
}

// repeated status
int PositionStruct::RepStatus(int nRecur) const {
    BOOL bSelfSide, bPerpCheck, bOppPerpCheck;
    const MoveStruct *lpmvs;

    bSelfSide = FALSE;
    bPerpCheck = bOppPerpCheck = TRUE;
    lpmvs = mvsList + nMoveNum - 1;
    while (lpmvs->wmv != 0 && lpmvs->ucpcCaptured == 0) {
        if (bSelfSide) {
            bPerpCheck = bPerpCheck && lpmvs->ucbCheck;
            if (lpmvs->dwKey == zobr.dwKey) {
                nRecur --;
                if (nRecur == 0) {
                    return 1 + (bPerpCheck ? 2 : 0) + (bOppPerpCheck ? 4 : 0);
                }
            }
        } else {
            bOppPerpCheck = bOppPerpCheck && lpmvs->ucbCheck;
        }
        bSelfSide = !bSelfSide;
        lpmvs --;
    }
    return 0;
}

// mirror of status
void PositionStruct::Mirror(PositionStruct &posMirror) const {
    int sq, pc;
    posMirror.ClearBoard();
    for (sq = 0; sq < 256; sq ++) {
        pc = ucpcSquares[sq];
        if (pc != 0) {
            posMirror.AddPiece(MIRROR_SQUARE(sq), pc);
        }
    }
    if (sdPlayer == 1) {
        posMirror.ChangeSide();
    }
    posMirror.SetIrrev();
}

static PositionStruct pos; // create instance

// global variance regard to current status
static struct {
    int sqSelected, mvLast;
    BOOL bFlipped, bGameOver, bGameDraw;
} Xqwl;

// hash table
struct HashItem {
    BYTE ucDepth, ucFlag;
    short svl;
    WORD wmv, wReserved;
    DWORD dwLock0, dwLock1;
};

// open book
struct BookItem {
    DWORD dwLock;
    WORD wmv, wvl;
};

// global variance relevant to searching
static struct {
    int mvResult;                  // search mv result
    int nHistoryTable[65536];      // history move table
    int mvKillers[LIMIT_DEPTH][2]; // killer table
    HashItem HashTable[HASH_SIZE]; // hash table
    int nBookSize;                 // open book size
    BookItem BookTable[BOOK_SIZE]; // open lib
} Search;


static int CompareBook(const void *lpbk1, const void *lpbk2) {
    DWORD dw1, dw2;
    dw1 = ((BookItem *) lpbk1)->dwLock;
    dw2 = ((BookItem *) lpbk2)->dwLock;
    return dw1 > dw2 ? 1 : dw1 < dw2 ? -1 : 0;
}

// search open book
static int SearchBook(void) {
    int i, vl, nBookMoves, mv;
    int mvs[MAX_GEN_MOVES], vls[MAX_GEN_MOVES];
    BOOL bMirror;
    BookItem bkToSearch, *lpbk;
    PositionStruct posMirror;
    // following steps to search open lib

    // 1. if there is no open lib, return immediately.
    if (Search.nBookSize == 0) {
        return 0;
    }
    // 2. search current status
    bMirror = FALSE;
    bkToSearch.dwLock = pos.zobr.dwLock1;
    lpbk = (BookItem *) bsearch(&bkToSearch, Search.BookTable, Search.nBookSize, sizeof(BookItem), CompareBook);
    // 3. search mirror status
    if (lpbk == NULL) {
        bMirror = TRUE;
        pos.Mirror(posMirror);
        bkToSearch.dwLock = posMirror.zobr.dwLock1;
        lpbk = (BookItem *) bsearch(&bkToSearch, Search.BookTable, Search.nBookSize, sizeof(BookItem), CompareBook);
    }
    // 4. return if not found
    if (lpbk == NULL) {
        return 0;
    }
    // 5. if found search a previous open lib item
    while (lpbk >= Search.BookTable && lpbk->dwLock == bkToSearch.dwLock) {
        lpbk --;
    }
    lpbk ++;
    // 6. fill mv and value
    vl = nBookMoves = 0;
    while (lpbk < Search.BookTable + Search.nBookSize && lpbk->dwLock == bkToSearch.dwLock) {
        mv = (bMirror ? MIRROR_MOVE(lpbk->wmv) : lpbk->wmv);
        if (pos.LegalMove(mv)) {
            mvs[nBookMoves] = mv;
            vls[nBookMoves] = lpbk->wvl;
            vl += vls[nBookMoves];
            nBookMoves ++;
            if (nBookMoves == MAX_GEN_MOVES) {
                break; // in case there are abnormal data in open lib
            }
        }
        lpbk ++;
    }
    if (vl == 0) {
        return 0; // in case there are abnormal data in open lib
    }
    // 7. randomly pick a move according to weights
    vl = rand() % vl;
    for (i = 0; i < nBookMoves; i ++) {
        vl -= vls[i];
        if (vl < 0) {
            break;
        }
    }
    return mvs[i];
}

// probe hash
static int ProbeHash(int vlAlpha, int vlBeta, int nDepth, int &mv) {
    BOOL bMate; // killer flag. if true, depth condition is not required.
    HashItem hsh;

    hsh = Search.HashTable[pos.zobr.dwKey & (HASH_SIZE - 1)];
    if (hsh.dwLock0 != pos.zobr.dwLock0 || hsh.dwLock1 != pos.zobr.dwLock1) {
        mv = 0;
        return -MATE_VALUE;
    }
    mv = hsh.wmv;
    bMate = FALSE;
    if (hsh.svl > WIN_VALUE) {
        if (hsh.svl < BAN_VALUE) {
            return -MATE_VALUE; // might cause the instability of search, but maybe find best move
        }
        hsh.svl -= pos.nDistance;
        bMate = TRUE;
    } else if (hsh.svl < -WIN_VALUE) {
        if (hsh.svl > -BAN_VALUE) {
            return -MATE_VALUE; // upward
        }
        hsh.svl += pos.nDistance;
        bMate = TRUE;
    }
    if (hsh.ucDepth >= nDepth || bMate) {
        if (hsh.ucFlag == HASH_BETA) {
            return (hsh.svl >= vlBeta ? hsh.svl : -MATE_VALUE);
        } else if (hsh.ucFlag == HASH_ALPHA) {
            return (hsh.svl <= vlAlpha ? hsh.svl : -MATE_VALUE);
        }
        return hsh.svl;
    }
    return -MATE_VALUE;
};

// save hash item
static void RecordHash(int nFlag, int vl, int nDepth, int mv) {
    HashItem hsh;
    hsh = Search.HashTable[pos.zobr.dwKey & (HASH_SIZE - 1)];
    if (hsh.ucDepth > nDepth) {
        return;
    }
    hsh.ucFlag = nFlag;
    hsh.ucDepth = nDepth;
    if (vl > WIN_VALUE) {
        if (mv == 0 && vl <= BAN_VALUE) {
            return; // might cause the instability of search, and best move cant be found.
        }
        hsh.svl = vl + pos.nDistance;
    } else if (vl < -WIN_VALUE) {
        if (mv == 0 && vl >= -BAN_VALUE) {
            return;
        }
        hsh.svl = vl - pos.nDistance;
    } else {
        hsh.svl = vl;
    }
    hsh.wmv = mv;
    hsh.dwLock0 = pos.zobr.dwLock0;
    hsh.dwLock1 = pos.zobr.dwLock1;
    Search.HashTable[pos.zobr.dwKey & (HASH_SIZE - 1)] = hsh;
};

// value of different class of pc
static BYTE cucMvvLva[24] = {
        0, 0, 0, 0, 0, 0, 0, 0,
        5, 1, 1, 3, 4, 3, 2, 0,
        5, 1, 1, 3, 4, 3, 2, 0
};

// find value of different class of pc
inline int MvvLva(int mv) {
    return (cucMvvLva[pos.ucpcSquares[DST(mv)]] << 3) - cucMvvLva[pos.ucpcSquares[SRC(mv)]];
}

// sort value according to MVV/LVA
static int CompareMvvLva(const void *lpmv1, const void *lpmv2) {
    return MvvLva(*(int *) lpmv2) - MvvLva(*(int *) lpmv1);
}

// sort value according to history move table
static int CompareHistory(const void *lpmv1, const void *lpmv2) {
    return Search.nHistoryTable[*(int *) lpmv2] - Search.nHistoryTable[*(int *) lpmv1];
}


// move sort
const int PHASE_HASH = 0;
const int PHASE_KILLER_1 = 1;
const int PHASE_KILLER_2 = 2;
const int PHASE_GEN_MOVES = 3;
const int PHASE_REST = 4;

// move sort struct
struct SortStruct {
    int mvHash, mvKiller1, mvKiller2; // hash table move and 2 killer move
    int nPhase, nIndex, nGenMoves;    // current status. No.? mv is applied, there are ? mv in total
    int mvs[MAX_GEN_MOVES];           // all moves

    void Init(int mvHash_) { // init, design hash mv and 2 killer mv table.
        mvHash = mvHash_;
        mvKiller1 = Search.mvKillers[pos.nDistance][0];
        mvKiller2 = Search.mvKillers[pos.nDistance][1];
        nPhase = PHASE_HASH;
    }
    int Next(void); // generate next mv.
};

// get next mv.
int SortStruct::Next(void) {
    int mv;
    switch (nPhase) {

        case PHASE_HASH:
            nPhase = PHASE_KILLER_1;
            if (mvHash != 0) {
                return mvHash;
            }

        case PHASE_KILLER_1:
            nPhase = PHASE_KILLER_2;
            if (mvKiller1 != mvHash && mvKiller1 != 0 && pos.LegalMove(mvKiller1)) {
                return mvKiller1;
            }

        case PHASE_KILLER_2:
            nPhase = PHASE_GEN_MOVES;
            if (mvKiller2 != mvHash && mvKiller2 != 0 && pos.LegalMove(mvKiller2)) {
                return mvKiller2;
            }

        case PHASE_GEN_MOVES:
            nPhase = PHASE_REST;
            nGenMoves = pos.GenerateMoves(mvs);
            qsort(mvs, nGenMoves, sizeof(int), CompareHistory);
            nIndex = 0;

        case PHASE_REST:
            while (nIndex < nGenMoves) {
                mv = mvs[nIndex];
                nIndex ++;
                if (mv != mvHash && mv != mvKiller1 && mv != mvKiller2) {
                    return mv;
                }
            }

        default:
            return 0;
    }
}

// handle the best move
inline void SetBestMove(int mv, int nDepth) {
    int *lpmvKillers;
    Search.nHistoryTable[mv] += nDepth * nDepth;
    lpmvKillers = Search.mvKillers[pos.nDistance];
    if (lpmvKillers[0] != mv) {
        lpmvKillers[1] = lpmvKillers[0];
        lpmvKillers[0] = mv;
    }
}

// Quiescence search
static int SearchQuiesc(int vlAlpha, int vlBeta) {
    int i, nGenMoves;
    int vl, vlBest;
    int mvs[MAX_GEN_MOVES];

    // 1. check repeated status
    vl = pos.RepStatus();
    if (vl != 0) {
        return pos.RepValue(vl);
    }

    // 2. if reach the depth limit, return status evaluation
    if (pos.nDistance == LIMIT_DEPTH) {
        return pos.Evaluate();
    }

    // 3. init best value
    vlBest = -MATE_VALUE;

    if (pos.InCheck()) {
        // 4. if is checked, generate all possible moves
        nGenMoves = pos.GenerateMoves(mvs);
        qsort(mvs, nGenMoves, sizeof(int), CompareHistory);
    } else {

        // 5. if not, evaluate status first
        vl = pos.Evaluate();
        if (vl > vlBest) {
            vlBest = vl;
            if (vl >= vlBeta) {
                return vl;
            }
            if (vl > vlAlpha) {
                vlAlpha = vl;
            }
        }

        // 6. if not cut, generate eating moves
        nGenMoves = pos.GenerateMoves(mvs, GEN_CAPTURE);
        qsort(mvs, nGenMoves, sizeof(int), CompareMvvLva);
    }

    // 7. try each move
    for (i = 0; i < nGenMoves; i ++) {
        if (pos.MakeMove(mvs[i])) {
            vl = -SearchQuiesc(-vlBeta, -vlAlpha);
            pos.UndoMakeMove();

            // 8. Alpha-Beta
            if (vl > vlBest) {
                vlBest = vl;
                if (vl >= vlBeta) {
                    return vl;
                }
                if (vl > vlAlpha) {
                    vlAlpha = vl;
                }
            }
        }
    }

    // 9. when all the moves are tried, return the best
    return vlBest == -MATE_VALUE ? pos.nDistance - MATE_VALUE : vlBest;
}

const BOOL NO_NULL = TRUE;

// fail-soft alpha-beta
static int SearchFull(int vlAlpha, int vlBeta, int nDepth, BOOL bNoNull = FALSE) {
    int nHashFlag, vl, vlBest;
    int mv, mvBest, mvHash, nNewDepth;
    SortStruct Sort;

    // 1. quiescence search
    if (nDepth <= 0) {
        return SearchQuiesc(vlAlpha, vlBeta);
    }

    // 1-1. check repeated status
    vl = pos.RepStatus();
    if (vl != 0) {
        return pos.RepValue(vl);
    }

    // 1-2. return status evaluation if reach depth limit
    if (pos.nDistance == LIMIT_DEPTH) {
        return pos.Evaluate();
    }

    // 1-3. try hash table cut
    vl = ProbeHash(vlAlpha, vlBeta, nDepth, mvHash);
    if (vl > -MATE_VALUE) {
        return vl;
    }

    // 1-4. try null step cut
    if (!bNoNull && !pos.InCheck() && pos.NullOkay()) {
        pos.NullMove();
        vl = -SearchFull(-vlBeta, 1 - vlBeta, nDepth - NULL_DEPTH - 1, NO_NULL);
        pos.UndoNullMove();
        if (vl >= vlBeta) {
            return vl;
        }
    }

    // 2. init
    nHashFlag = HASH_ALPHA;
    vlBest = -MATE_VALUE;
    mvBest = 0;

    // 3. init move sort
    Sort.Init(mvHash);

    // 4. try these moves
    while ((mv = Sort.Next()) != 0) {
        if (pos.MakeMove(mv)) {

            nNewDepth = pos.InCheck() ? nDepth : nDepth - 1;
            // PVS
            if (vlBest == -MATE_VALUE) {
                vl = -SearchFull(-vlBeta, -vlAlpha, nNewDepth);
            } else {
                vl = -SearchFull(-vlAlpha - 1, -vlAlpha, nNewDepth);
                if (vl > vlAlpha && vl < vlBeta) {
                    vl = -SearchFull(-vlBeta, -vlAlpha, nNewDepth);
                }
            }
            pos.UndoMakeMove();

            // 5. Alpha-Beta
            if (vl > vlBest) {
                vlBest = vl;
                if (vl >= vlBeta) {
                    nHashFlag = HASH_BETA;
                    mvBest = mv;
                    break;
                }
                if (vl > vlAlpha) {
                    nHashFlag = HASH_PV;
                    mvBest = mv;
                    vlAlpha = vl;
                }
            }
        }
    }

    // 5. all the moves are searched, add best move to history table, return best value
    if (vlBest == -MATE_VALUE) {

        return pos.nDistance - MATE_VALUE;
    }

    RecordHash(nHashFlag, vlBest, nDepth, mvBest);
    if (mvBest != 0) {

        SetBestMove(mvBest, nDepth);
    }
    return vlBest;
}

// root node alpha-beta search process
static int SearchRoot(int nDepth) {
    int vl, vlBest, mv, nNewDepth;
    SortStruct Sort{};

    vlBest = -MATE_VALUE;
    Sort.Init(Search.mvResult);
    while ((mv = Sort.Next()) != 0) {
        if (pos.MakeMove(mv)) {
            nNewDepth = pos.InCheck() ? nDepth : nDepth - 1;
            if (vlBest == -MATE_VALUE) {
                vl = -SearchFull(-MATE_VALUE, MATE_VALUE, nNewDepth, NO_NULL);
            } else {
                vl = -SearchFull(-vlBest - 1, -vlBest, nNewDepth);
                if (vl > vlBest) {
                    vl = -SearchFull(-MATE_VALUE, -vlBest, nNewDepth, NO_NULL);
                }
            }
            pos.UndoMakeMove();
            if (vl > vlBest) {
                vlBest = vl;
                Search.mvResult = mv;
                if (vlBest > -WIN_VALUE && vlBest < WIN_VALUE) {
                    vlBest += (rand() & RANDOM_MASK) - (rand() & RANDOM_MASK);
                }
            }
        }
    }
    RecordHash(HASH_PV, vlBest, nDepth, Search.mvResult);
    SetBestMove(Search.mvResult, nDepth);
    return vlBest;
}

// iterative deepening search process
static void SearchMain() {
    int i, t, vl, nGenMoves;
    int mvs[MAX_GEN_MOVES];

    // init
    memset(Search.nHistoryTable, 0, 65536 * sizeof(int));       // history table
    memset(Search.mvKillers, 0, LIMIT_DEPTH * 2 * sizeof(int)); // killer step
    memset(Search.HashTable, 0, HASH_SIZE * sizeof(HashItem));  // hash table
    t = clock();
    pos.nDistance = 0;

    // search openning book
    Search.mvResult = SearchBook();
    if (Search.mvResult != 0) {
        pos.MakeMove(Search.mvResult);
        if (pos.RepStatus(3) == 0) {
            pos.UndoMakeMove();
            return;
        }
        pos.UndoMakeMove();
    }

    // check if there is only one possible move
    vl = 0;
    nGenMoves = pos.GenerateMoves(mvs);
    for (i = 0; i < nGenMoves; i ++) {
        if (pos.MakeMove(mvs[i])) {
            pos.UndoMakeMove();
            Search.mvResult = mvs[i];
            vl ++;
        }
    }
    if (vl == 1) {
        return;
    }

    // iterative deepening process
    for (i = 1; i <= LIMIT_DEPTH; i ++) {
        vl = SearchRoot(i);
        if (vl > WIN_VALUE || vl < -WIN_VALUE) {
            break;
        }
        if (clock() - t > CLOCKS_PER_SEC) {
            break;
        }
    }
}

// AI response
static int ResponseMove() {
    int vlRep;

    SearchMain();
    cout << "search result is: " << Search.mvResult << endl;
    pos.MakeMove(Search.mvResult);

    Xqwl.mvLast = Search.mvResult;

    vlRep = pos.RepStatus(3);
    if (pos.IsMate()) {
        // check if the game is over
        cout << "GG! You lost." << endl;
        Xqwl.bGameOver = TRUE;
    } else if (vlRep > 0) {
        vlRep = pos.RepValue(vlRep);

        if (vlRep < -WIN_VALUE) {
            cout << "You lost!" << endl;
        } else if (vlRep > WIN_VALUE) {
            cout << "AI lost, you win!" << endl;
        } else {
            cout << "Draw!" << endl;
        }

        Xqwl.bGameOver = TRUE;
    } else if (pos.nMoveNum > 100) {
        cout << "draw" << endl;
        Xqwl.bGameOver = TRUE;
        Xqwl.bGameDraw = TRUE;
    } else {

        if (pos.InCheck()) {
            cout << "AI check!" << endl;
        } else if (pos.Captured()) {
            cout << "AI kill step" << endl;
        } else {
            cout << "AI normal move" << endl;
        }

        if (pos.Captured()) {
            pos.SetIrrev();
        }
    }
    return Search.mvResult;
}

// init
static void Startup(int idx) {
    pos.Startup(idx);
    Xqwl.sqSelected = Xqwl.mvLast = 0;
    Xqwl.bGameOver = FALSE;
}

string PrintMove(int mv) {
    int x_Src, y_Src, x_Dst, y_Dst;
    x_Src = SRC(mv) / 16 - 3;
    y_Src = SRC(mv) % 16 - 3;
    x_Dst = DST(mv) / 16 - 3;
    y_Dst = DST(mv) % 16 - 3;
    string outputStr =
            "[" + to_string(x_Src) + ", " + to_string(y_Src) + ", " +
            to_string(x_Dst) +
            ", " + to_string(y_Dst) + "]";
    return outputStr;
}

int main() {

    int listenfd, connfd;
    struct sockaddr_in servaddr;
    long n;
    srand((DWORD) time(NULL));
    InitZobrist();
    Xqwl.bFlipped = FALSE;
    Xqwl.bGameDraw = FALSE;

    if ((listenfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
        return 0;
    }
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(6667);
    if (bind(listenfd, (struct sockaddr *) &servaddr, sizeof(servaddr)) == -1) {
        printf("bind socket error: %s(errno: %d)\n", strerror(errno), errno);
        return 0;
    }
    if (listen(listenfd, 10) == -1) {
        printf("listen socket error: %s(errno: %d)\n", strerror(errno), errno);
        return 0;
    }

    while (1) {
        cout << "监听中心控制连接中..." << endl;
        if ((connfd = accept(listenfd, (struct sockaddr*)NULL, NULL)) == -1) {
            printf("accept socket error: %s(errno: %d)", strerror(errno), errno);
        }
        cout << "和中心控制已经连接！" << endl;

        while (1) {
            // receive player: 'r' or 'b'
            // chessboard status info  [x, y, c]#[x, y, c]#...
            cout << "接收 [下棋方] 和 [棋盘状态] 中..." << endl;
            char recv_player[2];
            char recv_boardInfo[1024];
            //  player
            n = recv(connfd, recv_player, 1, 0);
            cout << "接收下棋方长度: " << n << endl;
            if (n == 0) {
                break;
            }
            recv_player[n] = '\0';
            string player = string(recv_player);
            cout << "[下棋方 Player]: " << player << endl;
            //  chessboard status
            n = recv(connfd, recv_boardInfo, 1024, 0);
            cout << "接收棋盘状态信息长度: " << n << endl;
            recv_boardInfo[n] = '\0';
            string boardInfo = string(recv_boardInfo);
            cout << "[棋盘状态 boardInfo]: " << endl;
            cout << boardInfo << endl;

            cout << "原先棋盘状态:" << endl;
            pos.ClearBoard();
            pos.PrintBoard();
            pos.ReadInChess(boardInfo, pos.ucpcSquares, 256);
            cout << "读取后棋盘状态:" << endl;
            pos.PrintBoard();

            if (player == "r") {
                pos.sdPlayer = 0;
            } else if (player == "b") {
                pos.sdPlayer = 1;
            } else {
                cout << "PLAYER INFO ERROR!!" << endl;
            }

            // check if the game is over
            cout << "pos.sdPlayer: " << pos.sdPlayer << endl;
            cout << "pos.IsMate(): " << pos.IsMate() << endl;

            // 1. AI_mv[12] 2. is_mate[1] 3. winner[1]
            long int send_AI_mv_len;
            long int send_is_mate;
            long int send_winner;
            string AI_mv;
            string is_mate;
            string winner;
            //  check if is suicide move
            pos.ChangeSide();
            if (pos.Checked()) {
                cout << "棋局结束! 玩家 "<< pos.sdPlayer << "自杀了..." << endl;
                // 1. AI_mv
                AI_mv = "            ";
                send_AI_mv_len = send(connfd, AI_mv.c_str(), AI_mv.size(), 0);

                cout << "AI_mv为空，已经发送: " << AI_mv << endl;
                // 2. is_mate
                is_mate = "1";
                send_is_mate = send(connfd, is_mate.c_str(), is_mate.size(), 0);
                cout << "is_mate为1，已经发送: " << is_mate << endl;
                // 3. winner
                //    3.1 set winner
                if (player == "b") {
                    winner = "b";
                } else if (player == "r") {
                    winner = "r";
                }
                //    3.2 send winner
                send_winner = send(connfd, winner.c_str(), winner.size(), 0);
                cout << "winner为：" << winner << "，已经发送。" << endl;
                continue;
            } else {
                pos.ChangeSide();
            }

            //      if is over, return: is_mate = 1
            if (pos.IsMate()) {
                cout << "棋局结束!" << endl;
                // 1. AI_mv
                AI_mv = "            ";
                send_AI_mv_len = send(connfd, AI_mv.c_str(), AI_mv.size(), 0);
                //cout << "send_AI_mv len: " << send_AI_mv_len << endl;
                cout << "AI_mv为空，已经发送: " << AI_mv << endl;
                // 2. is_mate
                is_mate = "1";
                send_is_mate = send(connfd, is_mate.c_str(), is_mate.size(), 0);
                cout << "is_mate为1，已经发送: " << is_mate << endl;
                // 3. winner
                //    3.1 set winner
                if (player == "r") {
                    winner = "b";
                } else if (player == "b") {
                    winner = "r";
                }
                //    3.2 send winner
                send_winner = send(connfd, winner.c_str(), winner.size(), 0);
                cout << "winner为：" << winner << "，已经发送。" << endl;
            } else {
                //
                cout << "继续对弈..." << endl;
                // 1. AI_mv
                int AI_move = ResponseMove();
                cout << "当前下棋选手：" << pos.sdPlayer << endl;
                pos.PrintBoard();
                cout << "输赢判断：" << pos.IsMate() << endl;
                AI_mv = PrintMove(AI_move);
                send_AI_mv_len = send(connfd, AI_mv.c_str(), AI_mv.size(), 0);
                cout << "发送AI_mv = " << AI_mv << endl;
                // 2. is_mate
                is_mate = to_string(pos.IsMate());
                send_is_mate = send(connfd, is_mate.c_str(), is_mate.size(), 0);
                cout << "发送is_mate = " << is_mate << endl;
                // 3. winner
                winner = 'x';
                if (is_mate == "1") {
                    if (pos.sdPlayer == 0) {
                        winner = "b";
                    } else if (pos.sdPlayer == 1) {
                        winner = "r";
                    }
                }
                send_winner = send(connfd, winner.c_str(), winner.size(), 0);
                cout << "发送winner = " << winner << endl;
            }
        }
        //
        cout << "---关闭连接---" << endl;
        close(connfd);
    }

}





