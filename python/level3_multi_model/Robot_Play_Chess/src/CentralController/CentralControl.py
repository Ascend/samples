# Copyright 2021 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import sys
import socket
import time
import cv2
import numpy as np

sys.path.append("..")
from RobotController.RobotController import RobotController

# mapping list of chess classes in different modules
map_list = [0, 1, 2, 3, 4, 5, 6, 13, 8, 9, 10, 11, 7, 12]

def EncodeBoard(chessBoard: list) -> str:
    """
    param:
        chessBoard: List-> [List ->[int:x, int: y, int: class], [], ...]

    return:
        boardInfo: str: [x, y, c]#[x, y, c]#...
    """
    boardInfo = '#'.join([str(item) for item in chessBoard])
    return boardInfo

def EncodeAImv(chessBoard: list, AI_mv: str, winner: str, is_mate: str) -> str:
    """
    param:
        chessBoard: List-> [List ->[int:x, int: y, int: class], [], ...]
        AI_mv: str: [x_src, y_src, x_dst, y_dst]
        winner: str: 'r' or 'b'
        is_mate: str: 0 or 1 or 2,
                      0: continue，1: end，2: draw

    return:
        AI_mv_info str: #[is_mate, player]#[x_src, y_src, class_src]#[x_dst, y_dst, class_dst]
    """
    if AI_mv == (" " * 12):
        [x_src, y_src, x_dst, y_dst] = [-1, 0, -1, 0]
    else:  # e.g. [6, 2, 6, 4]
        [x_src, y_src, x_dst, y_dst] = map(int, AI_mv[1:-1].split(','))
    c = -1
    for item in chessBoard:
        if item[0] == x_src and item[1] == y_src:
            c = item[2]

    part_1 = '[' + is_mate + ', ' + winner + ']'
    part_2 = str([x_src, y_src, c])
    part_3 = str([x_dst, y_dst, c])
    AI_mv_info = '#' + part_1 + '#' + part_2 + '#' + part_3
    return AI_mv_info

def DecodeHumanMv(Human_mv: str) -> str:
    """
    param:
        Human_mv: str #[x, y, c]#[x, y, c]
    return:
        mv: list of int [x_src, y_src, x_dst, y_dst]
    """
    Human_mv = Human_mv.strip()  # str #[]#[]
    templist = Human_mv.split('#')[1:]  # list [[x, y, c], [x, y, c]]
    x_src = templist[0][1:-1].split(',')[0]
    y_src = templist[0][1:-1].split(',')[1]
    x_dst = templist[1][1:-1].split(',')[0]
    y_dst = templist[1][1:-1].split(',')[1]
    mv = '[' + x_src + ', ' + y_src + ', ' + x_dst + ', ' + y_dst + ']'
    print("检查Human move：", mv)
    return mv

def IsKillStep(chessboard: list, mv: list) -> bool:
    """
    chessboard: current chessboard info [[x, y, class], [], []...]
    mv: AI move info, [x_src, y_src, x_dst, y_dst]

    return:
        BOOL: true if this step is kill step, false if normal step.
    """
    for pc in chessboard:
        if mv[2] == pc[0] and mv[3] == pc[1]:
            return True
        else:
            continue

    return False

def MoveStr2List(mv: str) -> list:
    """
    param:
        mv: str [x_src, y_src, x_dst, y_dst]

    return:
        mv_list: List-> [int: x_src, y_src, x_dst, y_dst]
    """
    mv_list = []
    if mv != "            ":
        mv_list = list(map(int, mv[1:-1].split(",")))
    return mv_list

def WrapInfo(info: str) -> str:
    """
    param:
        info: "xxxx"
    return:
        wrapped_info: "<xxxx>"
    """
    wrapped_info = "<" + info + ">"
    return wrapped_info

class CentralControl:
    def __init__(self):

        self.currentPlayer = "r"  # "r" or "b"
        self.gameMode = "Human VS AI"  # Play or Watch
        self.chessBoard = []  # [[int:x, y, class],[],...]
        self.chessBoard_real = []  # real position of chess on the chessboard
        self.isMate = 0  # 1 if game is over

        # socket of perception
        self.socket_perception = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.HOST_perception = "192.168.8.22"
        self.PORT_perception = 6002
        # socket of chess engine
        self.socket_playing = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.HOST_playing = "192.168.8.155"
        self.PORT_playing = 6667
        # socket of webserver
        self.socket_web = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.HOST_web = "192.168.8.155"
        self.PORT_web = 8231

        self.robotController_r = RobotController("r")  # red robot
        self.robotController_r.RobotInit()
        print("红方机械臂初始化完成")
        self.robotController_b = RobotController("b")  # black robot
        self.robotController_b.RobotInit()
        print("黑方机械臂初始化完成")

    def SocketConnect(self, switch: str):
        """
        str: sting length is 3, switch[0] is for perception module，and
                            switch[1] is for chess engine module, and
                            switch[2] is for webserver module.
        """
        # perception module
        if switch[0] == "1":
            try:
                self.socket_perception.connect((self.HOST_perception, self.PORT_perception))
            except socket.error as arg:
                (err_msg, errno) = arg
                print("连接棋盘理解模块失败，error message: %s, errNO : %d" % (err_msg, errno))
            print("连接棋盘理解模块成功")
        else:
            print("跳过连接棋盘理解模块")

        # chess engine module
        if switch[1] == "1":
            try:
                self.socket_playing.connect((self.HOST_playing, self.PORT_playing))
            except socket.error as arg:
                (err_msg, errno) = arg
                print("连接对弈引擎模块失败，error message: %s, errNO : %d" % (err_msg, errno))
            print("连接对弈引擎模块成功")
        else:
            print("跳过连接对弈引擎模块")

        # webserver module
        if switch[2] == "1":
            self.socket_web.bind((self.HOST_web, self.PORT_web))
            self.socket_web.listen(1)
            print("等待webserver端连接...")
            self.socket_web, self.web_addr = self.socket_web.accept()
            print("web_conn: ", self.socket_web)
            print("web_addr: ", self.web_addr)
            print("连接WebServer端成功")
        else:
            print("跳过连接web模块")

    def GetChessboardInfo(self):
        """
        Take a picture for perception with camera to get the chessboard information.
        """
        capture = cv2.VideoCapture(2)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 100]
        waitFrameNum = 10  # wait for AE becoming stable
        while waitFrameNum:
            _, frame = capture.read()
            waitFrameNum -= 1
        ret, frame = capture.read()
        result, imgencode = cv2.imencode('.jpg', frame, encode_param)
        data = np.array(imgencode)
        stringData = data.tobytes()

        self.socket_perception.send(str.encode(str(len(stringData)).ljust(16)))
        self.socket_perception.send(stringData)

        receive = self.socket_perception.recv(1024)
        if len(receive):
            print("Chess Perception Results:  ", str(receive, encoding='utf-8'))

        [boardStr, boardStr_real] = receive.decode('utf-8').split("&")
        board_recv = boardStr.split("#")
        board_recv_real = boardStr_real.split("#")
        print("检查打印board_recv_real: ", board_recv_real)

        # ideal chessboard info
        new_board = []
        for item in board_recv:
            new_board.append(list(map(int, item[1:-1].split(','))))
        self.chessBoard = new_board.copy()
        # real chessboard info
        new_board_real = []
        for item in board_recv_real:
            new_board_real.append(list(map(int, item[1:-1].split(','))))
        self.chessBoard_real = new_board_real.copy()

        print("读取棋盘状态完成，数据已经加载到self.chessBoard: ")
        print(self.chessBoard)
        print("打印真实坐标：")
        print(self.chessBoard_real)
        print('\n')

        time.sleep(1)
        capture.release()

    def NextMoveViaSocket(self, chessboard: list, player: str) -> (str, str):
        """
        param:
        chessboard: current chessboard status [[x,y,class], [], ...]
        player: str current player， 'r' or 'b'

        return:
        AI_mv_info: str #[0, r]#[x, y, c]#[]
        AI_mv: str [x_src, y_src, x_dst, y_dst]
        """

        boardInfo = EncodeBoard(chessboard)
        print("boardInfo is: ", boardInfo)

        self.socket_playing.send(player.encode('utf-8'))  # 'r' or 'b'
        print("player 已发送！")
        time.sleep(1)

        self.socket_playing.send(boardInfo.encode('utf-8'))
        print("boardInfo 已发送！")
        print(boardInfo)

        AI_mv = self.socket_playing.recv(12).decode('utf-8')  # [6, 2, 6, 4]
        print("AI_mv received: ", AI_mv, type(AI_mv))
        is_mate = self.socket_playing.recv(1).decode('utf-8')  # 1
        print("is_mate received: ", is_mate, type(is_mate))
        winner = self.socket_playing.recv(1).decode('utf-8')  # 'b'
        print("winner received: ", winner, type(winner))

        time.sleep(1)

        print("对弈引擎计算结束！")
        AI_mv_info = EncodeAImv(chessboard, AI_mv, winner, is_mate)
        print("AI_mv_info: ", AI_mv_info)
        return AI_mv_info, AI_mv

    def Ideal2Real(self, ideal_index: list) -> list:
        """
        param:
            ideal_index: [x_idl, y_idl]
        return:
            real_index: [x_real, y_real]

        """
        real_index = []
        for i in range(len(self.chessBoard)):
            if self.chessBoard[i][0] == ideal_index[0] and self.chessBoard[i][1] == ideal_index[1]:
                print("chess found: ", self.chessBoard[i])
                print("real position: ", [self.chessBoard_real[i][0], self.chessBoard_real[i][1]])
                real_index = [self.chessBoard_real[i][0], self.chessBoard_real[i][1]]
        if real_index:
            return real_index
        else:
            print("Chess not found, please check input!")

    def MakeMove(self, player: str, mv: str):
        """
        Make robot execute the move signal.
        mv: str [x_src, y_src, x_dst, y_dst]
        """

        move = MoveStr2List(mv)  # [6, 2, 6, 4]
        print("机械臂解析:", move, player)

        # change ideal position to real position
        pos1t = tuple(self.Ideal2Real([move[0], move[1]]))
        pos2t = (move[2] * 100, move[3] * 100)
        print("机械臂执行指令：", [pos1t, pos2t])
        # select robot
        if player == "r":
            cur_robot = self.robotController_b
        else:  # player == 'b'
            cur_robot = self.robotController_b
        # check the move type of robot, Eat or Move
        if IsKillStep(self.chessBoard, move):
            cur_robot.Eat(pos1t, pos2t)
            time.sleep(2)
        else:
            cur_robot.Move(pos1t, pos2t)
            time.sleep(2)

    # use human to replace robot
    def MakeMove_help(self, player: str, mv: str):
        move = MoveStr2List(mv)
        print("请人工移动棋子！！！！")
        print("行动方: ", player)
        print("行动指令: ", move)
        input("移动完毕请按回车确认...")

    # switch player
    def ChangeSide(self):
        if self.currentPlayer == "r":
            self.currentPlayer = "b"
            print("红方结束，轮到黑方下棋！")
        elif self.currentPlayer == "b":
            self.currentPlayer = "r"
            print("黑方结束，轮到红方下棋！")

    # AI VS AI
    def AI_VS_AI(self):
        print("进入AI VS AI分支...")

        while self.isMate != 1:

            # mv2UI str: #[0, r]#[x, y, c]#[x, y, c], mv2Robot str: [x, y, x, y]
            mv2UI, mv2Robot = self.NextMoveViaSocket(self.chessBoard, self.currentPlayer)
	    self.isMate = int(mv2UI[2])
            print("下棋方： ", self.currentPlayer)

            if mv2UI[9] != "-":
                self.MakeMove_help(self.currentPlayer, mv2Robot)

            self.GetChessboardInfo()

            mv2UI = WrapInfo(mv2UI.ljust(32))
            print("检查发给UI信息:", mv2UI)
            self.socket_web.send(mv2UI.encode('utf-8'))
            # switch player
            self.ChangeSide()
            try:
                data = self.socket_web.recv(11 + 2, 0x40)
                self.gameMode = data.decode('utf-8')[1:-1].strip()
                print("成功接收到模式转换信号！当前对弈模式：", self.gameMode)
                break
            except BlockingIOError as e:
                data = None
        # end of game
        print("Quit AI vs AI mode...")

    # Human VS AI
    def Human_VS_AI(self):
        print("进入Human VS AI分支...")
        # if game is not over yet
        while self.isMate != 1:
            if self.currentPlayer == "r":
                print("等待红方落子...")
                self.socket_web.send("<Waiting for the red>".encode('utf-8'))
                # receive red move info
                Human_mv = self.socket_web.recv(32 + 2).decode('utf-8')[1:-1]
                if Human_mv[0] != "#":
                    print("对弈模式改变！！离开人机，进入AI自对弈：", Human_mv)
                    central.gameMode = Human_mv
                    print("判断当前模式：", central.gameMode)
                    break
                # decode red move info
                mv2Robot_R = DecodeHumanMv(Human_mv)
                # red robot play
                self.MakeMove_help(self.currentPlayer, mv2Robot_R)
                # update chessboard info
                self.GetChessboardInfo()
                # switch player
                self.ChangeSide()
            # generate AI move via chess engine
            # mv2UI is str such as: #[0, r]#[x, y, c]#[x, y, c], mv2Robot str: [x, y, x, y]
            print("AI 思考中...请耐心等待...")
            mv2UI, mv2Robot_B = self.NextMoveViaSocket(self.chessBoard, self.currentPlayer)
            # update the value of self.isMate
            if mv2UI[2] == '1':
                central.isMate = 1
            # black robot play
            print("下棋方： ", self.currentPlayer)
            if mv2UI[9] != "-":
                self.MakeMove_help(self.currentPlayer, mv2Robot_B)
            # update chessboard
            self.GetChessboardInfo()
            # send move info to webserver
            mv2UI = WrapInfo(mv2UI.ljust(32))
            print("检查发给UI信息:", mv2UI)
            self.socket_web.send(mv2UI.encode('utf-8'))
            # switch player
            self.ChangeSide()
        # end of game
        print("Quit Human vs AI mode...")

    def Initialization(self, ini: str = "111"):
        """
        Initialize the game by building connection among all the modules.
        Initialize the chessboard after connections are established.
        """
        # establish connections
        self.SocketConnect(ini)
        print("等待接收UI端初始化信号...")
        init_signal = self.socket_web.recv(27 + 2).decode('utf-8')  # Request Initial Chess Board
        print("init signal: ", init_signal[1:-1])
        # get current chessboard info via perception module
        self.GetChessboardInfo()
        # send info to webserver
        self.socket_web.send(WrapInfo(EncodeBoard(central.chessBoard).ljust(512)).encode('utf-8'))
        init_ACK = self.socket_web.recv(10 + 2)  # Initial OK
        print("Initial ACK: ", init_ACK.decode('utf-8')[1:-1])

    def ChangeMode(self):
        """
        Change game mode by setting the content of self.gameMode
        """
        print("当前默认mode = ", self.gameMode, "\n接收对弈模式中...")
        # length of "Human VS AI" is 11
        self.gameMode = central.socket_web.recv(11 + 2).decode('utf-8')[1:-1].strip()
        print("当前对弈模式：", self.gameMode)

if __name__ == '__main__':
    # instantiate
    central = CentralControl()
    # initialize
    central.Initialization("111")
    # receive mode
    central.ChangeMode()
    while central.isMate != 1:
        # branch 1：AI VS AI
        while central.gameMode == "AI VS AI" and central.isMate != 1:
            central.AI_VS_AI()

        # branch 2：Human VS AI
        while central.gameMode == "Human VS AI" and central.isMate != 1:
            central.Human_VS_AI()
    print("对弈结束。请重新启动")
    time.sleep(10)
