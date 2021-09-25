from mirobot import Mirobot


class RobotController:

    def __init__(self, player: str):
        self.player = player
        if self.player == "b":
            self.robot = Mirobot(portname='/dev/ttyUSB0', debug=False)
            self.chessPickupZ = 37
        else:  # self.player == "r"
            self.robot = Mirobot(portname='/dev/ttyUSB1', debug=False)
            self.chessPickupZ = 31

        self.mapTable = [[[0, 0, self.chessPickupZ] for i in range(1001)] for j in range(1101)]
        self.CalculateMapTable()

        self.RobotInit()

    # regression function
    # Y regression
    def RegressionY(self, A, B, index):
        [row1, col1] = A
        [row2, col2] = B
        [x1, y1, z1] = self.mapTable[row1][col1]
        [x2, y2, z2] = self.mapTable[row2][col2]

        distance = index - col1
        step = col2 - col1
        x = x1 + (x2 - x1) / step * distance
        y = y1 + (y2 - y1) / step * distance
        z = z1 + (z2 - z1) / step * distance
        P = [round(x, 2), round(y, 2), round(z, 2)]
        return P

    # X regression
    def RegressionX(self, A, B, index):
        [row1, col1] = A
        [row2, col2] = B
        [x1, y1, z1] = self.mapTable[row1][col1]
        [x2, y2, z2] = self.mapTable[row2][col2]

        distance = index - row1
        step = row2 - row1
        x = x1 + (x2 - x1) / step * distance
        y = y1 + (y2 - y1) / step * distance
        z = z1 + (z2 - z1) / step * distance
        P = [round(x, 2), round(y, 2), round(z, 2)]
        return P

    # calculating mapping table
    def CalculateMapTable(self):
        if self.player == "r":
            # red
            # line 1, x = 0, y = [0,2,4,6,8,10]
            self.mapTable[0][0] = [86.5, 99, 36.7]
            self.mapTable[0][200] = [124.6, 99, 36.7]
            self.mapTable[0][400] = [156.6, 101, 36.7]
            self.mapTable[0][600] = [193.1, 102.5, 36.2]
            self.mapTable[0][800] = [229.6, 102.5, 36.2]
            self.mapTable[0][1000] = [264.6, 102.5, 35.7]
            # line 2, x = 1, y = [0,2,4,6,8,10]
            self.mapTable[100][0] = [86.5, 84, 36.7]
            self.mapTable[100][200] = [124.6, 84, 36.7]
            self.mapTable[100][400] = [156.6, 84, 36.7]
            self.mapTable[100][600] = [193.1, 85.5, 36.2]
            self.mapTable[100][800] = [229.6, 85.5,36.2]
            self.mapTable[100][1000] = [264.6, 85.5, 35.7]
            # line 3, x = 3, y = [0,2,4,6,8,10]
            self.mapTable[300][0] = [87.5, 47, 36.7]
            self.mapTable[300][200] = [122.6, 47, 36.7]
            self.mapTable[300][400] = [157.1, 48, 36.7]
            self.mapTable[300][600] = [194.6, 49.5, 36.2]
            self.mapTable[300][800] = [230.1, 49, 36.2]
            self.mapTable[300][1000] = [265.1, 49, 36.2]
            # line 4, x = 5, y = [0,2,4,6,8,10]
            self.mapTable[500][0] = [91.6, 9.5, 37.2]
            self.mapTable[500][200] = [126.6, 9.5, 37.2]
            self.mapTable[500][400] = [159.6, 13, 36.7]
            self.mapTable[500][600] = [193.6, 13.5, 36.2]
            self.mapTable[500][800] = [232.6, 15, 35.7]
            self.mapTable[500][1000] = [267.6, 15, 35.7]
            # line 5, x = 6, y = [0,2,4,6,8,10]
            self.mapTable[600][0] = [91.6, -8, 36.7]
            self.mapTable[600][200] = [126.6, -8, 36.7]
            self.mapTable[600][400] = [158.6, -6, 36.7]
            self.mapTable[600][600] = [194.6, -4, 36.7]
            self.mapTable[600][800] = [232.6, -3.5, 35.7]
            self.mapTable[600][1000] = [267.6, -3.5, 35.7]
            # line 6, x = 8, y = [0,2,4,6,8,10]
            self.mapTable[800][0] = [93.6, -43, 36.7]
            self.mapTable[800][200] = [128.6, -43, 36.7]
            self.mapTable[800][400] = [159.6, -42, 36.7]
            self.mapTable[800][600] = [196.6, -40, 35.7]
            self.mapTable[800][800] = [231.6, -39, 35.7]
            self.mapTable[800][1000] = [266.6, -39, 35.7]
            # line 7, x = 10, y = [0,2,4,6,8,10]
            self.mapTable[1000][0] = [90.6, -79, 36.7]
            self.mapTable[1000][200] = [125.6, -79, 36.7]
            self.mapTable[1000][400] = [162.6, -76.5, 36.2]
            self.mapTable[1000][600] = [197.6, -75, 35.7]
            self.mapTable[1000][800] = [234.1, -75.5, 35.2]
            self.mapTable[1000][1000] = [269.1, -75.5, 35.2]
            # line 8, x = 11, y = [0,2,4,6,8,10]
            self.mapTable[1100][0] = [90.1, -95.5, 36.2]
            self.mapTable[1100][200] = [125.1, -95.5, 36.2]
            self.mapTable[1100][400] = [162.6, -92.5, 36.2]
            self.mapTable[1100][600] = [197.6, -91.5, 35.7]
            self.mapTable[1100][800] = [234.1, -92, 35.2]
            self.mapTable[1100][1000] = [269.1, -92, 35.2]
        else:
            # black
            # line 1, x = 0, y = [0,2,4,6,8,10]
            self.mapTable[0][0] = [269.6, -98, 37.7]
            self.mapTable[0][200] = [234.6, -98, 37.7]
            self.mapTable[0][400] = [199.7, -98.5, 37.7]
            self.mapTable[0][600] = [163.2, -99, 37.7]
            self.mapTable[0][800] = [128.2, -100, 37.7]
            self.mapTable[0][1000] = [92.7, -101, 37.7]
            # line 2, x = 1, y = [0,2,4,6,8,10]
            self.mapTable[100][0] = [269.6, -81, 37.7]
            self.mapTable[100][200] = [234.6, -82, 37.7]
            self.mapTable[100][400] = [198.7, -82, 37.7]
            self.mapTable[100][600] = [162.2, -83, 37.7]
            self.mapTable[100][800] = [127.2, -84, 37.7]
            self.mapTable[100][1000] = [92.2, -85, 37.7]
            # line 3, x = 3, y = [0,2,4,6,8,10]
            self.mapTable[300][0] = [265.2, -45, 37.7]
            self.mapTable[300][200] = [230.7, -44.5, 37.7]
            self.mapTable[300][400] = [196.2, -44.5, 37.7]
            self.mapTable[300][600] = [159.2, -44.5, 38.7]
            self.mapTable[300][800] = [127.7, -47, 37.7]
            self.mapTable[300][1000] = [96.2, -47, 37.7]
            # line 4, x = 5, y = [0,2,4,6,8,10]
            self.mapTable[500][0] = [264.2, -9.5, 37.7]
            self.mapTable[500][200] = [231.2, -9.5, 37.7]
            self.mapTable[500][400] = [198.7, -9.5, 37.7]
            self.mapTable[500][600] = [162.7, -10, 37.7]
            self.mapTable[500][800] = [127.7, -10, 38.2]
            self.mapTable[500][1000] = [92.2, -10, 38.2]
            # line 5, x = 6, y = [0,2,4,6,8,10]
            self.mapTable[600][0] = [265.7, 10, 37.7]
            self.mapTable[600][200] = [230.7, 10, 37.7]
            self.mapTable[600][400] = [195.7, 10, 37.7]
            self.mapTable[600][600] = [159.7, 9, 37.7]
            self.mapTable[600][800] = [127.7, 9, 38.2]
            self.mapTable[600][1000] = [95.7, 9, 38.2]
            # line 6, x = 8, y = [0,2,4,6,8,10]
            self.mapTable[800][0] = [263.1, 45.5, 37.7]
            self.mapTable[800][200] = [230.6, 46, 37.7]
            self.mapTable[800][400] = [198.6, 46, 37.7]
            self.mapTable[800][600] = [163.6, 46, 37.7]
            self.mapTable[800][800] = [127.6, 47, 37.7]
            self.mapTable[800][1000] = [91.6, 47.5, 37.7]
            # line 7, x = 10, y = [0,2,4,6,8,10]
            self.mapTable[1000][0] = [267.1, 80.5, 37.7]
            self.mapTable[1000][200] = [231.1, 80.5, 37.7]
            self.mapTable[1000][400] = [195.1, 81, 37.7]
            self.mapTable[1000][600] = [159.6, 81, 37.7]
            self.mapTable[1000][800] = [124.6, 82.5, 37.7]
            self.mapTable[1000][1000] = [89.6, 83, 37.7]
            # line 8, x = 11, y = [0,2,4,6,8,10]
            self.mapTable[1100][0] = [266.1, 115.5, 37.7]
            self.mapTable[1100][200] = [230.1, 115.5, 37.7]
            self.mapTable[1100][400] = [195.6, 116, 37.7]
            self.mapTable[1100][600] = [160.1, 116, 37.7]
            self.mapTable[1100][800] = [123.1, 117, 37.7]
            self.mapTable[1100][1000] = [90.1, 118, 37.7]

        # row: 0 1 3 5 6 8 10
        for row in [0, 1, 3, 5, 6, 8, 10, 11]:
            for col in range(1000):
                row_A = row * 100
                col_A = 200 * (col // 200)
                row_B = row * 100
                col_B = 200 * (col // 200 + 1)
                self.mapTable[100 * row][col] = self.RegressionY(A=[row_A, col_A], B=[row_B, col_B], index=col)
        # 0-1
        for row in range(0, 100):
            for col in range(1001):
                self.mapTable[row][col] = self.RegressionX(A=[0, col], B=[100, col], index=row)
        # 1-3
        for row in range(100, 300):
            for col in range(1001):
                self.mapTable[row][col] = self.RegressionX(A=[100, col], B=[300, col], index=row)
        # 3-5
        for row in range(300, 500):
            for col in range(1001):
                self.mapTable[row][col] = self.RegressionX(A=[300, col], B=[500, col], index=row)

        # 5-6
        for row in range(500, 600):
            for col in range(1001):
                self.mapTable[row][col] = self.RegressionX(A=[500, col], B=[600, col], index=row)
        # 6-8
        for row in range(600, 800):
            for col in range(1001):
                self.mapTable[row][col] = self.RegressionX(A=[600, col], B=[800, col], index=row)
        # 8-10
        for row in range(800, 1000):
            for col in range(1001):
                self.mapTable[row][col] = self.RegressionX(A=[800, col], B=[1000, col], index=row)
        # 10-11
        for row in range(1000, 1100):
            for col in range(1001):
                self.mapTable[row][col] = self.RegressionX(A=[1000, col], B=[1100, col], index=row)

    # convert coordinate
    def Convert2RobotCoordinate(self, ChessPos):
        # be careful of the margin +100
        robotCoordinate = (self.mapTable[ChessPos[0]+100][ChessPos[1]+100][0], self.mapTable[ChessPos[0]+100][ChessPos[1]+100][1])
        return robotCoordinate

    # Chess at ChessAPos eat chess at ChessAPos
    def Eat(self, ChessAPos, ChessBPos):

        self.Drop(ChessBPos)
        self.Move(ChessAPos, ChessBPos)
        print("Chess eat operation complete")

    # Move chess at ChessAPos to ChessBPos
    def Move(self, ChessAPos, ChessBPos):
        # Pick up
        chessRealPosA = self.Convert2RobotCoordinate(ChessAPos)
        self.robot.set_wrist_pose(chessRealPosA[0], chessRealPosA[1], self.chessPickupZ + 10)
        self.robot.set_wrist_pose(chessRealPosA[0], chessRealPosA[1], self.chessPickupZ)
        self.robot.pump_on()
        self.robot.set_wrist_pose(chessRealPosA[0], chessRealPosA[1], self.chessPickupZ + 10)

        # Put down
        chessRealPosB = self.Convert2RobotCoordinate(ChessBPos)
        self.robot.set_wrist_pose(chessRealPosB[0], chessRealPosB[1], self.chessPickupZ + 10)
        self.robot.set_wrist_pose(chessRealPosB[0], chessRealPosB[1], self.chessPickupZ + 1)
        self.robot.pump_off()
        # Lift the robotic arm
        self.robot.set_wrist_pose(chessRealPosB[0], chessRealPosB[1], self.chessPickupZ + 15)
        # Go to standby position
        self.robot.go_to_zero()
        self.robot.set_joint_angle({1: 90.0}, wait=True)
        print("Chess move operation complete")

    # drop a chess
    def Drop(self, ChessAPos):
        # Pick up
        chessRealPosA = self.Convert2RobotCoordinate(ChessAPos)
        self.robot.set_wrist_pose(chessRealPosA[0], chessRealPosA[1], self.chessPickupZ + 20)
        self.robot.set_wrist_pose(chessRealPosA[0], chessRealPosA[1], self.chessPickupZ)
        self.robot.pump_on()

        # Lift the robotic arm
        self.robot.set_wrist_pose(chessRealPosA[0], chessRealPosA[1], self.chessPickupZ + 10)
        # Put down
        chessDropPos = (100, -180, 100)
        self.robot.set_wrist_pose(chessDropPos[0], chessDropPos[1], chessDropPos[2])
        self.robot.pump_off()
        print("Chess drop operation complete")

    # initialize
    def RobotInit(self):
        self.robot.home_simultaneous()
        self.robot.set_joint_angle({1: 90.0}, wait=True)
        print("Robot-Black initialization complete")
