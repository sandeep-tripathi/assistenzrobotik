import sys

robot = [None] * 128
human = [None] * 128

for i in xrange(128):
    robot[i] = [0.0] * 128
    human[i] = [0.0] * 128

def belongs_to_robot(x, y):
    global robot, human
    if 54 <= x <= 74 and 54 <= y <= 74:
        return True # Fixed robot area in the center
    if robot[x][y] > 0:
        return True # The pixel itself already belongs to the robot
    if (x < 127 and robot[x+1][y]) > 0 or (y < 127 and robot[x][y+1] > 0) or (x > 0 and robot[x-1][y]) > 0 or (y > 0 and robot[x][y-1] > 0):
        return True # A directly adjacent pixel belongs to the robot
    if (x < 127 and y < 127 and robot[x+1][y+1]) > 0 or (x > 0 and y < 127 and robot[x-1][y+1] > 0) or (x > 0 and y > 0 and robot[x-1][y-1]) > 0 or (x < 127 and y > 0 and robot[x+1][y-1] > 0):
        return True # A diagonally adjacent pixel belongs to the robot
    if \
            (x > 1 and y > 1 and robot[x-2][y-2] > 1) or \
            (x > 1 and y > 0 and robot[x-2][y-1] > 1) or \
            (x > 1 and robot[x-2][y] > 1) or \
            (x > 1 and y < 127 and robot[x-2][y+1] > 1) or \
            (x > 1 and y < 126 and robot[x-2][y+2] > 1) or \
            (x > 0 and y < 126 and robot[x-1][y+2] > 1) or \
            (y < 126 and robot[x][y+2] > 1) or \
            (x < 127 and y < 126 and robot[x+1][y+2] > 1) or \
            (x < 126 and y < 126 and robot[x+2][y+2] > 1) or \
            (x < 126 and y < 127 and robot[x+2][y+1] > 1) or \
            (x < 126 and robot[x+2][y] > 1) or \
            (x < 126 and y > 0 and robot[x+2][y-1] > 1) or \
            (x < 126 and y > 1 and robot[x+2][y-2] > 1) or \
            (x < 127 and y > 1 and robot[x+1][y-2] > 1) or \
            (y > 1 and robot[x][y-2] > 1) or \
            (x > 0 and y > 1 and robot[x-1][y-2] > 1):
        return True # A pixel two further definitively belongs to the robot

def surveillance_callback(image_msg):
    global robot, human

    # for y in range(0, 128):
    #     for x in range(0, 128):
    #         n = ord(image_msg.data[y*128+x])
    #         sys.stdout.write(". " if n == 127 else "0 " if n < 127 else "1 ")
    #     sys.stdout.write("\n")

    for y in xrange(0, 128):
        for x in xrange(0, 128):
            if 5 <= x <= 92 and 33 <= y <= 56:
                continue # Conveyor belt
            if 90 <= x <= 106 and 60 <= y <= 76:
                continue  # Sink right
            if 17 <= x <= 92 and 33 <= y <= 56:
                continue  # TODO: Sink middle
            if 17 <= x <= 92 and 33 <= y <= 56:
                continue  # TODO: Sink left

            value = ord(image_msg.data[y*128+x])
            if value > 127:  # definitively new pixels
                if belongs_to_robot(x, y):
                    robot[x][y] = robot[x][y] + 5.0
                    human[x][y] = 0.0
                else:
                    human[x][y] = human[x][y] + 5.0
                    robot[x][y] = 0.0
            elif value < 127:  # definitively removed pixels
                robot[x][y] = 0.0
                human[x][y] = 0.0
            else:  # nothing
                if human[x][y] > 0 and belongs_to_robot(x, y):
                    robot[x][y] += 5.0
                    human[x][y] = 0.0
                robot[x][y] = max(0, robot[x][y] - 0.05)
                human[x][y] = max(0, human[x][y] - 0.05)

    sys.stdout.write(chr(27)+'[2j')
    for y in xrange(0, 64):
        for x in xrange(0, 128):
            if robot[x][y] > 0 and robot[x][y+1] > 0 and human[x][y] > 0 and human[x][y+1] > 0:
                sys.stdout.write("X") # Collision on both
            elif robot[x][y] == 0 and robot[x][y+1] > 0 and human[x][y] == 0 and human[x][y+1] > 0:
                sys.stdout.write("x") # Collision bottom
            elif robot[x][y] > 0 and robot[x][y+1] == 0 and human[x][y] > 0 and human[x][y+1] == 0:
                sys.stdout.write("*") # Collision top
            elif robot[x][y] > 0 and robot[x][y+1] > 0:
                sys.stdout.write("O")
            elif robot[x][y] > 0 and robot[x][y+1] == 0:
                sys.stdout.write("o")
            elif robot[x][y] == 0 and robot[x][y+1] > 0:
                sys.stdout.write("o")
            elif human[x][y] > 0 and human[x][y+1] > 0:
                sys.stdout.write(":")
            elif human[x][y] > 0 and human[x][y + 1] == 0:
                sys.stdout.write(".")
            elif human[x][y] == 0 and human[x][y + 1] > 0:
                sys.stdout.write(".")
            else:
                sys.stdout.write(" ")
        sys.stdout.write("\n")
    sys.stdout.flush()
