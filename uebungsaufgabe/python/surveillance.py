robot = []
human = []

for x in range(0, 128):
    robot[x] = []
    human[x] = []
    for y in range(0, 128):
        robot[x][y] = 0
        human[x][y] = 0

def belongs_to_robot(x, y):
    global robot, human
    if 123 <= x <= 150 and 123 <= y <= 150:
        return True # Fixed robot area in the center
    if robot[x][y] > 0:
        return True # The pixel itself already belongs to the robot
    if (x < 127 and robot[x+1][y]) > 0 or (y < 127 and robot[x][y+1] > 0) or (x > 0 and robot[x-1][y]) > 0 or (y > 0 and robot[x][y-1] > 0):
        return True # A directly adjacent pixel belongs to the robot
    if (x < 127 and y < 127 and robot[x+1][y+1]) > 0 or (x > 0 and y < 127 and robot[x-1][y+1] > 0) or (x > 0 and y > 0 and robot[x-1][y-1]) > 0 or (x < 127 and y > 0 and robot[x-1][y+1] > 0):
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
    for y in range(0, image_msg.height):
        offset = y * image_msg.width
        for x in range(0, image_msg.width):
            if 17 <= x <= 92 and 33 <= y <= 56:
                continue # Conveyor belt
            if 17 <= x <= 92 and 33 <= y <= 56:
                continue  # Sink 1
            if 17 <= x <= 92 and 33 <= y <= 56:
                continue  # Sink 2
            if 17 <= x <= 92 and 33 <= y <= 56:
                continue  # Sink 3

            value = image_msg.data[offset+x]
            if value > 127:  # definitively new pixels
                if belongs_to_robot(x, y):
                    robot[x][y] += 5
                else:
                    human[x][y] += 5
            elif value < 127:  # definitively removed pixels
                if belongs_to_robot(x, y):
                    robot[x][y] -= 5
                else:
                    human[x][y] -= 5
            else:  # nothing
                if belongs_to_robot(x, y):
                    robot[x][y] -= 1
                else:
                    human[x][y] -= 1

            if human[x][y] < 0:
                human[x][y] = 0
            if robot[x][y] < 0:
                robot[x][y] = 0
