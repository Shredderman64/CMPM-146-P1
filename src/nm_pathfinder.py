from heapq import heappop, heappush
from math import sqrt
#from queue import Queue

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    path = []
    boxes = []

    for box in mesh["boxes"]:
        if box[0] <= source_point[0] < box[1] and box[2] <= source_point[1] < box[3]:
            boxes.append(box)
            source_box = box    #used to call simple_search
        if box[0] <= destination_point[0] < box[1] and box[2] <= destination_point[1] < box[3]:
            boxes.append(box)
            dest_box = box      
    
    if len(boxes) != 2:
        print("No path!")
    elif boxes[0] == boxes[1]:  #if start and end are in the same box then create a line between them.
        path.append(source_point)
        path.append(destination_point)
    else:
        path, boxes = navmesh_search(source_box, dest_box, mesh,source_point,destination_point,boxes)
    # print(path)
    return path, boxes

def navmesh_search(source_box, dest_box, mesh,source_point,destination_point,boxes):
    path = []
    found = False

    frontier = []
    heappush(frontier, (0, source_box))
    came_from = dict()
    came_from[source_box] = None
    pathcosts = dict()
    pathcosts[source_box] = 0
    detail_points = dict()
    detail_points[source_box] = source_point
    detail_points[dest_box] = dest_box

    while frontier:     #BFS
        priority, current_box = heappop(frontier)
        if current_box == dest_box:
            found = True
            break
        for next in mesh["adj"][current_box]:
            distance, detail_point = calculate_distance(current_box, next, detail_points)
            #detail_points[next] = detail_point
            new_cost = priority + distance
            if next not in pathcosts or new_cost < pathcosts[next]:
                pathcosts[next] = new_cost
                came_from[next] = current_box
                detail_points[next] = detail_point
                heappush(frontier, (new_cost, next))
                #boxes.append(next)   #push the box we just visited so it appears on the visual. delete if you want less boxes on screen
        
    if not found:
        print("No path!")
        return []
    else:
        current_box = dest_box              #Set iterator to the destination box, we will work backwards
        path.append(destination_point)      #set the start position of the line to the end position.
        while current_box != source_box:    #We end once we find last box.
            next_box = came_from[current_box]      #set a temporary copy of the box we are traveling to.
            print(detail_points[next_box])
            path.append(detail_points[current_box])
            # print(tempPath)
            boxes.append(current_box)  #Uncomment this line for a cleaner amount of boxes ;)
            #path.append(tempPath)       #push the path we just saved
            current_box = came_from[current_box]    #traverse to next box (broken i think)
        path.append(source_point)
        return path, boxes
    
def calculate_distance(current_box, next_box, detail_points):
    #Save boxes as coordinates
    b1y1,b1y2,b1x1,b1x2 = current_box[0],current_box[1],current_box[2],current_box[3]   
    b2y1,b2y2,b2x1,b2x2 = next_box[0],next_box[1],next_box[2],next_box[3]
    #find range in the boxes
    xRange = [max(b1x1,b2x1),min(b1x2,b2x2)]
    yRange = [max(b1y1,b2y1),min(b1y2,b2y2)]
    
    sourceY,sourceX = detail_points[current_box]

    #find y location
    if sourceY < min(yRange):
        tempY = min(yRange)
    elif sourceY > max(yRange):
        tempY = max(yRange)
    else: tempY = sourceY
    #find x location
    if sourceX < min(xRange):
        tempX = min(xRange)
    elif sourceX > max(xRange):
        tempX = max(xRange)
    else: tempX = sourceX   #same as above

    detail_point = tempY,tempX

    # calculate distance between them
    distance = sqrt((tempX - sourceX)**2 + (tempY - sourceY)**2)
    return distance, detail_point

def calculate_path(current_box, next_box):
    return