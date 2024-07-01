from heapq import heappop, heappush
from math import sqrt

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
    heappush(frontier, (0, dest_box, False))
    heappush(frontier, (0, source_box, True))

    back_prev = dict()
    back_prev[source_box] = None
    forw_prev = dict()
    forw_prev[dest_box] = None
    pathcosts_back = dict()
    pathcosts_back[source_box] = 0
    pathcosts_forw = dict()
    pathcosts_forw[dest_box] = 0
    detail_points_back = dict()
    detail_points_back[source_box] = source_point
    detail_points_forw = dict()
    detail_points_forw[dest_box] = destination_point

    final_box = source_box

    while frontier:     
        queued, current_box, travelDirection = heappop(frontier)
        if (current_box != source_box and current_box != dest_box):
            if (pathcosts_back.get(current_box) is not None and pathcosts_forw.get(current_box) is not None):
                found = True
                final_box = current_box
                break
        for next in mesh["adj"][current_box]:
            #detail_points_back[next] = detail_point
            if (travelDirection == True): 
                distance, detail_point = calculate_distance(current_box, next, detail_points_back)
                new_cost = pathcosts_back[current_box] + distance
                if next not in pathcosts_back or new_cost < pathcosts_back[next]:
                    pathcosts_back[next] = new_cost
                    back_prev[next] = current_box
                    detail_points_back[next] = detail_point
                    priority = new_cost + heuristic(destination_point, detail_point)
                    heappush(frontier, (priority, next, True))
                    boxes.append(next)   #push the box we just visited so it appears on the visual. delete if you want less boxes on screen
            else: 
                distance, detail_point = calculate_distance(current_box, next, detail_points_forw)
                new_cost = pathcosts_forw[current_box] + distance
                if next not in pathcosts_forw or new_cost < pathcosts_forw[next]:
                    pathcosts_forw[next] = new_cost
                    forw_prev[next] = current_box
                    detail_points_forw[next] = detail_point
                    priority = new_cost + heuristic(source_point, detail_point)
                    heappush(frontier, (priority, next, False))
                    boxes.append(next)   #push the box we just visited so it appears on the visual. delete if you want less boxes on screen
        
    if not found:
        print("No path!")
        return []
    else:
        current_box = final_box              #Set iterator to the box both directions found
        while current_box != source_box:     #We end once we find source box.
            next_box = back_prev[current_box]      #set a temporary copy of the box we are traveling to.
            path.append(detail_points_back[current_box]) #add to the back of the list
            #boxes.append(current_box)  #Uncomment this line for a cleaner amount of boxes ;)
            current_box = back_prev[current_box]    #traverse to next box 
        path.append(source_point)                   
        current_box = final_box
        while current_box != dest_box:    #We end once we find last box.
            next_box = forw_prev[current_box]      #set a temporary copy of the box we are traveling to.
            path.insert(0,detail_points_forw[current_box])      #add to the front of the list
            #boxes.append(current_box)  #Uncomment this line for a cleaner amount of boxes ;)
            current_box = next_box    #traverse to next box 
        path.insert(0,destination_point)
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

def heuristic(goal, next_point):
    estimate = sqrt((goal[1] - next_point[1])**2 + (goal[0] - next_point[0])**2)
    return estimate