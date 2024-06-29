from queue import Queue

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

    print(source_point)
    print(destination_point)

    for box in mesh["boxes"]:
        if box[0] <= source_point[0] < box[1] and box[2] <= source_point[1] < box[3]:
            boxes.append(box)
            source_box = box
        if box[0] <= destination_point[0] < box[1] and box[2] <= destination_point[1] < box[3]:
            boxes.append(box)
            dest_box = box
    
    if len(boxes) != 2:
        print("No path!")
    elif boxes[0] == boxes[1]:  #if start and end are in the same box then create a line between them.
        path.append(source_point)
        path.append(destination_point)
    else:
        path, boxes = simple_search(source_box, dest_box, mesh,source_point,destination_point,boxes)
    print(path)
    return path, boxes

def simple_search(source_box, dest_box, mesh,source_point,destination_point,boxes):
    path = []
    found = False

    frontier = Queue()
    frontier.put(source_box)
    came_from = dict()
    came_from[source_box] = None

    while not frontier.empty():     #BFS
        current_box = frontier.get()
        if current_box == dest_box:
            found = True
            break
        for next in mesh["adj"][current_box]:
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current_box
                boxes.append(next)   #push the box we just visited so it appears on the visual. #delete if you want less boxes on screen
        


    if not found:
        print("No path!")
        return []
    else:
        current_box = dest_box              #Set iterator to the destination box, we will work backwards
        path.append(destination_point)      #set the start position of the line to the end position.
        while current_box != source_box:    #We end once we find last box.
            next_box = came_from[current_box]      #set a temporary copy of the box we are traveling to.
            print(current_box)
            print(source_box)
            #Save boxes as coordinates
            b1y1,b1y2,b1x1,b1x2 = current_box[0],current_box[1],current_box[2],current_box[3]   
            b2y1,b2y2,b2x1,b2x2 = next_box[0],next_box[1],next_box[2],next_box[3]
            #find range in the boxes
            xRange = [max(b1x1,b2x1),min(b1x2,b2x2)]    
            yRange = [max(b1y1,b2y1),min(b1y2,b2y2)]
            print(xRange)
            print(yRange)
            #find y location:
            if path[-1][0] < min(yRange):
                 tempY = min(yRange)
            elif path[-1][0] > max(yRange):
                tempY = max(yRange)
            else: tempY = path[-1][0]   #May need a better way to do this part
            #find x location
            if path[-1][1] < min(xRange):
                tempX = min(xRange)
            elif path[-1][1] > max(xRange):
                tempX = max(xRange)
            else: tempX = path[-1][1]   #same as above


            #put it in path
            tempPath = tempY,tempX  #save the path with the values
            print(tempPath)
            #boxes.append(current_box)  Uncomment this line for a cleaner amount of boxes ;)
            path.append(tempPath)       #push the path we just saved
            current_box = came_from[current_box]    #traverse to next box (broken i think)
        path.append(source_point)
        return path, boxes