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

    for box in mesh["boxes"]:
        if box[0] <= source_point[0] < box[1] and box[2] <= source_point[1] < box[3]:
            boxes.append(box)
        if box[0] <= destination_point[0] < box[1] and box[2] <= destination_point[1] < box[3]:
            boxes.append(box)
    
    if len(boxes) != 2:
        print("No path!")
    elif boxes[0] == boxes[1]:
        path.append(source_point)
        path.append(destination_point)
    else:
        boxes = simple_search(boxes[0], boxes[1], mesh)

    return path, boxes

def simple_search(source_box, dest_box, mesh):
    path = []
    found = False

    frontier = Queue()
    frontier.put(source_box)
    came_from = dict()
    came_from[source_box] = None

    while not frontier.empty():
        current_box = frontier.get()
        if current_box == dest_box:
            found = True
            break
        for next in mesh["adj"][current_box]:
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current_box

    if not found:
        print("No path!")
        return []
    else:
        current_box = dest_box
        while current_box != source_box:
            path.append(current_box)
            current_box = came_from[current_box]
        path.append(source_box)
        return path