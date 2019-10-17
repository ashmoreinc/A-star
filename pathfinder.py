"""
A* Path finding with Euclidean Heuristics
"""

WORLD: list = [[0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
               [1, 1, 1, 1, 1, 1, 1, 1, 0, 1],
               [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]  # This will be the array that will store the values required

# These will be the array placements that the algorithm will use to identify obstacles and paths
OBSTACLE: int = 1
AVAILABLE: int = 0

# These will be the positions where the algorithm starts tracking and where it aims to track too
START_POS: list = [1, 8]  # List in form [x, y]
END_POS: list = [1, 4]  # List in form [x, y]

# Already tracked nodes
traced: dict = {}  # Key will be a string of the position in the form 'x, y'

# Setting to allow for diagonal tracking
Allow_Diag: bool = True


class Node:
    """This represents a single point on the grid"""
    def __init__(self, position: list, previous_node, is_start_node: bool = False):
        self.IsStart = is_start_node

        self.Pos = position  # List in form [x, y]
        self.Previous = previous_node

        self.Euclidean = 0
        self.calculate_euclidean_distance()

        self.Surroundings = {}  # Dict with n, s, e, w, ne, se, sw, nw as identifiers

        self.Completed = False  # Signifies whether or not it is completed.

    def is_end_node(self) -> bool:
        """Checks if this node is the end node"""
        return self.Pos == END_POS

    def calculate_euclidean_distance(self):
        """Calculate the euclidean distance using pythagoras' theorem"""

        vert_dist = self.Pos[1] - END_POS[1]
        # Make positive
        if vert_dist < 0:
            vert_dist *= -1

        horiz_dist = self.Pos[0] - END_POS[0]
        # Make positive
        if horiz_dist < 0:
            horiz_dist *= -1

        dist = (horiz_dist ** 2 + vert_dist ** 2) ** 0.5  # sqr root of a^2 + b^2

        self.Euclidean = dist

    def get_surrounding_nodes(self):
        """Get and store all the nodes surrounding this node, being conscious of edge cases."""
        global traced, Allow_Diag, WORLD, AVAILABLE, OBSTACLE

        surroundings = {}  # Dict with n, s, e, w, ne, se, sw, nw as identifiers

        # Get the node to the left
        left_pos = [self.Pos[0] - 1, self.Pos[1]]  # [x, y]
        if left_pos[0] < 0:
            surroundings["w"] = None
        else:
            # Create the key that will be used to reference this new node in the traced dict
            key = str(left_pos[0]) + ", " + str(left_pos[1])

            if WORLD[left_pos[1]][left_pos[0]] == AVAILABLE:
                # Check if the position is an available path (ie not an obstacle)
                # Now check if it has already been saved in the traced nodes
                if key not in traced:
                    traced[key] = Node(left_pos, self)

                surroundings["w"] = traced[key]

        # Get the node to the right
        right_pos = [self.Pos[0] + 1, self.Pos[1]]  # [x, y]
        if right_pos[0] > len(WORLD[self.Pos[1]]) - 1:
            surroundings["e"] = None
        else:
            # Create the key that will be used to reference this new node in the traced dict
            key = str(right_pos[0]) + ", " + str(right_pos[1])
            if WORLD[right_pos[1]][right_pos[0]] == AVAILABLE:
                # Check if the position is an available path (ie not an obstacle)
                # Now check if it has already been saved in the traced nodes
                if key not in traced:
                    traced[key] = Node(right_pos, self)

                surroundings["e"] = traced[key]

        # Get the node to the top
        top_pos = [self.Pos[0], self.Pos[1] - 1]  # [x, y]
        if top_pos[1] < 0:
            surroundings["n"] = None
        else:
            # Create the key that will be used to reference this new node in the traced dict
            key = str(top_pos[0]) + ", " + str(top_pos[1])

            if WORLD[top_pos[1]][top_pos[0]] == AVAILABLE:
                # Check if the position is an available path (ie not an obstacle)
                # Now check if it has already been saved in the traced nodes
                if key not in traced:
                    traced[key] = Node(top_pos, self)

                surroundings["n"] = traced[key]

        # Get the node to the bottom
        bottom_pos = [self.Pos[0], self.Pos[1] + 1]  # [x, y]
        if bottom_pos[1] > len(WORLD) - 1:
            surroundings["s"] = None
        else:
            # Create the key that will be used to reference this new node in the traced dict
            key = str(bottom_pos[0]) + ", " + str(bottom_pos[1])
            if WORLD[bottom_pos[1]][bottom_pos[0]] == AVAILABLE:
                # Check if the position is an available path (ie not an obstacle)
                # Now check if it has already been saved in the traced nodes
                if key not in traced:
                    traced[key] = Node(bottom_pos, self)

                surroundings["s"] = traced[key]

        # Diagonal surroundings
        if Allow_Diag:
            # Get top left
            top_left_pos = [self.Pos[0] - 1, self.Pos[1] - 1]  # [x, y]
            if top_left_pos[0] < 0 or top_left_pos[1] < 0:
                surroundings['nw'] = None
            else:
                # Create the key to reference the tracer dict
                key = f'{top_left_pos[0]}, {top_left_pos[1]}'
                if WORLD[top_left_pos[1]][top_left_pos[0]] == AVAILABLE:
                    # Check if the path is available, ie not an obstacle
                    # Now check if it hasn't already been traced
                    if key not in traced:
                        traced[key] = Node(top_left_pos, self)

            # Get top right
            top_right_pos = [self.Pos[0] + 1, self.Pos[1] - 1]  # [x, y]
            if top_right_pos[0] > len(WORLD[top_left_pos[1]]) - 1 or top_right_pos[1] < 0:
                surroundings['ne'] = None
            else:
                # Create the key to reference the tracer dict
                key = f'{top_right_pos[0]}, {top_right_pos[1]}'
                if WORLD[top_right_pos[1]][top_right_pos[0]] == AVAILABLE:
                    # Check if the path is available, ie not an obstacle
                    # Now check if it hasn't already been traced
                    if key not in traced:
                        traced[key] = Node(top_right_pos, self)

            # Get bottom left
            bottom_left_pos = [self.Pos[0] - 1, self.Pos[1] + 1]  # [x, y]
            if bottom_left_pos[0] < 0 or bottom_left_pos[1] > len(WORLD) - 1:
                surroundings['sw'] = None
            else:
                # Create the key to reference the tracer dict
                key = f'{bottom_left_pos[0]}, {bottom_left_pos[1]}'
                if WORLD[bottom_left_pos[1]][bottom_left_pos[0]] == AVAILABLE:
                    # Check if the path is available, ie not an obstacle
                    # Now check if it hasn't already been traced
                    if key not in traced:
                        traced[key] = Node(bottom_left_pos, self)

            # Get bottom left
            bottom_right_pos = [self.Pos[0] + 1, self.Pos[1] + 1]  # [x, y]
            if bottom_right_pos[1] > len(WORLD) - 1:
                surroundings['sw'] = None
            elif bottom_right_pos[0] > len(WORLD[bottom_right_pos[1]]) - 1:
                surroundings['sw'] = None
            else:
                # Create the key to reference the tracer dict
                key = f'{bottom_right_pos[0]}, {bottom_right_pos[1]}'
                if WORLD[bottom_right_pos[1]][bottom_right_pos[0]] == AVAILABLE:
                    # Check if the path is available, ie not an obstacle
                    # Now check if it hasn't already been traced
                    if key not in traced:
                        traced[key] = Node(bottom_right_pos, self)

    def __str__(self):
        return f'Node ({str(self.Pos)} - Status: {self.Completed}'


def initialise_start():
    """Initialise the start position with a Node class"""
    global START_POS, traced

    key = str(START_POS[0]) + ", " + str(START_POS[1])
    node = Node(START_POS, None, is_start_node=True)

    traced[key] = node


def output_found_path():
    """Outputs the path once found."""
    # Fill an array with each step of the path
    # Back track through the nodes from end to start
    path = []  # This will be reversed as we are backtracking, we fix that later.

    current_node_key = f'{END_POS[0]}, {END_POS[1]}'
    current_node: Node = traced[current_node_key]

    path.append(current_node.Pos)

    at_start = False
    while not at_start:
        current_node: Node = current_node.Previous

        path.append(current_node.Pos)

        if current_node.IsStart:
            at_start = True

    # Then flip the reversed array so that its in the correct order.
    path.reverse()

    return path


def a_start_path_find() -> bool:
    """The A* algorithm"""
    # Initialise the start node
    initialise_start()

    # Loop till a path has been found or till all possible routes have been checked.
    path_found: bool = False
    while not path_found:

        lowest_euclid_node = None
        for node_key in traced:
            node = traced[node_key]

            if node.Completed:  # Skip this node if it has already been completed.
                continue

            if lowest_euclid_node is None:
                lowest_euclid_node = node
            elif node.Euclidean < lowest_euclid_node.Euclidean:
                lowest_euclid_node = node
        # Else: we dont need to do anything

        if lowest_euclid_node is None:
            break
        elif lowest_euclid_node.is_end_node():
            path_found = True
        else:
            lowest_euclid_node.get_surrounding_nodes()
            lowest_euclid_node.Completed = True

    return path_found


if __name__ == "__main__":
    if a_start_path_find():
        print("Path found.")
        print(output_found_path())
    else:
        print("Path could not be found.")
