import numpy as np
import random, sys, copy, math

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.strategy = sys.argv[2]

        self.first_run_finished = False

        self.heading_map = ['up', 'right', 'down', 'left']
        self.location = [0, 0]
        self.heading = 0 # u, r, d, l
        self.maze_dim = maze_dim
        self.goal_bounds = [self.maze_dim/2 - 1, self.maze_dim/2]
        self.counter = 1
        self.maze = np.empty((maze_dim, maze_dim), dtype=object)
        self.last_move_was = 'forward'
        self.edges = []
        self.current_path = []
        self.previous_node = "0-0"
        self.squares = {}
        self.state = "explore"
        self.moves_to_backtrack = []
        self.maze_analyzed_for_astar = False

        if self.strategy not in ["random", "astar"]:
            sys.exit("Strategy unknown: %s" % (self.strategy,))

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''

        rotation = 0
        movement = 0

        self.counter += 1

        if self.strategy == "random":
            # stop when the goal is first reached, as we don't have any
            # systematic approach to improve anyway
            if self.goal_reached():
                self.location = [0,0]
                self.heading = 0
                self.state = "explore"
                return 'Reset', 'Reset'

            if self.state == 'backtrack':
                if len(self.current_path) > 0:
                    heading, movement = self.heading, -1
                    move = self.current_path.pop()
                    self.heading = move[0]
                    self.update_location(movement)
                    return self.get_rotation_from_heading(heading) * -1, movement
                else:
                    self.state = "explore"
                    return 0, 0
            else:
                self.map(sensors)

                possible_moves = self.get_possible_moves(sensors)
                #print "possible moves: %s" % (possible_moves,)

                if len(possible_moves) > 1:
                    self.save_node()
                    heading, movement = self.choose_random_move(possible_moves)
                    self.current_path.append([heading, movement])
                    return self.make_move(heading, movement)
                if len(possible_moves) == 1:
                    heading, movement = [possible_moves[0][0], 1]
                    self.current_path.append([heading, movement])
                    return self.make_move(heading, movement)
                if len(possible_moves) == 0:
                    self.state = "backtrack"
                    return 0, 0


        if self.strategy == "astar":
            if not self.first_run_finished:
                if self.maze_fully_explored():
                    self.first_run_finished = True
                    self.counter = 0
                    self.location = [0, 0]
                    self.heading = 0
                    self.state = "explore"
                    return 'Reset', 'Reset'

                if self.state == 'backtrack':
                    if len(self.current_path) > 0:
                        heading, movement = self.heading, -1
                        move = self.current_path.pop()
                        self.heading = move[0]
                        self.update_location(movement)
                        return self.get_rotation_from_heading(heading) * -1, movement
                    else:
                        self.state = "explore"
                        return 0, 0
                else:
                    self.map(sensors)

                    possible_moves = self.get_possible_moves(sensors)
                    #print "possible moves: %s" % (possible_moves,)

                    if len(possible_moves) > 1:
                        self.save_node()
                        heading, movement = self.choose_move(possible_moves)
                        self.current_path.append([heading, movement])
                        return self.make_move(heading, movement)
                    if len(possible_moves) == 1:
                        heading, movement = [possible_moves[0][0], 1]
                        self.current_path.append([heading, movement])
                        return self.make_move(heading, movement)
                    if len(possible_moves) == 0:
                        self.state = "backtrack"
                        return 0, 0

            else:
                if self.goal_reached():
                    print "self.first_run_finished is: %s" % (self.first_run_finished,)
                    print "Goal reached in move %s" % (self.counter,)
                    self.location = [0, 0]
                    self.heading = 0
                    return 'Reset', 'Reset'

                else:
                    if self.maze_analyzed_for_astar == True:
                        print "moves: %s" % (self.astar_moves)
                        heading, movement = self.astar_moves.pop(0)
                        print "location: %s, heading: %s, movement: %s" % (self.location, heading, movement)
                        return self.make_move(heading, movement)
                    else:
                        self.analyze_edges()
                        self.add_reverse_edges()
                        self.add_missing_edges() 
                        self.astar_moves = self.get_moves_from_node_path(self.astar())
                        self.maze_analyzed_for_astar = True

                        heading, movement = self.astar_moves.pop(0)
                        return self.make_move(heading, movement)

    def get_moves_from_node_path(self, node_path):
        moves = []
        for i in range(len(node_path) - 1):
            from_node = node_path[i]
            to_node = node_path[i+1]
            moves += self.get_cheapest_edge(from_node, to_node)['path']
        return self.shorten_path(copy.deepcopy(moves))

    def add_missing_edges(self):
        # I found out during testing that my exploration method does not
        # find edges between neighbouring nodes when they have been visited before
        # through other paths. They are added here to ensure A* can find the
        # shortest paths
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                if self.is_node(i, j):
                    for n in self.get_reachable_neighbours(i, j):
                        if self.is_node(n[0], n[1]):
                            new_edge = {
                                "from": "{:d}-{:d}".format(i, j),
                                "to": "{:d}-{:d}".format(n[0], n[1]),
                                "path": [self.get_move_from_to([i,j], [n[0], n[1]])],
                                "cost": 1
                            }
                            self.append_edge(new_edge)

    def is_node(self, x, y):
        return len(self.maze[x,y]['doors']) >= 3

    def get_reachable_neighbours(self, x, y):
        doors = self.maze[x,y]['doors']
        neighbours = []
        if doors[0]: # up
            if y < self.maze_dim-2:
                neighbours.append([x, y+1])
        if doors[1]: # right
            if x < self.maze_dim-2:
                neighbours.append([x+1, y])
        if doors[2]: # down
            if y > 0:
                neighbours.append([x, y-1])
        if doors[3]: # left
            if x > 0:
                neighbours.append([x-1, y])
        return neighbours

    def get_move_from_to(self, a, b):
        if (math.fabs(a[0]-b[0]) + math.fabs(a[1]-b[1])) > 1:
            sys.exit("distance too far between %s and %s" % (a, b))

        if a[0] == b[0]:
            if a[1] < b[1]: 
                return [0, 1] # up
            else:
                return [2, 1] # down
        if a[1] == b[1]:
            if a[0] < b[0]:
                return [1, 1] # right
            else:
                return [3, 1] # left


    def add_reverse_edges(self):
        for edge in self.edges:
            # any edge in the opposite direction and of equal length counts as
            # reverse edge
            reverse_edges = [e for e in self.edges if edge['from'] == e['to']
                                                  and edge['to'] == e['from']
                                                  and len(edge['path']) == len(e['path'])]
            if len(reverse_edges) == 0:
                self.add_reverse_edge(edge)

    def add_reverse_edge(self, edge):
        new_edge = {'from': edge['to'], 'to': edge['from'], 'cost': edge['cost']}
        new_path = copy.deepcopy(edge['path'])
        new_path.reverse()
        new_path = [self.get_reverse_move(move) for move in new_path]
        new_edge['path'] = new_path
        self.append_edge(new_edge)

    def get_reverse_move(self, move):
        new_heading = (move[0] + 2) % 4
        return [new_heading, move[1]]

    def analyze_edges(self):
        for edge in self.edges:
            short_path = self.shorten_path(copy.deepcopy(edge['path']))
            edge['cost'] = len(short_path)

    def shorten_path(self, path):
        short_path = [] 
        for step in path:
            if len(short_path) is 0:
                short_path.append(step)
            else:
                prev = short_path.pop()
                if prev[0] == step[0] and prev[1] + step[1] <= 3:
                    prev[1] += step[1]
                    short_path.append(prev)
                else:
                    short_path.append(prev)
                    short_path.append(step)
        return short_path

    def astar(self):
        open_list = []
        closed_list = []
        predecessors = {}

        real_costs = {}
        estimated_costs = {}

        found = False
        current_node = "0-0"

        real_costs[current_node] = 0
        estimated_costs[current_node] = real_costs[current_node] + self.get_distance_to_goal(current_node)

        while found is False:
            # get a list of possible successor nodes from known edges
            nodes_to_check = self.get_successors(current_node)
            for next_node in nodes_to_check:
                if next_node == current_node:
                    continue
                if next_node in closed_list or next_node in open_list:
                    continue
                else:
                    edge_cost = self.get_edge_cost(current_node, next_node)
                    real_costs[next_node] = real_costs[current_node] + edge_cost

                    distance = self.get_distance_to_goal(next_node)
                    estimated_costs[next_node] = real_costs[next_node] + distance

                    predecessors[next_node] = current_node
                    open_list.append(next_node)

            closed_list.append(current_node)

            if len(open_list) is 0:
                break   # no solution
            open_list = sorted(open_list, key=lambda node: estimated_costs[node])
            current_node = open_list.pop(0)
            if self.is_in_goal(current_node):
                found = True

        if not found:
            sys.exit("no solution found")

        else:
            reverse_node_path = [current_node]
            while current_node != "0-0":
                current_node = predecessors[current_node]
                reverse_node_path.append(current_node)

            result = copy.copy(reverse_node_path)
            result.reverse()
            return result

    def is_in_goal(self, node):
        x, y = node.split('-')
        x = int(x)
        y = int(y)
        return x in self.goal_bounds and y in self.goal_bounds


    def get_distance_to_goal(self, node):
        x, y = node.split('-')
        x = int(x)
        y = int(y)
        distance = int(math.fabs(x - self.maze_dim/2) + math.fabs(y - self.maze_dim/2))
        return distance

    def get_edge_cost(self, a, b):
        return self.get_cheapest_edge(a, b)['cost']

    def get_cheapest_edge(self, a, b):
        edges = [edge for edge in self.edges if edge['from'] == a and edge['to'] == b]
        result = sorted(edges, key=lambda edge: edge['cost'])[0]
        # print "***"
        # print "edges from %s to %s: %s" % (a, b, edges)
        # print "returning %s as cheapest" % (result,)
        return result

    def get_successors(self, node):
        result = [edge['to'] for edge in self.edges if edge['from'] == node]
        #print "successors: %s " % (result,)
        return result

    def maze_fully_explored(self):
        result = True
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                if self.maze[i][j] is None:
                    result = False
                    break
        return result

    def goal_reached(self):
        x = self.location[0]
        y = self.location[1]

        return x in self.goal_bounds and y in self.goal_bounds

    def map(self, sensors):
        # this only works when we're exploring and moving forwards
        if self.last_move_was == 'backwards':
            return
        me = self.maze[self.location[0], self.location[1]] or {"visited": 0}

        doors = [False, False, False, False]
        if self.location[0] is not 0 or self.location[1] is not 0:
            # we are moving forward in explore mode, so where we came from is a door.
            doors[(self.heading + 2) % 4] = True
        for i in range(3):
            if sensors[i] > 0:
                doors[(self.heading + 4 + i -1) % 4] = True

        me['doors']   = doors
        me['visited'] += 1

        self.maze[self.location[0], self.location[1]] = me

    def choose_random_move(self, possible_moves):
        move = random.choice(possible_moves)
        return move[0], 1

    def choose_move(self, possible_moves):
        visited = []
        for move in possible_moves:
            cand = self.get_next_location(move[0], 1)
            #print "evaluating candidate: %s" % (cand,)
            if self.maze[cand[0], cand[1]]:
                visited.append(self.maze[cand[0], cand[1]]['visited'])
            else:
                visited.append(0)
        
        min_index = np.argmin(visited)

        return [possible_moves[min_index][0], 1]

    def get_next_location(self, heading, movement):
        result = 0
        if heading == 0:
            result = [self.location[0], self.location[1] + movement]
        if heading == 1:
            result = [self.location[0] + movement, self.location[1]]
        if heading == 2:
            result = [self.location[0], self.location[1] - movement]
        if heading == 3:
            result = [self.location[0] - movement, self.location[1]]

        #print "next location for %s, %s, %s: " % (self.location, heading, movement)
        #print "result: %s" % (result,)
        return result

    def make_move(self, heading, movement):
        rotation = self.get_rotation_from_heading(heading)
        self.heading = heading
        self.update_location(movement)
        #print "move %s, %s" % (rotation, movement)
        #print "new location %s, new heading %s" % (self.location, self.heading_map[self.heading])

        return rotation*90, movement

    def get_possible_moves(self, sensors):
        possible_moves = []
        for i in range(3):
            if sensors[i] != 0:
                possible_moves.append([(self.heading + 4 + i - 1) % 4, sensors[i]])
        return possible_moves

    def update_heading(self, rotation):
        self.heading += rotation
        if self.heading == 4:
            self.heading = 0
        if self.heading == -1:
            self.heading = 3

    def get_rotation_from_heading(self, heading):
        rotation = (heading - self.heading)
        if rotation == 3:
            rotation = -1
        if rotation == -3:
            rotation = 1
        #print "new heading: %s, self.heading: %s, rotation: %s" % (heading, self.heading, rotation)
        if rotation not in [-1, 0, 1]:
            sys.exit("impossible rotation: %s" % (rotation,))
        
        return rotation

    def update_location(self, movement):
        if self.heading == 0:
            self.location[1] +=  movement
        if self.heading == 1:
            self.location[0] += movement
        if self.heading == 2:
            self.location[1] -= movement
        if self.heading == 3:
            self.location[0] -= movement

        if movement < 0:
            self.last_move_was = 'backward'
        if movement > 0:
            self.last_move_was = 'forward'

    def save_node(self):
        node_name = str(self.location[0]) + "-" + str(self.location[1])
        previous_path = self.current_path
        edge = {"from": self.previous_node, "path": previous_path, "to": node_name}
        self.append_edge(edge)
        self.previous_node = node_name
        self.current_path = []

    def append_edge(self, edge):
        if edge['from'] == edge['to']:
            return
        if edge not in self.edges:
            self.edges.append(edge)


