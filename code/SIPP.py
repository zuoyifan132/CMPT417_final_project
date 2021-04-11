import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]     # add wait direction
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    constraint_table = dict()
    temp_constraints = []

    # deep copy constraints
    for constraint in constraints:
        temp_constraints.append(constraint)

    # expand positive constraints to negative constraints
    for constraint in temp_constraints:
        if constraint['positive'] == 1 and constraint['agent'] != agent:
            # vertex constraint:
            if len(constraint['loc']) == 1:
                temp_constraints.append({'agent':agent, 'loc':constraint['loc'], 'timestep':constraint['timestep'],
                                     'positive':0})
            # edge constraint:
            else:
                temp_constraints.append({'agent':agent, 'loc':[constraint['loc'][1], constraint['loc'][0]], 
                                        'timestep':constraint['timestep'], 'positive':0})

    for constraint in temp_constraints:
        # the constraint belong to the agent if the constraint is specific for 
        if constraint['agent'] == agent:
            time_step = -1
            if type(constraint['timestep']) == type(1):
                time_step = constraint['timestep']
            # new time step
            if time_step not in constraint_table.keys():
                constraint_table[time_step] = []
            constraint_table[time_step].append(constraint)

    return constraint_table          


def get_location(path, time):
	if len(path) == 0:
		raise BaseException('path emprt')
	if time < 0:
		return path[0]
	elif time < len(path):
		return path[time]
	else:
		return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    if next_time in constraint_table.keys():
        for same_time_constraint in constraint_table[next_time]:
            # check vertex constraint
            if len(same_time_constraint['loc']) == 1:
                # check negative constraint
                if same_time_constraint['positive'] == 0:
                    if same_time_constraint['loc'][0] == next_loc:
                        return False
                # check positive constraint
                else:
                    if same_time_constraint['loc'][0] == next_loc:
                        continue
                    else:
                        return False
            # check edge constraint
            else:
                # check negative constraint
                if same_time_constraint['positive'] == 0:
                    if same_time_constraint['loc'][0] == curr_loc and same_time_constraint['loc'][1] == next_loc:
                        return False
                # check positive constraint      
                else:
                    if same_time_constraint['loc'][0] == curr_loc and same_time_constraint['loc'][1] == next_loc:
                        continue
                    else:
                        return False

    # prohibit for all future time, it's only possible to be vertex constraint 
    if (-1) in constraint_table.keys():
        for same_time_constraint in constraint_table[-1]:
            if same_time_constraint['loc'][0] == next_loc and same_time_constraint['timestep'][1] <= next_time:
                return False

    return True


def push_node(open_list, node):
	#print("generate node:")
	#print(node)
	heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
	_, _, _, curr = heapq.heappop(open_list)
	#print("expand node:")
	#print(curr)
	return curr


def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


# combine two safe intervals
def combine_two(I1, I2):
    result = []

    for i in range(len(I1)):
        for j in range(len(I2)):
            # two interval doesn't have overlap, make sure -1 is infinite 
            if (I1[i][1] < I2[j][0] and I1[i][1] != -1) or (I1[i][0] > I2[j][1] and I2[j][1] != -1):
                continue
            # two interval have overlap
            else: 
                # I1[i] include I2[j]]
                if (I1[i][0] <= I2[j][0] and I1[i][1] >= I2[j][1] and I2[j][1] != -1) or\
                   (I1[i][0] <= I2[j][0] and I1[i][1] == -1 and I2[j][1] != -1) or\
                   (I1[i][0] <= I2[j][0] and I1[i][1] == -1 and I2[j][1] == -1):
                    if I2[j] not in result:
                        result.append(I2[j])
                # I2[j] include I1[i]
                elif (I2[j][0] <= I1[i][0] and I2[j][1] >= I1[i][1] and I1[i][1] != -1) or\
                     (I2[j][0] <= I1[i][0] and I2[j][1] == -1 and I1[i][1] != -1) or\
                     (I2[j][0] <= I1[i][0] and I2[j][1] == -1 and I1[i][1] == -1):
                    if I1[i] not in result:
                        result.append(I1[i])
                # overlap but not include
                else:
                    if (I2[j][1] >= I1[i][1] >= I2[j][0] and I1[i][1] != -1) or\
                       (I2[j][1] == -1 and I1[i][1] >= I2[j][0]):
                        if (I2[j][0], I1[i][1]) not in result:
                            result.append((I2[j][0], I1[i][1]))
                    elif (I1[i][1] >= I2[j][1] >= I1[i][0] and I2[j][1] != -1) or\
                         (I1[i][1] == -1 and I2[j][1] >= I1[i][0]):
                        if (I1[i][0], I2[j][1]) not in result:
                            result.append((I1[i][0], I2[j][1]))
    return result


# combine all the safe intervals
def combine_intervals(all_intervals):
	result = []

	# combine first two interval to get first result
	if len(all_intervals) >= 2:
		result = combine_two(all_intervals[0], all_intervals[1])
	else:
		if len(all_intervals) > 0:
			return all_intervals[0]

	# combine each interval with the result
	for i in range(2, len(all_intervals)):
		result = combine_two(result, all_intervals[i])

	return result


# calculate safe interval
def get_safe_interval(cfg, timestep, obstacles):
    each_safe_interval = []

    for path in obstacles:
        index = [] 		# list of index of obstacles that hits the cfg
        temp = []
        for i in range(len(path)):
            if path[i] == cfg:
                index.append(i)

        for i in range(len(index)):
        	# first meet point not no start loc, and time not past
            if index[i] != 0 and i == 0 and index[0] > timestep:
                temp.append((timestep, index[i]-1))

            # not the first meet point and obstacle not stay in meet point
            if i != 0 and index[i-1]+1 != index[i]:
            	# time not past 
            	if index[i-1]+1 <= timestep <= index[i]-1:
            		temp.append((timestep, index[i]-1))
            	elif timestep < index[i]:
            		temp.append((index[i-1]+1, index[i]-1))

            if i == len(index)-1 and index[i] != len(path)-1:
            	if timestep > index[i]+1:
            		temp.append((timestep, -1))
            	else:
            		temp.append((index[i]+1, -1))

        if len(index) != 0:
        	each_safe_interval.append(temp)

    result_interval = combine_intervals(each_safe_interval)
    result_interval.sort(key=lambda x:x[0])

    return result_interval


# set time limit
def set_time_limit(my_map):
    possible_agent_number = 0
    for row in my_map:
        for col in row:
            if col != '@':
                possible_agent_number += 1
    time_limit = (possible_agent_number-1)*possible_agent_number

    return time_limit


# earliest arrival time at cfg during interval i with no collisions
def find_earliest_arrival(safe_interval, curr_time, curr_loc, next_loc, obstacles):
	# the safe interval is already past
	if curr_time > safe_interval[1] and safe_interval[1] != -1:
		return None
	# need to wait then move
	wait_time = safe_interval[0] - curr_time - 1
	agent_path = [curr_loc]
	for j in range(wait_time):
		agent_path.append(curr_loc)
	agent_path.append(next_loc)

	for each_obstacle in obstacles:
		temp1 , temp2 = agent_path[:], each_obstacle[curr_time:]
		l1, l2 = len(each_obstacle), len(agent_path)

		if len(temp2) == 0:
			continue

		# detect edge collision
		for i in range(max(l1, l2)):
			if i < max(l1, l2)-1:
				if get_location(temp1, i) == get_location(temp2, i+1) and get_location(temp2, i) == get_location(temp1, i+1):
					#print("edge collision: ", get_location(temp1, i), get_location(temp1, i+1), get_location(temp2, i), get_location(temp2, i+1))
					return None

	return len(agent_path)-1


# expand node to get all valid successors
def get_successors(curr, my_map, h_values, obstacles):
	successors = []

	for dir in range(4):
		cfg = move(curr['loc'], dir)

		# set boudary constraint 
		if cfg[0] < 0 or cfg[1] < 0 or cfg[0] >= len(my_map) or cfg[1] >= len(my_map[0]): 
			continue
		# encounter a block
		if my_map[cfg[0]][cfg[1]]:		# the position is True meaning '@': invalid movement
			continue
		
		start_t = curr['timestep'] + 1		# the m_time is alwasy 1
		end_t = curr['safe_interval'][1]
		if curr['safe_interval'][1] != -1:
			end_t = curr['safe_interval'][1] + 1

        # all safe inervals in cfg
		cfg_safe_intervals = get_safe_interval(cfg, curr['timestep']+1, obstacles)

		for each_SI in cfg_safe_intervals:
			if (each_SI[0] > end_t and end_t != -1) or (each_SI[1] < start_t and each_SI[1] != -1):
				if curr['loc'] == (4,5) and curr['safe_interval'] == (4,-1):
					print("pruned by 1 ", each_SI, start_t, end_t)
				continue
        	# earliest arrival time at cfg during interval i with no collisions
			t = find_earliest_arrival(each_SI, curr['timestep'], curr['loc'], cfg, obstacles)
			if t is None:
				if curr['loc'] == (4,5) and curr['safe_interval'] == (4,-1):
					print("pruned by 2")
				continue

			successor = {'loc': cfg, 'g_val': curr['g_val']+t, 'h_val': h_values[cfg], 'parent': curr, 
			'timestep': curr['timestep']+t, 'safe_interval': each_SI}

			successors.append(successor)

	return successors



# safe interval A start search
def a_star_safe_interval(my_map, start_loc, goal_loc, h_values, obstacles):
    # open list and closed list
    open_list = []
    closed_list = dict()

    # the goal location is not connect to the start location: No solution!
    if start_loc not in h_values.keys():
        return None

    # initialize h value 
    h_value = h_values[start_loc]   
    
    max_time = 0
    if len(obstacles) > 0:
        max_time = max(len(i) for i in obstacles)

    # note root only choose the first interval of all save intervals
    root_safe_interval = get_safe_interval(start_loc, 0, obstacles)[0]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0, 'safe_interval': root_safe_interval}
    push_node(open_list, root)
    closed_list[(root['loc'], root_safe_interval)] = root

    # add time limit to avoid infinite loop
    time_limit = set_time_limit(my_map)

    while len(open_list) > 0 and time_limit > 0:
    	curr = pop_node(open_list)

    	# find solution
    	if curr['loc'] == goal_loc and curr['timestep'] >= max_time:
    		print("find solution:")
    		print()
    		print(curr)
    		return get_path(curr)

    	# expand curr node, get all successors
    	successors = get_successors(curr, my_map, h_values, obstacles)

    	for successor in successors:
    		# expand the old(in closed list) child if with smaller f-val    
    		if (successor['loc'], successor['safe_interval']) in closed_list:
    			existing_node = closed_list[(successor['loc'], successor['safe_interval'])]
    			if compare_nodes(successor, existing_node):
    				closed_list[(successor['loc'], successor['safe_interval'])] = successor
    				push_node(open_list, successor)

    		# expand the child if it isn't in closed list
    		else:
    			closed_list[(successor['loc'], successor['safe_interval'])] = successor
    			push_node(open_list, successor)

    	time_limit -= 1

    return None




my_map = [[1,1,1,1,0,1,1,1,1],
		  [1,1,1,1,0,1,1,1,1],
		  [1,1,1,1,0,1,1,1,1],
		  [1,1,1,1,0,1,1,1,1],
		  [0,0,0,0,0,0,0,0,0],
		  [1,1,1,1,0,1,1,1,1],
		  [1,1,1,1,0,1,1,1,1],
		  [1,1,1,1,0,1,1,1,1],
		  [1,1,1,1,0,1,1,1,1]]
h_values = compute_heuristics(my_map, (8,4))
curr = {'loc': (3,4), 'g_val': 0, 'h_val': 5, 'parent': None, 'timestep': 0, 'safe_interval': [0,4]}
obstacles = [[(8,4), (7,4), (6,4), (5,4), (4,4), (3,4), (2,4), (1,4)], [(4,6), (4,5), (4,4), (4,3), (4,2)]]
print(a_star_safe_interval(my_map, curr['loc'], (8,4), h_values, obstacles))

#expand node:{'loc': (2, 4), 'g_val': 1, 'h_val': 6, 'parent': {'loc': (3, 4), 'g_val': 0, 'h_val': 5, 'parent': None, 'timestep': 0, 'safe_interval': (0, 4)}, 'timestep': 1, 'safe_interval': (1, 5)}
















































