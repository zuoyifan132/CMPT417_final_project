
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
		return all_intervals[0]

	# combine each interval with the result
	for i in range(2, len(all_intervals)):
		result = combine_two(result, all_intervals[i])

	return result


# calculate safe interval
def get_safe_interval(cfg, obstacles):
    each_safe_interval = []

    for path in obstacles:
        index = [] 		# list of index of obstacles that hits the cfg
        temp = []
        for i in range(len(path)):
            if path[i] == cfg:
                index.append(i)

        for i in range(len(index)):
            if index[i] != 0 and i == 0:
                temp.append((0, index[i]-1))

            if i != 0 and index[i-1]+1 != index[i]:
            	temp.append((index[i-1]+1, index[i]-1))

            if i == len(index)-1 and index[i] != len(path)-1:
                temp.append((index[i]+1, -1))

        if len(index) != 0:
        	each_safe_interval.append(temp)

    result_interval = combine_intervals(each_safe_interval)

    return result_interval


#all_intervals = [[(0,3), (8,-1)], [(0,0), (4,9), (12,-1)], [(1,7), (9,9), (13,15), (17,-1)]]



cfg = (4,4)
#cfg = (0,4)
#obstacles = [[(0,0), (0,1), (0,2), (0,3), (0,4), (0,5), (0,6), (0,5), (0,4), (0,3)]]		# answer (0,3), (5,7), (9,-1)
#obstacles = [[(0,4), (0,5), (0,6), (0,5), (0,4), (0,3)]]			# answer (1,3), (5,-1)
#obstacles = [[(0,0), (0,1), (0,2), (0,3), (0,4), (0,4), (0,4), (0,5), (0,6)]]		# answer (0,3), (7,-1)
#obstacles = [[(0,0), (0,1), (0,2), (0,3), (0,4), (0,5), (0,4)]]
#obstacles = [[(0,4), (0,4), (0,5), (0,4), (0,4), (0,3), (0,4)]]		# answer (2, 2), (5, 5)
obstacles = [[(8,4), (7,4), (6,4), (5,4), (4,4), (3,4), (2,4), (1,4)], [(4,6), (4,5), (4,4), (4,3), (4,2)]]
print(get_safe_interval(cfg, obstacles))


