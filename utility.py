def floor_to_block2(position, floor):
    possible_block = [floor, abs(floor - 17)]
    distance_to_block = [(t + 18 - position) % 18 for t in possible_block]
    min_distance = min(distance_to_block)
    return possible_block[distance_to_block.index(min_distance)], min_distance

def distance_to_block(position, floor):
    return min([(t + 18 - position) % 18 for t in [floor, abs(floor - 17)]])