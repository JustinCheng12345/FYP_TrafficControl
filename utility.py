def distance_to_floor(position, floor, dest):
    return ((floor if dest > floor else 17 - floor)+18 - position) % 18
