import math

def lat_lon_to_rad(lat_lon_in_deg):
	(lat_deg, lon_deg) = lat_lon_in_deg
	return (math.radians(lat_deg), math.radians(lon_deg))

def clockwise_angle_distance(target, current):
	regular_distance = (target - current) % (2 * PI)
	if regular_distance <= PI: return regular_distance
	else: return regular_distance - (2 * PI)
	
def bearing_from_to(from_lat_lon, to_lat_lon):
	(from_lat, from_lon) = from_lat_lon
	(to_lat, to_lon) = to_lat_lon
	return math.atan2(
		math.sin(to_lon - from_lon) * math.cos(to_lat),
		math.cos(from_lat) * math.sin(to_lat) - math.sin(from_lat) * math.cos(to_lat) * math.cos(to_lon - from_lon)
	)

print(math.degrees(bearing_from_to(
	lat_lon_to_rad((29.716897, -95.410912)),
	lat_lon_to_rad((29.714922, -95.410879)),
)))

p1 = (29.716508, -95.409327)
(p1_lat, p1_lon) = p1

print([
math.degrees(bearing_from_to(
	lat_lon_to_rad(p1),
	lat_lon_to_rad((p1_lat + math.sin(ang) * 0.001, p1_lon + math.cos(ang) * 0.001))
))
for ang in [math.radians(x) for x in [90.0, 135.0, 180.0, 225.0, 270.0, 315.0, 0.0, 45.0, ]]
])

