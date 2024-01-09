import math

def generate_circular_waypoints(radius, num_points, duration):
    waypoints = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        time = duration * i / num_points
        orientation = -angle  # Adjust the orientation calculation as needed
        waypoints.append((x, y, 0, 0, 0, orientation, time))
    return waypoints

radius = 1.0  # Adjust the radius of the circle
num_points = 100  # Adjust the number of points on the circle
duration = 10.0  # Adjust the total duration of the animation

waypoints = generate_circular_waypoints(radius, num_points, duration)

for waypoint in waypoints:
    print(f'''
<waypoint>
    <time>{waypoint[6]}</time>
    <pose>{waypoint[0]} {waypoint[1]} {waypoint[2]} {waypoint[3]} {waypoint[4]} {waypoint[5]}</pose>
</waypoint>
    ''')