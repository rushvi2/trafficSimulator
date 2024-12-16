
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.geometry.segment import Segment
from trafficSimulator.core.geometry.segment import Segment
from trafficSimulator.core.geometry.quadratic_curve import QuadraticCurve
from trafficSimulator.core.geometry.cubic_curve import CubicCurve

from trafficSimulator.core.vehicle import Vehicle
from trafficSimulator.core.vehicle_generator import VehicleGenerator

from trafficSimulator.core.simulation import Simulation
from trafficSimulator.visualizer.window import Window

from trafficSimulator.core.traffic_light import TrafficLight
from trafficSimulator.qlearning.q_agent import QLearningAgent

sim = Simulation(max_duration=200, use_q_learning=True)

lane_space = 3.5
intersection_size = 12
length = 100

# Intersection in
sim.create_segment((lane_space/2, length+intersection_size/2), (lane_space/2, intersection_size/2))
sim.create_segment((length+intersection_size/2, -lane_space/2), (intersection_size/2, -lane_space/2))
sim.create_segment((-lane_space/2, -length-intersection_size/2), (-lane_space/2, -intersection_size/2))
sim.create_segment((-length-intersection_size/2, lane_space/2), (-intersection_size/2, lane_space/2))
# Intersection out
sim.create_segment((-lane_space/2, intersection_size/2), (-lane_space/2, length+intersection_size/2))
sim.create_segment((intersection_size/2, lane_space/2), (length+intersection_size/2, lane_space/2))
sim.create_segment((lane_space/2, -intersection_size/2), (lane_space/2, -length-intersection_size/2))
sim.create_segment((-intersection_size/2, -lane_space/2), (-length-intersection_size/2, -lane_space/2))
# Straight
sim.create_segment((lane_space/2, intersection_size/2), (lane_space/2, -intersection_size/2))
sim.create_segment((intersection_size/2, -lane_space/2), (-intersection_size/2, -lane_space/2))
sim.create_segment((-lane_space/2, -intersection_size/2), (-lane_space/2, intersection_size/2))
sim.create_segment((-intersection_size/2, lane_space/2), (intersection_size/2, lane_space/2))
# Right turn
sim.create_quadratic_bezier_curve((lane_space/2, intersection_size/2), (lane_space/2, lane_space/2), (intersection_size/2, lane_space/2))
sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2), (lane_space/2, -lane_space/2), (lane_space/2, -intersection_size/2))
sim.create_quadratic_bezier_curve((-lane_space/2, -intersection_size/2), (-lane_space/2, -lane_space/2), (-intersection_size/2, -lane_space/2))
sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2), (-lane_space/2, lane_space/2), (-lane_space/2, intersection_size/2))
# Left turn
sim.create_quadratic_bezier_curve((lane_space/2, intersection_size/2), (lane_space/2, -lane_space/2), (-intersection_size/2, -lane_space/2))
sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2), (-lane_space/2, -lane_space/2), (-lane_space/2, intersection_size/2))
sim.create_quadratic_bezier_curve((-lane_space/2, -intersection_size/2), (-lane_space/2, lane_space/2), (intersection_size/2, lane_space/2))
sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2), (lane_space/2, lane_space/2), (lane_space/2, -intersection_size/2))

traffic_light_1 = TrafficLight(red_duration=10, green_duration=10, initial_state="red")
traffic_light_2 = TrafficLight(red_duration=10, green_duration=10, initial_state="red")
traffic_light_3 = TrafficLight(red_duration=10, green_duration=10, initial_state="green")
traffic_light_4 = TrafficLight(red_duration=10, green_duration=10, initial_state="green")

sim.add_traffic_light(traffic_light_1, 0)  # southbound segment
sim.add_traffic_light(traffic_light_2, 2)  # northbound segment
sim.add_traffic_light(traffic_light_3, 1)  # eastbound segment
sim.add_traffic_light(traffic_light_4, 3)  # westbound segment

# Straight Segments:
# 0, 1, 2, 3 for incoming (south in, east in, north in, west in)
# 4, 5, 6, 7 for outgoing (south out, east out, north out, west out)
# 8, 9, 10, 11 for intersections (south to north, east to West, north to south, west to east)
# Right Turn Segments:
# 12 s to w
# 13 e to n
# 14 n to e
# 15 w to s
# Left Turn Segments:
# 16 s to e
# 17 e to s
# 18 n to w
# 19 w to n
vg = VehicleGenerator({
    'vehicles': [
        # straight
        (2, {'path': [0, 8, 6], 'v': 16.6}),  # s to n
        (2, {'path': [1, 9, 7], 'v': 16.6}),  # e to w
        (2, {'path': [2, 10, 4], 'v': 16.6}), # n to s
        (2, {'path': [3, 11, 5], 'v': 16.6}), # w to e
        # right turn
        (2, {'path': [0, 12, 5], 'v': 16.6}), # s to e
        (2, {'path': [1, 13, 6], 'v': 16.6}), # e to n
        (2, {'path': [2, 14, 7], 'v': 16.6}), # n to w
        (2, {'path': [3, 15, 4], 'v': 16.6}), # w to s
        # left turn
        (2, {'path': [0, 16, 7], 'v': 16.6}), # s to w
        (2, {'path': [1, 17, 4], 'v': 16.6}), # e to s
        (2, {'path': [2, 18, 5], 'v': 16.6}), # n to e
        (2, {'path': [3, 19, 6], 'v': 16.6})  # w to n
    ]
})
sim.add_vehicle_generator(vg)
sim.set_max_duration(100)

# Run simulation
win = Window(sim)
win.run()
win.show()
