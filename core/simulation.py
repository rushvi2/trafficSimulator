from .vehicle_generator import VehicleGenerator
from .geometry.quadratic_curve import QuadraticCurve
from .geometry.cubic_curve import CubicCurve
from .geometry.segment import Segment
from .vehicle import Vehicle
# from FinalProject.qlearning.q_agent import QLearningAgent
from qlearning.q_agent import QLearningAgent

class Simulation:
    def __init__(self, max_duration=200, use_q_learning=False, state_size=4, action_size=2):
        self.segments = []
        self.vehicles = {}
        self.vehicle_generator = []
        self.traffic_signals = []
        self.t = 0.0
        self.frame_count = 0
        self.dt = 1 / 60
        self.traffic_lights = []
        self.max_duration = max_duration
        self.running = True
        self.use_q_learning = use_q_learning

        if self.use_q_learning:
            self.q_agent = QLearningAgent(state_size=state_size, action_size=action_size)

    def set_max_duration(self, duration):
        self.max_duration = duration

    def add_traffic_light(self, traffic_light, segment_index):
        self.traffic_lights.append((traffic_light, segment_index))

    def add_vehicle(self, veh):
        self.vehicles[veh.id] = veh
        if len(veh.path) > 0:
            self.segments[veh.path[0]].add_vehicle(veh)

    def add_segment(self, seg):
        self.segments.append(seg)

    def add_vehicle_generator(self, gen):
        self.vehicle_generator.append(gen)

    def create_vehicle(self, **kwargs):
        veh = Vehicle(kwargs)
        self.add_vehicle(veh)

    def create_segment(self, *args):
        seg = Segment(args)
        self.add_segment(seg)

    def create_quadratic_bezier_curve(self, start, control, end):
        cur = QuadraticCurve(start, control, end)
        self.add_segment(cur)

    def create_cubic_bezier_curve(self, start, control_1, control_2, end):
        cur = CubicCurve(start, control_1, control_2, end)
        self.add_segment(cur)

    def create_vehicle_generator(self, **kwargs):
        gen = VehicleGenerator(kwargs)
        self.add_vehicle_generator(gen)
    def run(self, steps):
        for _ in range(steps):
            if self.t < self.max_duration and self.running:
                self.update()
            else:
                break
    def encode_state(self):
        state = 0
        for i, (traffic_light, _) in enumerate(self.traffic_lights):
            state = state * 2 + (1 if traffic_light.is_red() else 0)  
        if self.use_q_learning:
            return min(state, self.q_agent.state_size - 1) 
        return state

    def calculate_reward(self):
        # Large penalty for unsafe lights,total wait time, reward reducing wait times, moving

        reward = 0

        for idx, (light1, segment_index1) in enumerate(self.traffic_lights):
            for jdx, (light2, segment_index2) in enumerate(self.traffic_lights):
                if idx != jdx and light1.is_green() and light2.is_green():
                    if self.cross_traffic(segment_index1, segment_index2):
                        reward -= 100 

        total_wait_time = sum(vehicle.wait_time for vehicle in self.vehicles.values())
        reward -= total_wait_time  

        moving_vehicles = sum(1 for vehicle in self.vehicles.values() if vehicle.v > 0)
        reward += moving_vehicles  
        return reward

    def cross_traffic(self, idx1, idx2):
        direct_conflicts = {(0, 2), (2, 0), (1, 3), (3, 1)}
        if (idx1, idx2) in direct_conflicts:
            return True
        left_turn_conflicts = {(0, 1), (1, 0)}  
        if (idx1, idx2) in left_turn_conflicts:
            return True

        return False


    def update(self):
        if self.use_q_learning:
            current_state = self.encode_state()
            action = self.q_agent.choose_action(current_state)
            self.set_traffic_phase(action)

        if self.use_q_learning:
            action = self.q_agent.choose_action(current_state)

        if not self.running:
            return

        for idx, (traffic_light, segment_index) in enumerate(self.traffic_lights):
            if self.use_q_learning:
                traffic_light.set_state(action == idx)
            traffic_light.update(self.dt)

        for segment_index, segment in enumerate(self.segments):
            if len(segment.vehicles) > 0:
                for i, vehicle_id in enumerate(segment.vehicles):
                    vehicle = self.vehicles[vehicle_id]
                    stop_for_light = False
                    for traffic_light, light_segment_index in self.traffic_lights:
                        if segment_index == light_segment_index and traffic_light.is_red():
                            intersection_distance = segment.get_length() - vehicle.x
                            stop_for_light = intersection_distance <= vehicle.s0

                    lead = self.vehicles[segment.vehicles[i - 1]] if i > 0 else None
                    vehicle.stopped = stop_for_light or (lead and vehicle.x + vehicle.l + vehicle.s0 >= lead.x)
                    vehicle.update(lead, self.dt)

        for segment in self.segments:
            if len(segment.vehicles) == 0:
                continue

            vehicle_id = segment.vehicles[0]
            vehicle = self.vehicles[vehicle_id]

            if vehicle.x >= segment.get_length():
                if vehicle.current_road_index + 1 < len(vehicle.path):
                    vehicle.current_road_index += 1
                    next_road_index = vehicle.path[vehicle.current_road_index]
                    if next_road_index < len(self.segments): 
                        self.segments[next_road_index].add_vehicle(vehicle)
                    else:
                        print(f"invalid road index")
                else:
                    print(f"vehicle at end of path.")
                vehicle.x = 0
                segment.vehicles.popleft()

        for gen in self.vehicle_generator:
            gen.update(self)

        self.t += self.dt
        self.frame_count += 1

        if self.use_q_learning:
            reward = self.calculate_reward()
            new_state = self.encode_state()
            self.q_agent.update_q_table(current_state, action, reward, new_state)

        if self.t >= self.max_duration:
            self.stop()

    def stop(self):
        self.running = False
        self.print_average_wait_time()

    def print_average_wait_time(self):
        total_wait_time = sum(vehicle.wait_time for vehicle in self.vehicles.values())
        number_of_vehicles = len(self.vehicles)
        if number_of_vehicles > 0:
            average_wait_time = total_wait_time / number_of_vehicles
            print(f"Total Average Wait Time: {average_wait_time:.2f} seconds")
        else:
            print("No vehicles passed through the simulation.")

    def set_traffic_phase(self, phase):
        if phase == 0:  # n-s green
            self.traffic_lights[0][0].set_state(True)   
            self.traffic_lights[1][0].set_state(True)
            self.traffic_lights[2][0].set_state(False)
            self.traffic_lights[3][0].set_state(False)
        elif phase == 1:  # e-w green
            self.traffic_lights[0][0].set_state(False)
            self.traffic_lights[1][0].set_state(False)
            self.traffic_lights[2][0].set_state(True)
            self.traffic_lights[3][0].set_state(True)
        elif phase == 2:  # left turn
            self.traffic_lights[4][0].set_state(True) 
            self.traffic_lights[5][0].set_state(True)
            for i in range(4):
                self.traffic_lights[i][0].set_state(False)
