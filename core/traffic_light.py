class TrafficLight:
    def __init__(self, red_duration, green_duration, initial_state="red", min_phase_duration=5):
        self.red_duration = red_duration
        self.green_duration = green_duration
        self.state = initial_state
        self.timer = 0
        self.phase_timer = 0
        self.min_phase_duration = min_phase_duration

    def update(self, dt):
        self.timer += dt
        self.phase_timer += dt

        if self.state == "red" and self.timer >= self.red_duration:
            if self.phase_timer >= self.min_phase_duration:  
                self.state = "green"
                self.timer = 0
                self.phase_timer = 0
        elif self.state == "green" and self.timer >= self.green_duration:
            if self.phase_timer >= self.min_phase_duration: 
                self.state = "red"
                self.timer = 0
                self.phase_timer = 0

    def is_red(self):
        return self.state == "red"

    def is_green(self):
        return self.state == "green"

    def set_state(self, is_green):
        if is_green and not self.is_green() and self.phase_timer >= self.min_phase_duration:
            self.state = "green"
            self.timer = 0
            self.phase_timer = 0
        elif not is_green and not self.is_red() and self.phase_timer >= self.min_phase_duration:
            self.state = "red"
            self.timer = 0
            self.phase_timer = 0
