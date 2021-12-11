from scipy.spatial import distance

from Simulator.Network import Parameter as para
from Simulator.Mobile_Charger.MobileCharger_Method import get_location, charging


class MobileCharger:
    def __init__(self, id, energy=None, e_move=None, start=para.depot, end=para.depot, velocity=None,
                 e_self_charge=None, capacity=None):
        self.id = id
        self.is_charging = False  # is true if mc stand and charge
        self.is_self_charging = False  # is true if mc is charged
        self.is_active = False
        self.is_stand = True
        self.duty = []

        self.start = start  # from location
        self.end = end  # to location
        self.current = start  # location now
        self.end_time = -1

        self.energy = energy  # energy now
        self.capacity = capacity  # capacity of mc
        self.e_move = e_move  # energy for moving
        self.e_self_charge = e_self_charge  # energy receive per second
        self.velocity = velocity  # velocity of mc
        self.on_duty = False

    def get_status(self):
        if not self.is_active:
            return "deactivated"
        if self.is_self_charging:
            return "self-charging..."
        if self.on_duty:
            if not self.is_stand:
                return "on duty, moving..."
            else:
                return "on duty, charging..."
        return "unknown"

    def update_location(self, func=get_location):
        self.current = func(self)
        self.energy -= self.e_move

    def charge(self, net=None, func=charging):
        func(self, net)

    def self_charge(self):
        self.energy = min(self.energy + self.e_self_charge, self.capacity)
        self.is_self_charging = True

    # def check_state(self):
    #     if distance.euclidean(self.current, self.end) < 1:
    #         self.is_stand = True
    #         self.current = self.end
    #     else:
    #         self.is_stand = False
    #     if distance.euclidean(para.depot, self.end) < 10 ** -3:
    #         self.is_self_charge = True
    #     else:
    #         self.is_self_charge = False

    def get_next_location(self, network, time_stem):
        if len(self.duty) == 0:
            next_location, charging_time = para.depot, 0
            self.on_duty = False
        else:
            next_location, charging_time = self.next_duty(network)
            self.duty.pop(0)
        self.start = self.current
        self.end = next_location
        moving_time = distance.euclidean(self.start, self.end) / self.velocity
        self.end_time = time_stem + moving_time + charging_time

    def next_duty(self, network):
        node = network.node[self.duty[0]]
        location = node.location
        moving_time = distance.euclidean(self.current, location) / self.velocity
        charging_time = 1 + (node.energy_max - node.energy + node.avg_energy * moving_time) / (
                    para.alpha / (para.beta ** 2) - node.avg_energy)
        return location, charging_time

    def run(self, network, time_stem, optimizer=None):
        if not self.on_duty:
            if self.energy < self.capacity:
                self.self_charge()
            else:
                self.is_self_charging = False
                if len(optimizer.duty_list) == 0:
                    self.is_active = False
                else:
                    self.duty = optimizer.duty_list[0]
                    optimizer.pop_duty()
                    print('\tMC#{} take duty'.format(self.id), self.duty)
                    self.on_duty = True
                    self.is_active = True
        else:
            if self.end_time > time_stem:
                if distance.euclidean(self.current, self.end) < 10 ** (-3):
                    self.is_stand = True
                    self.current = self.end
                    if self.end == para.depot:
                        self.self_charge()
                        self.on_duty = False
                    else:
                        self.charge(network)
                        self.is_charging = True
                else:
                    self.is_stand = False
                    self.update_location()
            else:
                self.get_next_location(network=network, time_stem=time_stem)
