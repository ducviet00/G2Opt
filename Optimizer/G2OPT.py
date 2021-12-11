import random
from operator import itemgetter

import numpy as np
from scipy.spatial import distance

from Simulator.Network import Parameter as para


class G2OPT:
    def __init__(self, network=None):
        # this parameter have to be changed
        self.E_max = network.node[0].energy_max  # maximum energy of sensor
        self.E_high = network.node[0].energy_max  # maximum energy of sensor
        self.E_min = 0  # minimum energy of sensor
        self.sensor = None  # location of sensor
        location = []
        for node in network.node:
            location.append(node.location)
        self.sensor = np.array(location)
        self.p = 0.9 * np.random.random(len(self.sensor)) + 0.1  # consumption energy
        self.e = self.E_max * np.ones(len(self.sensor))  # current energy of sensor
        self.E_M = network.mc_list[0].capacity  # capacity of mc
        self.q_c = network.mc_list[0].e_self_charge  # charging power of mc
        self.q_m = network.mc_list[0].e_move  # mobile power of mc
        self.velocity = network.mc_list[0].velocity  # velocity of mc
        self.duty_list = []
        # this parameter can be changed
        self.pop_size = 20  # the number of individual in population
        self.max_gen = 100  # the number of generation
        # this parameter is not changed
        self.t_l = 0  # lower bound of charging
        self.t_u = (self.e - self.E_min) / self.p  # upper bound of charging

    def fitness(self, all_path, t_arrive):
        travel = 0
        num_mc = len(all_path)
        critical = 0
        for path in all_path:
            for order_node in range(len(path)):
                if order_node == 0:
                    d = distance.euclidean(self.sensor[path[order_node]], para.depot)
                else:
                    d = distance.euclidean(self.sensor[path[order_node - 1]],
                                           self.sensor[path[order_node]])
                travel = travel + d
            travel = travel + distance.euclidean(self.sensor[path[-1]], para.depot)
        for id_sensor, _ in enumerate(t_arrive):
            if t_arrive[id_sensor] > self.t_u[id_sensor]:
                critical = critical + (t_arrive[id_sensor] - self.t_u[id_sensor])
        return travel/1000 + num_mc + critical/1000

    def get_path(self, gen):
        all_path = []
        id_gen = 0
        t_arrive = [-1 for _ in range(len(gen))]
        while id_gen < len(gen):
            path = []
            temp_e = self.e
            temp_mc = self.E_M
            empty_path = False
            current_time = 0
            while id_gen < len(gen):
                if len(path) == 0:
                    t_move = distance.euclidean(para.depot, self.sensor[gen[id_gen]]) / self.velocity
                else:
                    t_move = distance.euclidean(self.sensor[gen[id_gen - 1]],
                                                self.sensor[gen[id_gen]]) / self.velocity
                current_time = current_time + t_move
                temp_e = temp_e - t_move * self.p
                temp_mc = temp_mc - t_move * self.q_m
                e_charge = self.E_max - temp_e[gen[id_gen]]
                t_depot = distance.euclidean(para.depot, self.sensor[gen[id_gen]]) / self.velocity
                if temp_e[gen[id_gen]] > 0 and temp_mc - e_charge > t_depot * self.q_m:
                    path.append(gen[id_gen])
                    t_arrive[gen[id_gen]] = current_time
                    t_charge = e_charge / 0.04
                    current_time = current_time + t_charge
                    temp_e = temp_e - t_charge * self.p
                    temp_e[gen[id_gen]] = self.E_max
                    temp_mc = self.E_M - e_charge
                    id_gen = id_gen + 1
                else:
                    if len(path) == 0:
                        empty_path = True
                        break
                    else:
                        all_path.append(path)
                        break
            if empty_path:
                break
        return all_path, t_arrive

    def population(self):
        population = []
        for i in range(self.pop_size):
            gen = list(range(len(self.e)))
            random.shuffle(gen)
            all_path, t_arrive = self.get_path(gen)
            individual = {"gen": gen, "fitness": self.fitness(all_path, t_arrive), "path": all_path}
            population.append(individual)
        return population

    def pmx(self, gen1, gen2):
        off = []
        n = len(gen1)
        cut_a = random.randint(1, n - 1)
        cut_b = random.randint(1, n - 1)
        while cut_b == cut_a:
            cut_b = random.randint(1, n - 1)
        start = min(cut_a, cut_b)
        end = max(cut_a, cut_b)
        # print start, end
        temp = gen2[start:end]
        # print temp
        index = 0
        while index < start:
            for item in gen1:
                if item not in temp and item not in off:
                    off.append(item)
                    index = index + 1
                    break
        off.extend(temp)
        for item in gen2:
            if item not in off:
                off.append(item)
        # print "temp1 =", gen1, "temp2 =", gen2, "Off =", off
        return off

    def crossover(self, father1, father2):
        offspring = self.pmx(father1["gen"], father2["gen"])
        all_path, t_arrive = self.get_path(offspring)
        individual = {"gen": offspring, "fitness": self.fitness(all_path, t_arrive), "path": all_path}
        return individual

    def mutation(self, father):
        n = len(father["gen"])
        cut_a = random.randint(1, n - 1)
        cut_b = random.randint(1, n - 1)
        while cut_b == cut_a:
            cut_b = random.randint(1, n - 1)
        offspring = father["gen"][:]
        offspring[cut_a] = father["gen"][cut_b]
        offspring[cut_b] = father["gen"][cut_a]
        all_path, t_arrive = self.get_path(offspring)
        individual = {"gen": offspring, "fitness": self.fitness(all_path, t_arrive), "path": all_path}
        return individual

    def evolution(self, p_c=0.9, p_m=0.05):
        p = self.population()
        pop_size = len(p)
        ite = 0
        while ite < self.max_gen:
            temp_p = []
            for i, individual in enumerate(p):
                r_c = random.random()
                if r_c < p_c:
                    j = random.randint(0, len(p)-1)
                    while j == i:
                        j = random.randint(0, len(p)-1)
                    offspring = self.crossover(p[i], p[j])
                    temp_p.append(offspring)
                r_m = random.random()
                if r_m < p_m:
                    offspring = self.mutation(p[i])
                    temp_p.append(offspring)
            p.extend(temp_p)
            p = sorted(p, key=itemgetter("fitness"), reverse=False)
            p = p[:pop_size]
            print("ite =", ite, "fitness =", p[0]["fitness"], "nb mc =", len(p[0]["path"]))
            ite = ite + 1

        return p[0]

    def get_charging_route(self, network=None):
        self.update(network=network)    # def partition(self, partition_func=partition_function):
    #     partition_func(self)
    #     self.partitioned = True
        individual = self.evolution()
        self.duty_list = individual["path"]
        return individual["path"]

    def pop_duty(self):
        self.duty_list = self.duty_list[1:]

    def update(self, network):
        location = []
        e = []
        e_avg = []
        for node in network.node:
            location.append(node.location)
            e.append(node.energy)
            e_avg.append(node.avg_energy)
        self.e = np.array(e)
        self.p = np.array(e_avg)
        self.sensor = np.array(location) 