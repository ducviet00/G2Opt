import random

from scipy.spatial.distance import euclidean
from Simulator.Network.Package import Package
import Simulator.Network.Parameter as para


def uniform_com_func(net):
    for node in net.node:
        if node.id in net.target and random.random() <= node.prob and node.is_active:
            package = Package(package_size=net.package_size)
            node.send(net, package)
            # print(package.path)
    return True


def to_string(net):
    min_energy = 10 ** 10
    min_node = -1
    for node in net.node:
        if node.energy < min_energy:
            min_energy = node.energy
            min_node = node
    min_node.print_node()


def count_package_function(net):
    count = 0
    for target_id in net.target:
        package = Package(is_energy_info=True)
        net.node[target_id].send(net, package)
        if package.path[-1] == -1:
            count += 1
    return count


def partition_function(net):
    nb_partition = len(net.mc_list)
    for node in net.node:
        if (1000 - node.check_point[-1]["time"]) > 50:
            node.set_check_point(1000)
    sorted_list_node = sorted(net.node, key=lambda x: x.angle)
    ttl = 0  # Total traffic load = Sum of average energy consumption of each node
    for node in net.node:
        ttl += node.avg_energy
    rtl = ttl / nb_partition  # Regional traffic load
    k = 0
    temp_rtl = 0
    for i, node in enumerate(sorted_list_node):
        temp_rtl += node.avg_energy
        if temp_rtl > rtl and k + 1 < nb_partition:
            print("partition #{} with rtl={}".format(k, temp_rtl))
            k += 1
            temp_rtl = node.avg_energy
            net.partitioned_node.append([])
        net.partitioned_node[-1].append(node)
        node.label = k
    print("partition #{} with rtl={}".format(k, temp_rtl))
    for region in net.partitioned_node:
        distance_sum = 0
        for i in range(len(region) - 1):
            for j in range(i + 1, len(region)):
                distance_sum += euclidean(region[i].location, region[j].location)
        net.D_avg.append(distance_sum / (len(region) * (len(region) - 1) / 2))


def get_D_max(net):
    D_max = 0
    for node in net.node:
        for other_node in net.node:
            D_max = max(D_max, euclidean(node.location, other_node.location))
    return D_max


def get_ECR_max(net):
    ECR_max = 0
    for node in net.node:
        ECR_max = max(ECR_max, node.avg_energy)
    return ECR_max


def get_CN_max(net):
    CN_max = 0
    for mc in net.mc_list:
        for node in net.partitioned_node[mc.id]:
            CN = 0
            for other_node in net.partitioned_node[mc.id]:
                if other_node.id != node.id:
                    d = euclidean(other_node.location, node.location)
                    p = para.alpha / (d + para.beta) ** 2
                    if (p - other_node.avg_energy) > 0:
                        CN += 1
            node.critical_density = CN
            CN_max = max(CN_max, CN)
    return CN_max
