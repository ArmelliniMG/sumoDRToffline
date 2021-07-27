# (C) 2021 Armellini
# This code is licensed under MIT license (see LICENSE for details)

# @file    drtOffline.py
# @author  Maria Giuliana Armellini
# @date    2020-01-01

# The algorithm solves an offline DARP (Dial a Ride Problem) for multiple
# vehicles and requests. Multiple variations on the problem depending on the
# inputs are possible, for example: time windows and DRT as public transport
# feeder. The algorithm solves as default the optimum solution, but a maximal
# running time can be given and the solution will be maybe not the best
# possible.

# The algorithm was inspired by the DARP solve method from
# https://www.researchgate.net/publication/312049744_On-demand_high-capacity_ride-sharing_via_dynamic_trip-vehicle_assignment

from argparse import ArgumentParser
import time
from collections import namedtuple
import sys
import os
import psutil
import rvGraph  # Imports functions to calculate the request-vehicles Graph
import rtvGraph  # Imports functions to calculate request-trips-vehicles Graph
import lpSolver  # Import functions to solve the integer linear programming

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


class Request:
    def __init__(self, ID, name, depart, orig, dst, pax, pax_wc, orig_pos,
                 dst_pos, orig_window, dst_window, drf):
        self.ID = ID  # request ID
        self.name = name  # request ID
        self.depart = depart  # desired depart time
        self.orig_edge = orig  # origin edge
        self.dest_edge = dst  # destination edge
        try:
            self.orig_pos = float(orig_pos)  # origin position
        except ValueError:
            self.orig_pos = 5
        try:
            self.dest_pos = float(dst_pos)  # destination position
        except ValueError:
            self.dest_pos = 5
        try:
            # origin time window
            self.orig_window = [int(orig_window.split(",")[0]),
                                int(orig_window.split(",")[1])]
        except ValueError:
            self.orig_window = None
        try:
            # destination time window
            self.dest_window = [int(dst_window.split(",")[0]),
                                int(dst_window.split(",")[1])]
        except ValueError:
            self.dest_window = None
        try:
            self.drf = float(drf)  # direct route factor
        except ValueError:
            self.drf = 2
        try:
            self.pax = int(pax)  # number of passengers
        except ValueError:
            self.pax = 1
        try:
            self.pax_wc = int(pax_wc)  # number of wheelchair passengers
        except ValueError:
            self.pax_wc = 0

        # value assigned later
        self.rejected = None  # if request must be rejected
        self.rejected_cause = None  # caused of reject
        self.direct_route = None  # direct route duration
        self.drt_area = []  # request in DRT service area
        self.drt_leg = []  # if DRT only for first/last mile
        # public transport trip information [(line, from stop, to stop, time),]
        self.pt = []
        self.cost = 1  # requests with pt connection have priority
        self.value = 1
        self.waiting = 0
        self.arrival = 0
        self.trips = []  # list with potential trips

        # self.check_tt = [0,0] # attribute for checking travel time

        # not used variables:
        # self.delta = delta  # latest depart time allowed
        # self.walk = None  # walk from/to edge
        # self.start = None  # trip start time
        # self.pick_time = None  # DRT pick-up time
        # self.del_time = None  # DRT delivery time
        # self.vehicle = None  # assigned vehicle


class Vehicle:
    def __init__(self, ID, cost, cap, wc, depot, depot_pos, area, sumo_type,
                 start_time, end_time):
        self.ID = ID  # name of the vehicle
        self.cost = cost  # cost of using this vehicle
        self.cap = cap  # maximal passenger capacity
        self.cap_wc = wc  # maximal wheelchair passenger capacity
        self.depot = depot  # depot edge
        self.depot_pos = depot_pos  # depot position on edge
        if area:  # service area
            self.area = area
        else:
            self.area = None
        if sumo_type:  # sumo vehicle class
            self.type = sumo_type
        else:
            self.type = "taxi"
        if start_time:  # earliest depart
            self.start_time = int(start_time)
        else:
            self.start_time = 0
        if end_time:  # latest arrival
            self.end_time = int(end_time)
        else:
            self.end_time = 86400

        # value assigned later
        self.trip = None  # assigned trip
        self.depart = None  # assigned departure time
        self.pax = None  # passengers id
        self.idle_time = 0
        self.trip_duration = 0
        self.waiting_time = 0
        self.max_pax = 0

        # not used variables:
        # self.current_passengers = 0  # current number of passengers
        # self.current_wc = 0  # current number of wheelchair passengers
        # self.route = ""  # assigned edges


def initOptions():
    ap = ArgumentParser()
    ap.add_argument("-n", "--network", metavar="FILE", required=True,
                    help="SUMO network file")
    ap.add_argument("-r", "--reservations", metavar="FILE", required=True,
                    help="File with reservations (persons)")
    ap.add_argument("-v", "--taxis", metavar="FILE", required=True,
                    help="File with drt vehicles")
    ap.add_argument("-a", "--service-area", metavar="FILE",
                    help="File with edges for each service area")
    ap.add_argument("-s", "--pt-stops", metavar="FILE",
                    help="File with public transport stops")
    ap.add_argument("--drf-min", type=int, default=600,
                    help="minimum extra time for the trip in comparision with car")  # noqa

    ap.add_argument("-d", "--additional", type=str,
                    help="Additional files for simulation")
    # ap.add_argument("--depot", dest="depot", help="depot file", metavar="FILE")  # noqa
    # ap.add_argument("--depot-time", type=float, default=1800,
    #                 help="time without requests after vehicle go to depot (default 1800sec)")  # noqa
    # ap.add_argument("--depot-minstay", type=float, default=3600,
    #                 help="min waiting time at depot for rerouting (default 3600sec)")  # noqa
    ap.add_argument("--stop-length", type=int, required=True,
                    help="time in seconds a vehicle needs to stop for pick up or deliver a passenger")  # noqa
    # ap.add_argument("--fix-route-time", type=int, default=600,
    #                 help="maximal time (in sec) before pick up for route changes")  # noqa
    ap.add_argument("--c_ko", dest="c_ko", type=float, default=1000000000000,
                    help="Cost of ignoring a reservation")
    ap.add_argument("--cost-per-trip", type=float, default=600,
                    help="avoid using multiple vehicles if trip time is similar")  # noqa
    ap.add_argument("--search-fleet", action='store_true',
                    help="Searches the minimum number of taxis to serve the demand")  # noqa
    ap.add_argument("--optimum", action='store_true',
                    help="Searches the optimum")
    ap.add_argument("--max-memory", type=float, default=3000,
                    help="max memory capacity (default: 3 GB)")
    ap.add_argument("--max-time", type=float, default=5,
                    help="max searching time (default: 5 sec)")
    ap.add_argument("--DRT", type=str, default="DRT", help="DRT-Service name")
    ap.add_argument("-o", dest="path", type=str, default="",
                    help="Path for output files")
    # ap.add_argument("--parking", type=str, default="true",
    #                 help="Taxis park at stops")
    ap.add_argument("--sumocfg", type=str, default="true",
                    help="Write a SUMO configuration file for further simulations")  # noqa
    ap.add_argument("--maxwalk", type=int, default=240,
                    help="Maximum walking time")
    ap.add_argument("--maxwait", type=int, default=360,
                    help="maximum pt-waiting time")
    ap.add_argument("--minwait", type=int, default=60,
                    help="minimum pt-waiting time")
    ap.add_argument("--veh-wait", type=int, default=180,
                    help="maximum waiting time for passenger in the vehicle")
    ap.add_argument("--ride-hailing", type=bool, default=True,
                    help="search ride hailing when not optimum")
    ap.add_argument("--keep-trips", type=int, default=300,
                    help="consider options if differences larger than")
    ap.add_argument("--verbose", action='store_true')
    return ap


def get_delimiter(file_line):
    if ";" in file_line:
        delimiter = ";"
    elif "\t" in file_line:
        delimiter = "\t"
    else:
        print("Please use comma or tab delimited format for input file")
        sys.exit()

    return delimiter


def read_requests(options):
    # reads requests, creates class instance and returns requests list
    with open(options.reservations, "r") as requests_file:
        requests_lines = requests_file.read().splitlines()
        delimiter = get_delimiter(requests_lines[0])

        requests = [request.split(delimiter) for request in requests_lines[1:]]

    # creates Request Class
    requests = [Request(int(index),  # ID
                        str(request[0]), int(request[1]),  # name, depart
                        str(request[2]), str(request[3]),  # orig, dest
                        int(request[4]), int(request[5]),  # pax, pax_wc
                        str(request[6]), str(request[7]),  # orig_pos, dest_pos
                        str(request[8]), str(request[9]),  # orig, dest_window
                        str(request[10])  # drf
                        ) for index, request in enumerate(requests)]
    return requests


def read_vehicles(options):
    # reads vehicles, creates class instances and returns vehicles list
    with open(options.taxis, "r") as vehicles_file:
        vehicles_lines = vehicles_file.read().splitlines()
        delimiter = get_delimiter(vehicles_lines[0])

        vehicles = [vehicle.split(delimiter) for vehicle in vehicles_lines[1:]]

    # creates Request Class
    vehicles = [Vehicle(str(vehicle[0]), int(vehicle[1]),  # ID, cost
                        int(vehicle[2]), int(vehicle[3]),  # cap, wc
                        str(vehicle[4]), float(vehicle[5]),  # depot, depot_pos
                        str(vehicle[6]), str(vehicle[7]),  # area, sumo_type
                        vehicle[8], vehicle[9]  # start_time, end_time
                        ) for vehicle in vehicles]
    return vehicles


def offline_case(options, requests, vehicles, service_areas, pt_stops):
    # runs offline case

    if len(requests) < 1:
        print("no Requests were loaded")
        sys.exit()

    print("\nNumber of requests:", len(requests))
    print("Start RV-Graph")
    RV_graph, RD_dic = rvGraph.RV_offline(options, requests, vehicles,
                                          service_areas, pt_stops)

    # for debugging only:
    if options.verbose:
        with open("%s%s_RV_graph_debugging.txt" % (options.path, options.DRT),
                  "w+") as debug_file:
            for pair in RV_graph.keys():
                debug_file.write("%s: %s\n" % (pair, RV_graph[pair]))

    print("Start RTV-Graph")
    RTV_graph, memory_problems = rtvGraph.RTV(options, RV_graph, vehicles,
                                              requests, RD_dic, service_areas)

    # for debugging only:
    if options.verbose:
        with open("%s%s_RTV_graph_debugging.txt" % (options.path, options.DRT),
                  "w") as debug_file:
            for trip in RTV_graph:
                debug_file.write("%s: %s\n" % (trip.trip_id, trip))

    print("Start LP Solver")
    lpSolver.run_LP(options, vehicles, requests, RTV_graph, memory_problems,
                    RV_graph, RD_dic, Vehicle)

    if options.sumocfg == "true":  # write sumocfg file:
        options.additional = options.additional.replace('.dummy', '')
        with open("%s%s_Simulation.sumocfg" % (options.path, options.DRT),
                  "w") as sumocfg:
            sumocfg.write('<?xml version="1.0" encoding="UTF-8"?>\n\n')
            sumocfg.write('<configuration>\n\n')
            sumocfg.write('\t<input>\n')
            sumocfg.write('\t\t<net-file value="%s"/>\n' % options.network)
            sumocfg.write('\t\t<route-files value="%s%s_Persons_routes.xml, %s%s_Vehicles_routes.xml"/>\n'  # noqa
                          % (options.path, options.DRT,
                             options.path, options.DRT))
            if options.additional:
                sumocfg.write('\t\t<additional-files value="%s"/>\n' % options.additional)  # noqa
            sumocfg.write('\t</input>\n')
            sumocfg.write('\t<time>\n')
            depart_times = [request.depart for request in requests
                            if request.depart is not None
                            and request.rejected is not True]
            depart_times.extend([vehicle.depart for vehicle in vehicles
                                 if vehicle.depart is not None])
            if pt_stops:
                begin = min([x.time for x in pt_stops]) - 100
            else:
                begin = 0
            sumocfg.write('\t\t<begin value="%d"/>\n' % max(min(min(depart_times)-300, begin), 0))  # noqa
            sumocfg.write('\t</time>\n')
            sumocfg.write('</configuration>\n')


def main():
    start_code_time = time.perf_counter()
    process = psutil.Process(os.getpid())

    # read inputs
    argParser = initOptions()
    options = argParser.parse_args()
    requests = read_requests(options)
    vehicles = read_vehicles(options)

    try:
        with open(options.service_area, "r") as service_area_file:
            service_area_lines = service_area_file.read().splitlines()
            delimiter = get_delimiter(service_area_lines[0])
            service_areas = []
            service_areas_element = namedtuple('Area', ['name', 'edges'])

            for area in service_area_lines[1:]:
                service_areas.append(service_areas_element(area.split(delimiter)[0], area.split(delimiter)[1:]))  # noqa
    except TypeError:
        service_areas = None

    try:
        with open(options.pt_stops, "r") as pt_stops_file:
            pt_stops_lines = pt_stops_file.read().splitlines()
            delimiter = get_delimiter(pt_stops_lines[0])
            pt_stops = []
            pt_stops_element = namedtuple('pt_stop', ['time', 'vehicle',
                                                      'line', 'stop', 'edge'])

            for stop in pt_stops_lines[1:]:
                stop_values = stop.split(delimiter)
                stop_values[0] = int(stop_values[0])
                pt_stops.append(pt_stops_element._make(stop_values))
    except TypeError:
        pt_stops = None

    # start
    offline_case(options, requests, vehicles, service_areas, pt_stops)

    print("Finished!")
    print("--- %0.1f sec and %0.1f MB ---" %
          (time.perf_counter() - start_code_time,
           process.memory_info().rss*0.000001))

    with open("%s%s_DRT_info.xml" %
              (options.path, options.DRT), "a") as summary_file:
        summary_file.write("""<!--Running_time(sec)="%0.1f" Running_memory(MB)="%0.1f"-->\n""" %  # noqa
                           (time.perf_counter() - start_code_time, process.memory_info().rss*0.000001))  # noqa


if __name__ == "__main__":
    main()
