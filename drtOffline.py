# (C) 2021 Armellini
# This code is licensed under MIT license (see LICENSE for details)

# @file    drtOffline.py
# @author  Maria Giuliana Armellini
# @date    2020-01-01

# The algorithm solves an offline DARP (Dial a Ride Problem) for multiple 
# vehicles and requests. Multiple variations on the problem depending on the
# inputs are possible, for example: time windows and DRT as public transport feeder.
# The algorithm solves as default the optimum solution, but a maximal running time
# can be given and the solution will be maybe not the best possible.

# The algorithm was inspired by the DARP solve method from 
# https://www.researchgate.net/publication/312049744_On-demand_high-capacity_ride-sharing_via_dynamic_trip-vehicle_assignment

from argparse import ArgumentParser
import time
from collections import namedtuple
import sys
import os
import psutil
from rvGraph import *  # Imports all functions needed to calculate the request vehicles Graph
from rtvGraph import *  # Imports all functions needed to calculate the request trips vehicles Graph
from lpSolver import *  # Import all functions needed to solve the ILP

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


class Request:
    def __init__(self, ID, name, depart, orig, dst, pax, pax_wc, orig_pos, dst_pos, orig_window, dst_window, drf):
        self.ID = ID  # request ID
        self.name = name  # request ID
        self.depart = depart  # desired depart time
        self.orig_edge = orig  # origin edge
        self.dest_edge = dst  # destination edge
        try:
            self.orig_pos = float(orig_pos) # origin position
        except:
            self.orig_pos = 5
        try:
            self.dest_pos =  float(dst_pos) # destination position
        except:
            self.dest_pos = 5
        try:
            self.orig_window = [int(orig_window.split(",")[0]), int(orig_window.split(",")[1])] # origin time window
        except:
            self.orig_window = None
        try:
            self.dest_window = [int(dst_window.split(",")[0]), int(dst_window.split(",")[1])]  # destination time window
        except:
            self.dest_window = None
        try:
            self.drf = float(drf)  # direct route factor
        except:
            self.drf = 2
        try:
            self.pax = int(pax)  # number of passengers
        except:
            self.pax = 1
        try:
            self.pax_wc = int(pax_wc)  # number of wheelchair passengers
        except:
            self.pax_wc = 0

        # value assigned later
        self.rejected = None  # if request must be rejected
        self.rejected_cause = None  # caused of reject
        self.direct_route = None  # direct route duration
        self.drt_area = []  # request in DRT service area
        self.drt_leg = []  # if DRT only for first/last mile
        self.pt = []  # public transport trip information [(line, from stop, to stop, time),...,[]]
        self.cost = 1  # requests with pt connection have priority
        self.value = 1
        self.waiting = 0
        self.arrival = 0
        self.trips = [] # list with potential trips

        #self.check_tt = [0,0] # attribute for checking travel time

        # not used variables:
        # self.delta = delta  # latest depart time allowed
        # self.walk = None  # walk from/to edge
        # self.start = None  # trip start time
        # self.pick_time = None  # DRT pick-up time
        # self.del_time = None  # DRT delivery time
        # self.vehicle = None  # assigned vehicle


class Vehicle:
    def __init__(self, ID, cost, cap, wc, depot, depot_pos, area, sumo_type, start_time, end_time):
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
    ap.add_argument("-n", "--network", dest="network", metavar="FILE",
                    help="SUMO network file", required=True)
    ap.add_argument("-r", "--requests", dest="requests", metavar="FILE",
                    help="requests file", required=True)
    ap.add_argument("-v", "--vehicles", dest="vehicles", metavar="FILE",
                    help="vehicles file", required=True)
    ap.add_argument("-a", "--service_area", dest="service_area", metavar="FILE",
                    help="service area edges file")
    ap.add_argument("-s", "--public_stops", dest="pt_stops", metavar="FILE",
                    help="public transport stops file")
    ap.add_argument("--extra-tt", type=int, default=600,
                    help="minimum extra time for the trip in comparision with car")

    ap.add_argument("-d", "--additional", dest="additional", type=str,
                    help="additional files")
    #ap.add_argument("--depot", dest="depot", help="depot file", metavar="FILE", required=False)
    #ap.add_argument("--depot_time", dest="depot_time", help="time without requests after vehicle go to depot "
    #                       "(default 1800sec)", type=float, default=1800, required=False)
    #ap.add_argument("--depot_minstay", dest="depot_minstay", help="min waiting time at depot for rerouting "
    #                        "(default 3600sec)", type=float, default=3600, required=False)
    ap.add_argument("--stop_length", dest="stop_length", type=int, required=True,
                    help="time in seconds a vehicle needs to stop for pick up or deliver a passenger")
    #ap.add_argument("--fix_route_time", dest="fix_route_time", help="maximal time (in sec) before pick up "
    #                        "for route changes", type=int, default=600, required=False)
    ap.add_argument("--c_ko", dest="c_ko", help="cost of ignoring a request", type=float, default=1000000000000)
    ap.add_argument("--cost_per_trip", dest="cost_per_trip", help="avoid using multiple vehicles if trip time is"
                                                                         "similar", type=float, default=600, required=False)
    ap.add_argument("--search_fleet", help="if min fleet must be search", action='store_true')
    ap.add_argument("--debug", help="get all info steps", action='store_true')
    ap.add_argument("--optimum", dest="optimum", help="if the optimum must be search", type=str, default="False", required=False)                                                                        
    ap.add_argument("--max_memory", dest="max_mem", help="max memory capacity (default: 3 GB)", type=float,
                           default=20000, required=False)
    ap.add_argument("--max_time", dest="max_time", help="max searching time (default: 5 sec)", type=float,
                           default=5, required=False)
    ap.add_argument("--DRT", dest="DRT", help="DRT-System name", type=str, default="DRT", required=False)
    ap.add_argument("-o", dest="path", help="path for output files", type=str, default="", required=False)
    #ap.add_argument("--park", dest="parking", help="park bus at stops", type=str, default="true", required=False)
    ap.add_argument("--sumocfg", dest="sumocfg", help="write sumocfg file", type=str, default="true", required=False)
    ap.add_argument("--maxwalk", dest="maxwalk", help="maximum walking time", type=int, default=240, required=False)
    ap.add_argument("--maxwait", dest="maxwait", help="maximum pt-waiting time", type=int, default=360, required=False)
    ap.add_argument("--minwait", dest="minwait", help="minimum pt-waiting time", type=int, default=60, required=False)
    ap.add_argument("--veh_wait", dest="veh_wait", help="maximum waiting time for passenger in the vehicle", type=int, default=180, required=False)
    ap.add_argument("--ride-hailing", dest="ride_hailing", help="search ride hailing when not optimum", type=bool, default=True, required=False)
    ap.add_argument("--keep-trips", help="consider options if differences larger than", type=int, default=300, required=False)
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

def read_requests(options):  # reads requests, creates class instance and returns requests list
    with open(options.requests, "r") as requests_file:
        requests_lines = requests_file.read().splitlines()
        delimiter = get_delimiter(requests_lines[0])

        requests = [request.split(delimiter) for request in requests_lines[1:]]

    # creates Request Class
    requests = [Request(int(index), str(request[0]), int(request[1]),  # ID, name, depart
                        str(request[2]), str(request[3]), int(request[4]), int(request[5]),  # orig, dest, pax, pax_wc
                        str(request[6]), str(request[7]),  # orig_pos, dest_pos
                        str(request[8]), str(request[9]),  # orig_window, dest_window
                        str(request[10])  # drf
                        ) for index, request in enumerate(requests)]
    return requests


def read_vehicles(options):  # reads vehicles, creates class instances and returns vehicles list
    with open(options.vehicles, "r") as vehicles_file:
        vehicles_lines = vehicles_file.read().splitlines()
        delimiter = get_delimiter(vehicles_lines[0])
        
        vehicles = [vehicle.split(delimiter) for vehicle in vehicles_lines[1:]]

    # creates Request Class
    vehicles = [Vehicle(str(vehicle[0]), int(vehicle[1]), int(vehicle[2]), int(vehicle[3]),  # ID, cost, cap, wc
                        str(vehicle[4]), float(vehicle[5]),  # depot, depot_pos
                        str(vehicle[6]), str(vehicle[7]), vehicle[8], vehicle[9]  # area, sumo_type, start_time, end_time
                        ) for vehicle in vehicles]
    return vehicles


def offline_case(options, requests, vehicles, service_areas, pt_stops):  # runs offline case

    if len(requests) < 1:
        print("no Requests were loaded")
        sys.exit()

    print("\nNumber of requests:", len(requests))
    print("Start RV-Graph")
    RV_graph, RD_dic = RV_offline(options, requests, vehicles, service_areas, pt_stops)

    #for debugging only:
    if options.debug:
        with open("%s%s_RV_graph_debugging.txt" % (options.path, options.DRT), "w+") as debug_file:
            for pair in RV_graph.keys():
                debug_file.write("%s: %s\n" % (pair, RV_graph[pair]))

    print("Start RTV-Graph")
    RTV_graph, memory_problems = RTV(options, RV_graph, vehicles, requests, RD_dic, service_areas)

    #for debugging only:
    if options.debug:
        with open("%s%s_RTV_graph_debugging.txt" % (options.path, options.DRT), "w") as debug_file:
            for trip in RTV_graph:
                debug_file.write("%s: %s\n" % (trip.trip_id, trip))

    print("Start LP Solver")
    run_LP(options, vehicles, requests, RTV_graph, memory_problems, RV_graph, RD_dic, Vehicle)

    if options.sumocfg == "true":  # write sumocfg file:
        try:
            options.additional = options.additional.replace('.dummy', '')
        except:
            "no pt routes given"
        with open("%s%s_Simulation.sumocfg" % (options.path, options.DRT), "w") as sumocfg:
            sumocfg.write('<?xml version="1.0" encoding="UTF-8"?>\n\n')
            sumocfg.write('<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
                          'xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">\n\n')
            sumocfg.write('\t<input>\n')
            sumocfg.write('\t\t<net-file value="%s"/>\n' % options.network)
            sumocfg.write('\t\t<route-files value="%s%s_Persons_routes.xml, %s%s_Vehicles_routes.xml"/>\n'
                          % (options.path, options.DRT, options.path, options.DRT))
            if options.additional:
                sumocfg.write('\t\t<additional-files value="%s"/>\n' % options.additional)
            sumocfg.write('\t</input>\n')
            sumocfg.write('\t<time>\n')            
            depart_times = [request.depart for request in requests if request.depart != None and request.rejected != True]
            depart_times.extend([vehicle.depart for vehicle in vehicles if vehicle.depart != None])
            if pt_stops:
                begin = min([x.time for x in pt_stops]) - 100
            else:
                begin = 0
            sumocfg.write('\t\t<begin value="%d"/>\n' % max(min(min(depart_times)-300, begin),0))
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
                service_areas.append(service_areas_element(area.split(delimiter)[0], area.split(delimiter)[1:]))
    except:
        service_areas = None

    try:
        with open(options.pt_stops, "r") as pt_stops_file:
            pt_stops_lines = pt_stops_file.read().splitlines()
            delimiter = get_delimiter(pt_stops_lines[0])
            pt_stops = []
            pt_stops_element = namedtuple('pt_stop', ['time', 'vehicle', 'line', 'stop', 'edge'])

            for stop in pt_stops_lines[1:]:
                stop_values = stop.split(delimiter)
                stop_values[0] = int(stop_values[0])
                pt_stops.append(pt_stops_element._make(stop_values))
    except:
        pt_stops = None

    # start
    offline_case(options, requests, vehicles, service_areas, pt_stops)

    print("Finished!")
    print("--- %0.1f sec and %0.1f MB ---" % (time.perf_counter() - start_code_time, process.memory_info().rss*0.000001))
    
    with open("%s%s_DRT_info.xml" % (options.path, options.DRT), "a") as summary_file:
        summary_file.write("""<!--Running_time(sec)="%0.1f" Running_memory(MB)="%0.1f"-->\n""" % (time.perf_counter() - start_code_time, process.memory_info().rss*0.000001))
        


if __name__ == "__main__":
    main()
