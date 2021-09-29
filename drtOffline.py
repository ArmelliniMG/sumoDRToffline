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
from sumolib.xml import parse_fast, parse_fast_nested  # noqa


class Request:
    def __init__(self, req_data):
        self.ID = req_data.get('id')
        self.name = req_data.get('name')
        self.depart = req_data.get('depart')
        self.orig_edge = req_data.get('from')
        self.dest_edge = req_data.get('to')
        self.orig_pos = req_data.get('departPos', 5)
        self.dest_pos = req_data.get('arrivalPos', 5)
        self.orig_window = req_data.get('pickup_window', None)
        self.dest_window = req_data.get('dropoff_window', None)
        self.drf = req_data.get('drf', 2)
        self.pax = int(req_data.get('passenger', 1))
        self.pax_wc = int(req_data.get('passenger_wc', 0))

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
    def __init__(self, veh_data):
        self.ID = veh_data.get('id')
        self.type = veh_data.get('type')
        self.cost = veh_data.get('cost', 0)
        self.cap = veh_data.get('max_capacity', 4)
        self.cap_wc = veh_data.get('max_wc', 0)
        self.depot = veh_data.get('depot_edge')
        self.depot_pos = veh_data.get('depot_pos', 0)
        self.area = veh_data.get('service_area', None)
        self.start_time = veh_data.get('start_time', 0)
        self.end_time = veh_data.get('end_time', 86400)

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
    ap.add_argument("--taz-area", action='store_true')
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


def read_requests(options):
    requests = []
    request_dict = {}
    parsing_person = None
    person_idx = 0
    # reads requests, creates class instance and returns requests list
    for person, param in parse_fast_nested(options.reservations,
                                           "person", ("id", "depart",
                                                      "from", "to"),
                                           "param", ("key", "value")):
        if parsing_person != person.id:
            if parsing_person is not None:
                requests.append(Request(request_dict))
                person_idx += 1
            request_dict = {}
            parsing_person = person.id
            request_dict['id'] = person_idx
            request_dict['name'] = person.id
            request_dict['depart'] = float(person.depart)
            request_dict['from'] = person.attr_from
            request_dict['to'] = person.to
        if param.key in ('pickup_window', 'dropoff_window'):
            t_window = param.value.split(",")
            request_dict[param.key] = float(t_window[0]), float(t_window[1])
        else:
            request_dict[param.key] = float(param.value)

    requests.append(Request(request_dict))  # append last request

    return requests


def read_vehicles(options):
    vehicles = []
    vehicle_dict = {}
    parsing_vehicle = None
    # reads requests, creates class instance and returns requests list
    for vehicle, param in parse_fast_nested(options.taxis,
                                            "vehicle", ("id", "type"),
                                            "param", ("key", "value")):
        if parsing_vehicle != vehicle.id:
            if parsing_vehicle is not None:
                vehicles.append(Vehicle(vehicle_dict))
            vehicle_dict = {}
            parsing_vehicle = vehicle.id
            vehicle_dict['id'] = vehicle.id
            vehicle_dict['type'] = vehicle.type
        if param.key in ('depot_edge', 'service_area'):
            vehicle_dict[param.key] = param.value
        else:
            vehicle_dict[param.key] = float(param.value)

    vehicles.append(Vehicle(vehicle_dict))  # append last request

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

    service_areas = None
    if options.service_area:
        service_areas = []
        service_area = namedtuple('Area', ['name', 'edges'])
        if options.taz_area:
            parsing_area = None
            for area, edge in parse_fast(options.service_area,
                                         "taz", "id", "tazSource", "id"):
                if parsing_area != area.id:
                    area_edges = []
                    if parsing_area is not None:
                        service_areas.append(service_area(parsing_area,
                                                          area_edges))
                    parsing_area = area.id
                area_edges.append(edge)
        else:
            for area in parse_fast(options.service_area,
                                "service_area", ("id", "edges")):
                service_areas.append(service_area(area.id,
                                                  area.edges.split(',')))

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
