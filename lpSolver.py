# (C) 2021 Armellini
# This code is licensed under MIT license (see LICENSE for details)

# @file    lpSolver.py
# @author  Maria Giuliana Armellini
# @date    2020-01-01

# Solves the ILP and writes the results as SUMO files

import re
from pulp import *
import os
from collections import namedtuple
import copy
import time

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import sumolib


# formulates the problem as an integer linear programming (ILP), solves the problem with the tool "Pulp" and
# parses the solution as a SUMO-Routes file and XML-File with Summary info


def LPSOLVE(options, vehicles, requests, costs, vehicle_constraints, request_constraints, request_both_constraints, RTV_graph):
    # founds the combination of trips that minimize the costs function

    req_costs = [request.cost for request in requests]
    order_trips = costs.keys()

    # pdb.set_trace()
    ILP_result = []

    # Create the 'prob' variable to contain the problem data
    prob = LpProblem("Dial_a_Ride_Problem", LpMinimize)

    # A dictionary called 'Trips_vars' is created to contain the referenced Variables (all possible trips)
    Trips_vars = LpVariable.dicts("Trip", order_trips, cat='Binary')

    # add objective function
    prob += lpSum([costs[i] * Trips_vars[i] for i in order_trips]) - \
            options.c_ko * lpSum([sum([a * b for a, b in zip(request_constraints[i], req_costs)]) * Trips_vars[i] for i in order_trips]), \
            "Total_Trips_Travel_Time"
            # options.c_ko * lpSum([sum(request_constraints[i]) * Trips_vars[i] for i in order_trips]), \

    # cost of the possible solution= trips cost - (cost of serve a request * Nr of request served)

    # add constraints
    for index in range(len(vehicles)):
        prob += lpSum([vehicle_constraints[i][index] * Trips_vars[i] for i in order_trips]) <= 1, \
                "Max_1_Trip_for_Vehicle_%s" % index

    for index in range(len(requests)):
        prob += lpSum([request_constraints[i][index] * Trips_vars[i] for i in order_trips]) <= 1, \
                "Max_1_Trip_for_Request_%s" % index

    for index in range(len(requests)):
        prob += lpSum([request_constraints[i][index] * Trips_vars[i] for i in order_trips]) >= 0, \
                "Assure_2legs_trips_%s" % index
                #first + second = -0,25+1,25 = 1 -> 2 first already avoid by >=0 and 2 last avoid by <= 1
                #could happend 1,25-0,25*5 or this kind of combinations but it shouldnt because it will be selected the min time,
                #so 2 trips will be always minor than 3.

    for index in range(len(requests)):
        prob += lpSum([request_both_constraints[i][index] * Trips_vars[i] for i in order_trips]) >= -1, \
                "Assure_samept_2legs_%s" % index

    for index in range(len(requests)):
        prob += lpSum([request_both_constraints[i][index] * Trips_vars[i] for i in order_trips]) <= 1, \
                "Assure_samept_2legs_2_%s" % index


    prob += lpSum([sum(vehicle_constraints[i]) * Trips_vars[i] for i in order_trips]) >= 1, \
            "Avoid_null_result_by_assigning_at_least_one_vehicle"

    # The problem data is written into following file
    prob.writeLP("%sDRT_lp.txt" % options.path)

    # The problem is solved using PuLP's Solver choice
    prob.solve(PULP_CBC_CMD(msg=0))

    if LpStatus[prob.status] != 'Optimal':
        sys.exit("No optimal solution could be found. Return value: %s" % LpStatus[prob.status])
    else:  # if optimal solution was found
        for v in prob.variables():
            if v.varValue == 1:
                result = v.name.split("Trip_")[1]
                ILP_result.append(result)

    trips = []
    served_num = 0
    for trip in ILP_result:
        trips.append(RTV_graph[int(trip)])
        served_num += sum(RTV_graph[int(trip)].request_bin)

    return trips, served_num

def generate_output(options, vehicles, requests, memory_problems, fix_rejected):

    with open("%s%s_Persons_routes.xml" % (options.path, options.DRT), "w+") as pax_file, \
            open("%s%s_Vehicles_routes.xml" % (options.path, options.DRT), "w+") as route_file, \
            open("%s%s_DRT_info.xml" % (options.path, options.DRT), "w+") as summary_file:

        route_file.write("<routes>\n")
        pax_file.write("<routes>\n")
        summary_file.write("<DRT>\n")

        net = sumolib.net.readNet(options.network)
        
        used_fleet = 0
        vehicles.sort(key=lambda x: (x.depart is None, x.depart))
        for vehicle in vehicles:
            if not vehicle.trip:
                # if vehicle not assigned
                continue
            used_fleet += 1
            # write trip summary
            summary_file.write("""\t<vehicle id="%s" trip="%s" depart="%s" duration="%s" waiting_time="%s" idle_time="%s" passengers="%s" max_passengers="%s"/>\n"""
                               % (vehicle.ID, vehicle.trip, time.strftime('%H:%M:%S', time.gmtime(vehicle.depart)), vehicle.trip_duration, vehicle.waiting_time, vehicle.idle_time, vehicle.pax, vehicle.max_pax))

            # write vehicle route
            route_file.write("""\t<trip id="%s" type="%s" depart="%s" departLane="best" from="%s" to ="%s">\n"""
                                 % (vehicle.ID, vehicle.type, time.strftime('%H:%M:%S', time.gmtime(vehicle.depart)), vehicle.depot, vehicle.depot))

            stops = vehicle.trip.split("_")

            stops_id = ()
            stops_edge = ()
            stops_pos = ()
            stops_lane = ()
            stops_triggered = []
            stops_expected = ()

            # write all vehicle stops
            for j, stop in enumerate(stops[1:]):
                req_id= [s for s in stop if s.isdigit()]
                req_id = int(''.join(req_id))
                request = [r for r in requests if r.ID == req_id][0]
                expected = ""

                if stop.endswith("y"):  # if pick up
                    edge = [req_trip.orig_edge for req_trip in request.trips if str(req_trip.orig_ID) == stop[:-1]][0]
                    pos = [req_trip.orig_pos for req_trip in request.trips if str(req_trip.orig_ID) == stop[:-1]][0]
                    lane = next((lane for lane in range(len(net.getEdge(edge).getLanes())) if
                                    any([vclass in net.getEdge(edge).getLane(int(lane))._allowed for vclass in ['bus', "passenger"]])), None)
                    triggered = True
                    for p in range(request.pax + request.pax_wc):
                        expected = "%s_%i %s" % (request.name, p, expected)
                elif stop.endswith("z"):  # if drop off
                    edge = [trip.dest_edge for trip in request.trips if str(trip.dest_ID) == stop[:-1]][0]
                    pos = [trip.dest_pos for trip in request.trips if str(trip.dest_ID) == stop[:-1]][0]
                    lane = next((lane for lane in range(len(net.getEdge(edge).getLanes())) if
                                    any([vclass in net.getEdge(edge).getLane(int(lane))._allowed for vclass in ['bus', "passenger"]])), None)
                    triggered = False
                    expected = None
                
                stops_id = stops_id + (stop,)
                stops_edge = stops_edge + (edge,)
                stops_pos = stops_pos + (pos,)
                stops_lane = stops_lane + (lane,)
                stops_triggered.append(triggered)
                stops_expected = stops_expected + (expected,)            

                # save request information
                for stop2 in stops[j+2:]:
                    req_id2= [s for s in stop2 if s.isdigit()]
                    req_id2 = int(''.join(req_id2))
                    if req_id == req_id2 and stop2.endswith("z"):
                        pt_vehicle = [req_trip.pt_vehicle for req_trip in request.trips if str(req_trip.orig_ID) == stop[:-1]][0]                    
                        request.pt.append(["%s_%s" % (stop, stop2), vehicle.ID, pt_vehicle])
                        break
                       

            add_expected = ""
            stop_id = ""
            for i, stop_edge in enumerate(stops_edge):
                j = i+1
                if j != len(stops_edge) and stop_edge == stops_edge[j] and stops_pos[i] == stops_pos[j]:
                    stop_id = "%s %s" % (stop_id, stops_id[i])
                    if stops_expected[i] != None:
                        add_expected = "%s %s" % (add_expected, stops_expected[i])
                    if stops_triggered[i] == True and stops_triggered[j] != True:
                        stops_triggered[j] = True
                    continue
                if stops_expected[i] != None:
                    stop_expect = stops_expected[i]
                    route_file.write(
                        """\t\t<stop lane="%s_%s" startPos="%0.1f" endPos="%0.1f" duration="%d" triggered="%s" expected="%s%s" permitted="%s%s" parking="True"/><!-- %s%s -->\n"""
                        % (stop_edge, stops_lane[i], stops_pos[i] - 0.5, stops_pos[i] + 0.5, options.stop_length, stops_triggered[i], stop_expect, add_expected, stop_expect, add_expected, stops_id[i], stop_id))
                elif stops_expected[i] == None and add_expected != "":
                    route_file.write(
                        """\t\t<stop lane="%s_%s" startPos="%0.1f" endPos="%0.1f" duration="%d" triggered="%s" expected="%s" permitted="%s%s" parking="True"/><!-- %s%s -->\n"""
                        % (stop_edge, stops_lane[i], stops_pos[i] - 0.5, stops_pos[i] + 0.5, options.stop_length, stops_triggered[i], add_expected, stops_triggered[i], add_expected, stops_id[i], stop_id))
                else:
                    route_file.write(
                        """\t\t<stop lane="%s_%s" startPos="%0.1f" endPos="%0.1f" duration="%d" triggered="%s" parking="True"/><!-- %s%s -->\n"""
                        % (stop_edge, stops_lane[i], stops_pos[i] - 0.5, stops_pos[i] + 0.5, options.stop_length, stops_triggered[i], stops_id[i], stop_id))
                add_expected = ""
                stop_id = ""
            route_file.write("""\t</trip>\n""")

        not_served = []  # list with not served requests
        requests.sort(key=lambda x: x.depart)
        for request in requests:  # write passenger trips
            if len(request.pt) == 1:
                # if only DRT or DRT for first OR last mile
                stop = request.pt[0][0]
                stop = stop.split("_")
                leg = [trip.leg for trip in request.trips if (str(trip.orig_ID) == stop[0][:-1]) and (str(trip.dest_ID) == stop[1][:-1])][0]
                if not leg:
                # if request only use DRT
                    waiting_pickup = request.depart - request.orig_window[0]
                    duration = request.arrival-request.depart
                    drf = (request.arrival-request.depart)/request.direct_route
                    passengers = request.pax+request.pax_wc
                    summary_file.write("""\t<request id="%s" name="%s" depart="%s" passengers="%s" rejected="false" rejected_cause="None" waiting_pickup="%0.0f" duration="%0.0f" drf="%0.2f"/>\n"""
                                % (request.ID, request.name, time.strftime('%H:%M:%S', time.gmtime(request.depart)), passengers, waiting_pickup, duration, drf))

                    for person in range(request.pax + request.pax_wc):
                        pax_file.write("""\t<person id="%s_%d" depart="%s" departPos="%0.1f">\n"""
                                    % (request.name, person, time.strftime('%H:%M:%S', time.gmtime(request.depart)), request.orig_pos))
                        pax_file.write("""\t\t<ride from="%s" to="%s" arrivalPos="%0.1f" lines="%s"/>\n """ % (request.orig_edge, request.dest_edge,
                                                                                                                request.dest_pos, request.pt[0][1]))
                        pax_file.write("\t</person>\n")
                    
                #TODO pt connection must be checked
                elif leg == "first":
                # if request only use DRT for first mile and then take pt and walk
                    passengers = request.pax+request.pax_wc
                    summary_file.write("""\t<request id="%s" name="%s" depart="%s" passengers="%s" rejected="false" rejected_cause="None" waiting_pickup="first_mile" duration="first_mile" drf="first_mile"/>\n"""
                                % (request.ID, request.name, time.strftime('%H:%M:%S', time.gmtime(request.depart)), passengers))

                    DRT_dest_edge = [trip.dest_edge for trip in request.trips if str(trip.dest_ID) == stop[1][:-1]][0]
                    DRT_dest_pos = [trip.dest_pos for trip in request.trips if str(trip.dest_ID) == stop[1][:-1]][0]
                    pt_vehicle = [trip.pt_vehicle for trip in request.trips if str(trip.dest_ID) == stop[1][:-1]][0]
                    bus_stop_from = [trip.bus_stop_from for trip in request.trips if str(trip.dest_ID) == stop[1][:-1]][0]
                    bus_stop_to = [trip.bus_stop_to for trip in request.trips if str(trip.dest_ID) == stop[1][:-1]][0]
                    for person in range(request.pax + request.pax_wc):
                        pax_file.write("""\t<person id="%s_%d" depart="%s" departPos="%0.1f">\n"""
                                    % (request.name, person, time.strftime('%H:%M:%S', time.gmtime(request.depart)), request.orig_pos))
                        #pax_file.write("""\t\t<walk from="%s" to="%s" arrivalPos="%0.1f"/>\n"""
                        #            % (request.orig_edge, request.orig_edge, request.orig_pos))
                        pax_file.write("""\t\t<ride from="%s" to="%s" arrivalPos="%0.1f" lines="%s"/>\n """ % (request.orig_edge, DRT_dest_edge, DRT_dest_pos, 
                                                                                                        request.pt[0][1]))
                        pax_file.write("""\t\t<walk busStop="%s"/>\n""" % bus_stop_from)
                        pax_file.write("""\t\t<ride busStop="%s" lines="%s"/>\n """ % (bus_stop_to, pt_vehicle))
                        pax_file.write("""\t\t<walk to="%s" arrivalPos="%0.1f"/>\n""" % (request.dest_edge, request.dest_pos))
                        pax_file.write("\t</person>\n")
                    
                elif leg == "last":
                # if request only use DRT for last mile. First walk to pt stop and take pt line                
                    passengers = request.pax+request.pax_wc
                    summary_file.write("""\t<request id="%s" name="%s" depart="%s" passengers="%s" rejected="false" rejected_cause="None" waiting_pickup="last_mile" duration="last_mile" drf="last_mile"/>\n"""
                                % (request.ID, request.name, time.strftime('%H:%M:%S', time.gmtime(request.depart)), passengers))

                    DRT_orig_edge = [trip.orig_edge for trip in request.trips if str(trip.orig_ID) == stop[0][:-1]][0]
                    DRT_orig_pos = [trip.orig_pos for trip in request.trips if str(trip.orig_ID) == stop[0][:-1]][0]
                    pt_vehicle = [trip.pt_vehicle for trip in request.trips if str(trip.orig_ID) == stop[0][:-1]][0]
                    bus_stop_from = [trip.bus_stop_from for trip in request.trips if str(trip.orig_ID) == stop[0][:-1]][0]
                    bus_stop_to = [trip.bus_stop_to for trip in request.trips if str(trip.orig_ID) == stop[0][:-1]][0]
                    for person in range(request.pax + request.pax_wc):
                        pax_file.write("""\t<person id="%s_%d" depart="%s" departPos="%0.1f">\n"""
                                    % (request.name, person, time.strftime('%H:%M:%S', time.gmtime(request.depart)), request.orig_pos))
                        pax_file.write("""\t\t<walk from="%s" busStop="%s"/>\n""" % (request.orig_edge, bus_stop_from))
                        pax_file.write("""\t\t<ride busStop="%s" lines="%s"/>\n """ % (bus_stop_to, pt_vehicle))
                        pax_file.write("""\t\t<walk to="%s" arrivalPos="%0.1f"/>\n""" % (DRT_orig_edge, DRT_orig_pos))
                        pax_file.write("""\t\t<ride to="%s" arrivalPos="%0.1f" lines="%s"/>\n """ % (request.dest_edge, request.dest_pos,
                                                                                                        request.pt[0][1]))
                        pax_file.write("\t</person>\n")
                       
            #TODO  pt connection must be checked
            elif len(request.pt) == 2:
                # if DRT for first AND last mile
                passengers = request.pax+request.pax_wc
                summary_file.write("""\t<request id="%s" name="%s" depart="%s" passengers="%s" rejected="false" rejected_cause="None" waiting_pickup="first_last_mile" duration="first_last_mile" drf="first_last_mile"/>\n"""
                            % (request.ID, request.name, time.strftime('%H:%M:%S', time.gmtime(request.depart)), passengers))

                stop1 = request.pt[0][0]
                stop1 = stop1.split("_")
                stop2 = request.pt[1][0]
                stop2 = stop2.split("_")
                DRT1_dest_edge = [trip.dest_edge for trip in request.trips if str(trip.dest_ID) == stop1[1][:-1]][0]
                DRT1_dest_pos = [trip.dest_pos for trip in request.trips if str(trip.dest_ID) == stop1[1][:-1]][0]
                DRT2_orig_edge = [trip.orig_edge for trip in request.trips if str(trip.orig_ID) == stop2[0][:-1]][0]
                DRT2_orig_pos = [trip.orig_pos for trip in request.trips if str(trip.orig_ID) == stop2[0][:-1]][0]
                pt_vehicle = [trip.pt_vehicle for trip in request.trips if str(trip.orig_ID) == stop2[0][:-1]][0]
                bus_stop_from = [trip.bus_stop_from for trip in request.trips if str(trip.dest_ID) == stop1[1][:-1]][0]
                bus_stop_to = [trip.bus_stop_to for trip in request.trips if str(trip.orig_ID) == stop2[0][:-1]][0]
                for person in range(request.pax + request.pax_wc):
                    pax_file.write("""\t<person id="%s_%d" depart="%s" departPos="%0.1f">\n"""
                                % (request.name, person, time.strftime('%H:%M:%S', time.gmtime(request.depart)), request.orig_pos))
                    #pax_file.write("""\t\t<walk from="%s" to="%s" arrivalPos="%0.1f"/>\n"""
                    #            % (request.orig_edge, request.orig_edge, request.orig_pos))
                    pax_file.write("""\t\t<ride from="%s" to="%s" arrivalPos="%0.1f" lines="%s"/>\n """ % (request.orig_edge, DRT1_dest_edge, DRT1_dest_pos, 
                                                                                                    request.pt[0][1]))
                    pax_file.write("""\t\t<walk busStop="%s"/>\n""" % bus_stop_from)
                    pax_file.write("""\t\t<ride busStop="%s" lines="%s"/>\n """ % (bus_stop_to, pt_vehicle))
                    pax_file.write("""\t\t<walk to="%s" arrivalPos="%0.1f"/>\n""" % (DRT2_orig_edge, DRT2_orig_pos))
                    pax_file.write("""\t\t<ride to="%s" arrivalPos="%0.1f" lines="%s"/>\n """ % (request.dest_edge, request.dest_pos,
                                                                                                    request.pt[1][1]))
                    pax_file.write("\t</person>\n")
       
            elif (len(request.pt) == 0) or (request.rejected == True):
                # request not possible to serve
                not_served.append(request.name)
                if request.rejected != True:
                    request.rejected = True
                    if not request.rejected_cause:
                        request.rejected_cause = "No DRT vehicle available"
                summary_file.write("""\t<request id="%s" name="%s" depart="%s" passengers="%s" rejected="true" rejected_cause="%s" waiting_pickup="" duration="" drf=""/>\n"""
                                % (request.ID, request.name, time.strftime('%H:%M:%S', time.gmtime(request.depart)), request.pax+request.pax_wc, request.rejected_cause))
            else:
                print("Check remain option")

        if not_served:
            print("\tWarning: Requests", not_served, "could not be served")
        summary_file.write("""\t<summary vehicles="%d" requests="%d" rejected="%d" rejected_independ_fleet="%d" memoryroblems="%s"/>\n""" % (used_fleet, len(requests), len(not_served), fix_rejected, memory_problems))
        summary_file.write("</DRT>\n")
        route_file.write("</routes>")
        pax_file.write("</routes>")

    print("Selected trips:")
    print([vehicle.trip for vehicle in vehicles if vehicle.trip])


def parse_output_of_LPSOLVE(options, trips, requests, memory_problems, vehicles, RTV_dic, RV_dic, RD_dic):
    # write SUMO-routes (person and vehicles separated to avoid unsorted routes) and Summary file
    
    trips.sort(key=lambda x: x.departure_time)
    for trip in trips:
        
        # vehicle info
        vehicle = vehicles[int(trip.vehicle_bin.index(1))]
        vehicle.trip = trip.trip_id
        vehicle.depart = trip.departure_time
        vehicle.waiting_time = trip.waiting_time
        vehicle.trip_duration = trip.trip_duration

        # parse stops
        stops = trip.trip_id.split("_")        
        time_cum = trip.departure_time
        num_pax = 0
        max_pax = 0
        pax = []
        for index, stop in enumerate(stops):            
            # drop off time window not be contemplated, request will be dropped off
            # at earliest time possible -> to aloow dropp off time windows, this analyis
            # should be done backwards, which nevertheless has some other problems (see
            # qinrui algorithm)
            if index > 0:                
                req_id= [s for s in stop if s.isdigit()]
                req_id = int(''.join(req_id))
                request = [r for r in requests if r.ID == req_id][0]
                if index == 1 and options.search_fleet:
                    # correct vehicle id if search fleet is true
                    pair = "%s_%s" % (vehicles[0].ID, stop)
                else:
                    pair = "%s_%s" % (stops[index-1], stop)
                time_cum += RV_dic[pair][0]
                
                if stop.endswith("y"):
                    leg = [req_trip.leg for req_trip in request.trips if str(req_trip.orig_ID) == stop[:-1]][0]
                    if leg == "last":
                        # data for last mile not relevant
                        pax.append(request.name)
                        num_pax += 1
                        max_pax = max(num_pax, max_pax) # search max number of pax transported
                        continue
                    req_orig_window = [req_trip.orig_window for req_trip in request.trips if str(req_trip.orig_ID) == stop[:-1]][0]
                    i = 1
                    next_stop = stops[index+i]
                    next_req_id= [s for s in next_stop if s.isdigit()]
                    next_req_id = int(''.join(next_req_id))
                    next_stop_time = RV_dic["%s_%s" % (stop, stops[index+i])][0]
                    while next_req_id != req_id:
                        i += 1
                        next_stop = stops[index+i]
                        next_req_id= [s for s in next_stop if s.isdigit()]
                        next_req_id = int(''.join(next_req_id))
                        next_stop_time += RV_dic["%s_%s" % (stops[index+i-1], stops[index+i])][0]

                    #if next_req_id == req_id and next_stop.endswith("z"):
                    #next_pair = "%s_%s" % (stop, stops[index+1])
                    #next_stop_time = RV_dic[next_pair][0]
                    req_dest_window = [req_trip.dest_window for req_trip in request.trips if str(req_trip.dest_ID) == next_stop[:-1]][0][0]
                    if req_orig_window[0] < (req_dest_window - next_stop_time) < req_orig_window[1]:
                        req_pick_time = req_dest_window - next_stop_time
                    else:
                        req_pick_time = req_orig_window[0]
                    #else:
                    #        req_pick_time = req_orig_window[0]
                        
                    if time_cum < req_pick_time:
                        # if vehicle arrives before earliest request departure
                        if num_pax == 0:
                            # vehicle waiting idle
                            vehicle.idle_time += req_pick_time - time_cum
                        time_cum = req_pick_time
                    elif time_cum > req_pick_time:
                        # if passenger waited
                        request.waiting = time_cum - req_pick_time
                    
                    request.depart = time_cum
                    pax.append(request.name)
                    num_pax += 1
                    max_pax = max(num_pax, max_pax) # search max number of pax transported
                
                elif stop.endswith("z"):
                    leg = [req_trip.leg for req_trip in request.trips if str(req_trip.dest_ID) == stop[:-1]][0]
                    if leg == "first":
                        # data for first mile not relevant
                        num_pax += -1
                        continue
                    request.arrival = max(time_cum, request.arrival)
                    num_pax += -1
        vehicle.max_pax = max_pax
        vehicle.pax = pax

def create_input_for_LPSOLVE(options, vehicles, requests, RTV_graph):
    vehicle_constraints = {}
    request_constraints = {}
    request_both_constraints = {}
    costs = {}

    # add bonus_cost to trip cost (makes trips with more served requests cheaper than splitting the requests to more
    # smaller trips if both strategies would yield a similar cost and considers cost of using a specific car)
    for idx, trip in enumerate(RTV_graph):
        bonus_cost = (sum(trip.request_bin) + 1) * options.cost_per_trip + \
                     vehicles[int(trip.vehicle_bin.index(1))].cost
        costs.update({idx: trip.trip_duration + bonus_cost})  # generate dict with cost
        vehicle_constraints.update({idx: trip.vehicle_bin})  # generate dict with vehicle used in the trip
        request_constraints.update({idx: trip.request_bin})  # generate dict with served requests in the trip
        #try fix requests with drt for both legs
        if (1.25 in trip.request_bin) or (-0.25 in trip.request_bin):
            request_both_bin = [0] * len(requests)
            for req_index, req_value in enumerate(trip.request_bin):
                if req_value == 1.25 or req_value == -0.25:
                    req = requests[req_index]
                    stops = trip.trip_id.split("_")
                    stops = [stop for stop in stops if str(req.ID) in stop]
                    stop = [stop for stop in stops if len(stop)> 2][0]
                    if 'y' in stop[-1]:                        
                        request_both_bin[req_index] = ord(stop[1])
                    elif 'z' in stop[-1]:
                        request_both_bin[req_index] = -ord(stop[1])
            request_both_constraints.update({idx: request_both_bin})
        else:            
            request_both_bin = [0] * len(requests)
            request_both_constraints.update({idx: request_both_bin})
    
    return vehicle_constraints, request_constraints, request_both_constraints, costs

def run_LP(options, vehicles, requests, RTV_graph, memory_problems, RV_dic, RD_dic, Vehicle):
   
    vehicle_constraints, request_constraints, request_both_constraints, costs = create_input_for_LPSOLVE(options, vehicles, requests, RTV_graph)

    trips, served_num = LPSOLVE(options, vehicles, requests, costs, vehicle_constraints, request_constraints, request_both_constraints, RTV_graph)
    
    requests_vector = [0]*len(requests)
    for trip in RTV_graph:
        requests_vector = [x + y for x, y in zip(requests_vector, trip.request_bin)]        
    fix_rejected = requests_vector.count(0)
    
    if options.search_fleet:
        RTV_graph_element = namedtuple('RTV_graph_element', ['trip_id', 'departure_time', 'trip_duration', 'vehicle_bin', 'request_bin', 'waiting_time'])
        i = 0
        v_base = vehicles[0]
        base_index = len(RTV_graph)
        while (served_num + fix_rejected) < len(requests):
            # add one vehicle and solve the ILP until all request are served
            i = i + 1
            vehicles.append(Vehicle("%s%s" % (v_base.ID, i), v_base.cost, v_base.cap, v_base.cap_wc, v_base.depot, v_base.depot_pos, v_base.area, v_base.type, v_base.start_time, v_base.end_time))
            for j in range(i):
                # to update all values the index must be the first of the updated values
                update_index = len(RTV_graph) - base_index * (j+1)
                RTV_graph[update_index].vehicle_bin.append(0) # update vehicle_bin existing trips
            # vehicle bin for new vehicle
            new_v_bin = [0] * (len(vehicles))
            new_v_bin[-1] = 1
            for element in RTV_graph:
                if element.trip_id.split("_")[0] == v_base.ID:
                    trip_id = element.trip_id.replace(v_base.ID,"%s%s" % (v_base.ID, i))
                    RTV_graph.append(RTV_graph_element(trip_id, element.departure_time, element.trip_duration, new_v_bin, element.request_bin, element.waiting_time))
                else:
                    break
            
            # solve ILP again
            vehicle_constraints, request_constraints, request_both_constraints, costs = create_input_for_LPSOLVE(options, vehicles, requests, RTV_graph)
            trips, served_num = LPSOLVE(options, vehicles, requests, costs, vehicle_constraints, request_constraints, request_both_constraints, RTV_graph)

    parse_output_of_LPSOLVE(options, trips, requests, memory_problems, vehicles, RTV_graph, RV_dic, RD_dic)

    generate_output(options, vehicles, requests, memory_problems, fix_rejected)

