# (C) 2021 Armellini
# This code is licensed under MIT license (see LICENSE for details)

# @file    rvGraph.py
# @author  Maria Giuliana Armellini
# @date    2020-01-01

# Creates the pairwise Graph for requests and vehicles
 
try:
    import xml.etree.cElementTree as ET
except ImportError as e:
    print("recovering from ImportError '%s'" % e)
    import xml.etree.ElementTree as ET
import subprocess
import re
from collections import namedtuple
import sys
import os
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    import sumolib
except ImportError:
    sys.exit("please declare environment variable 'SUMO_HOME'")


def get_direct_routes(options, requests):
    # determined shortest route for each request
    with open("%saux_dua_input.xml" % options.path, "w+") as dua_file:
        dua_file.write("<routes>\n")
        for req in requests:
            # write Duarouter input
            dua_file.write("""\t<trip id="SR-%sy_%sz" depart="0.0" departPos="%0.1f" from="%s" to="%s" 
                            arrivalPos="%0.1f"/>\n""" % (req.ID, req.ID, req.orig_pos, req.orig_edge, req.dest_edge, req.dest_pos))
        dua_file.write("</routes>\n")
    
    # run duarouter:
    subprocess.call("duarouter -n %s --route-files %saux_dua_input.xml -o %saux_dua_output.xml --ignore-errors true --no-warnings true" % (options.network,  options.path,  options.path))
    
    # parse solutions:
    for trip, route in sumolib.xml.parse_fast_nested("%saux_dua_output.alt.xml" % options.path, "vehicle", ("id"), "route", ("cost", "edges")):        
        travel_time = int(float(route.cost))
        route_id = trip.id.split("-")[1]
        req_id = int(route_id.split("_")[1][:-1])
        request =  [req for req in requests if req.ID == req_id][0] # request element
        
        # set direct route time
        request.direct_route = travel_time + options.stop_length  # minimum travel time
        
        if len(route.edges.split(" ")) <= 1:
            print("Warning: Request %s has same origin/destination edge" % request.ID)

        # define time windows with drf if not given
        if request.orig_window and request.dest_window:
            continue
        elif not request.orig_window and not request.dest_window:
            earliest_pickup = request.depart
            earliest_dropoff = earliest_pickup + travel_time
            latest_dropoff = earliest_pickup + max(options.extra_tt, (travel_time * request.drf))
            latest_pickup = latest_dropoff - travel_time

        elif not request.orig_window and request.dest_window:
            # if pickup at depart time can lead to a loger route than possible, then reset value
            earliest_pickup = max(request.depart, request.dest_window[1] - travel_time * request.drf)
            # check earliest pick up not later than earliest drop off       
            earliest_pickup = min(earliest_pickup, request.dest_window[0] - travel_time)
            latest_pickup = request.dest_window[1] - travel_time

        elif request.orig_window and not request.dest_window:
            earliest_dropoff = request.orig_window[0] + travel_time
            latest_dropoff = request.orig_window[1] + max(options.extra_tt, (travel_time * request.drf))
        
        # set time windows
        if not request.orig_window:            
            request.orig_window = [earliest_pickup, latest_pickup]
        if not request.dest_window:
            request.dest_window = [earliest_dropoff, latest_dropoff]
        
        # DEBUG only
        if request.orig_window[0] > request.orig_window[1]:
            print("Bug: Origin window of request ", req_id, " not possible. Latest time < than earlier")
        if request.dest_window[0] > request.dest_window[1]:
            print("Bug: Destination window of request ", req_id, " not possible. Latest time < than earlier")


def get_pt_routes(requests, options, unique_stops, service_areas, req_trip, drt_service, pt_stops):
    # determined DRT service area and if a trip with PT is required for each request
    with open("%saux_duapt_input.xml" % options.path, "w+") as dua_PTfile:
        dua_PTfile.write("<routes>\n")
        
        requests.sort(key=lambda x: x.depart)
        for req in requests:
            req.drt = []
            search_PT = False
            for area in service_areas:            
                if req.orig_edge in area.edges and req.dest_edge in area.edges:
                    # request in DRT area
                    req.drt.append(drt_service(area.name,"both"))
                    req.trips.append(req_trip(req.ID, req.ID, req.orig_edge, req.dest_edge, req.orig_pos, req.dest_pos, req.depart, req.orig_window, req.dest_window, area.name, None, None, None, None, None))
                elif req.orig_edge in area.edges:
                    # only request origin in DRT area. First mile with DRT and then PT
                    req.drt.append(drt_service(area.name,"first"))
                    search_PT = True
                elif req.dest_edge in area.edges:
                    # only request destination in DRT area. Last mile with DRT (before PT)
                    req.drt.append(drt_service(area.name,"last"))
                    search_PT = True
            
            if not req.drt:
                # request origin and destination are not in DRT service areas: trip not possible
                #warning_info.append("Request %s (id %s) could not be served. Origin and destination outside DRT service areas" %(req.name, req.ID))
                print("Request %s could not be served. Origin and destination outside DRT service areas" % req.ID)
                req.rejected = True
                req.rejected_cause = "No DRT service available in trip area"
            
            if search_PT == True:
                # if trip with PT is neccesary
                # Write trip for PT
                dua_PTfile.write("""\t<person id="%s" depart="%d">\n""" % (req.ID, req.depart))
                dua_PTfile.write("""\t\t<personTrip from="%s" to="%s" modes="public"/>\n""" % (req.orig_edge, req.dest_edge))
                dua_PTfile.write("\t</person>\n")

                # Write trip from request origin to all possibles PT stops
                for stop_id, stop_edge in unique_stops.items():
                    dua_PTfile.write("""\t<person id="%s-%s" depart="0">\n""" % (req.ID, stop_id))
                    dua_PTfile.write("""\t\t<walk from="%s" to="%s"/>\n""" % (req.orig_edge, stop_edge))
                    dua_PTfile.write("\t</person>\n")
                    dua_PTfile.write("""\t<person id="%s-%s" depart="0">\n""" % (stop_id, req.ID))
                    dua_PTfile.write("""\t\t<walk from="%s" to="%s"/>\n""" % (stop_edge, req.dest_edge))
                    dua_PTfile.write("\t</person>\n")
        
        dua_PTfile.write("</routes>\n")

    # runs Duarouter.py for public transport:
    subprocess.call(
        "duarouter -n %s --additional-files %s --route-files %saux_duapt_input.xml -o %saux_duapt_output.xml --ignore-errors true --no-warnings true"
        % (options.network, options.additional, options.path, options.path))
    
    # remove trips that requires pt combination
    pt_combination = {}
    pt_routed = []
    count = -1
    index = None
    rejected = None
    for _, parsenode in ET.iterparse("%saux_duapt_output.xml" % options.path, None):
        count += 1
        if parsenode.tag == "walk" and "busStop" in parsenode.attrib and not index:
            from_stop = parsenode.attrib["busStop"]
        elif parsenode.tag == "ride":
            if index:
                # if person need more than 1 pt ride, trip not possible
                rejected = True
                continue
            to_stop = parsenode.attrib["busStop"]
            pt_line = parsenode.attrib["lines"]
            index = count
        elif parsenode.tag == "person" and not "-" in parsenode.attrib["id"]:
            pt_routed.append(int(parsenode.attrib["id"]))
            if rejected:
                # if person need more than 1 pt ride, trip not possible
                req = [x for x in requests if x.ID == int(parsenode.attrib["id"])][0]
                print("Request %s could not be served. PT connection requires multiple lines" % parsenode.attrib["id"])
                req.rejected = True
                req.rejected_cause = "PT connection requires multiple lines"
            elif not from_stop:
                # if not pt combination possible
                req = [x for x in requests if x.ID == int(parsenode.attrib["id"])][0]
                print("Request %s could not be served. PT connection not available" % parsenode.attrib["id"])
                req.rejected = True
                req.rejected_cause = "PT connection not available"
            else:
                pt_combination[parsenode.attrib["id"]] = [from_stop, to_stop, pt_line, index]
            from_stop, to_stop, pt_line, index, rejected = None, None, None, None, None
            count = -1
        elif parsenode.tag == "person" and "-" in parsenode.attrib["id"]:
            count = -1
    
    for req in requests:
        if req.ID not in pt_routed:
            print("Request %s could not be served. PT connection not available" % req.ID)
            req.rejected = True
            req.rejected_cause = "PT connection not available"
    
    # for person in sumolib.xml.parse("%saux_dua_output.xml" % options.path, "person"):
    #     if "-" in person.id:
    #         # walk trip
    #         continue
    #     elif person.id not in pt_combination:
    #         for index, trip in enumerate(person._child_list):
    #             if trip.name == "ride":
    #                 from_stop = person._child_list[index-1].busStop
    #                 pt_combination[person.id] = [from_stop, trip.busStop, trip.lines, index]
    #     else:
    #         # if person have to take more than 1 pt service, trip not possible
    #         del pt_combination[person.id]
    #         req = [x for x in requests if x.ID == int(person.id)][0]
    #         print("Request %s could not be served. No connection with PT available" % person.id)
    #         req.rejected = True
        
    # parse travel times
    travel_time = {}
    for person, trip in sumolib.xml.parse_fast_nested("%saux_duapt_output.alt.xml" % options.path, "person", ("id"), "personTrip", ("costs")):
        costs = [int(float(x)) for x in trip.costs.split( )]
        if "-" in person.id:
            # should be only 1 cost
            travel_time[person.id] = sum(costs)
        elif person.id in pt_combination:
            # first/last cost is the walk time to/from stop
            ride_index = pt_combination[person.id][3]
            walk_first = sum(costs[:ride_index])
            walk_last = sum(costs[ride_index+1:])
            travel_time[person.id] = [walk_first, costs[ride_index], walk_last]

    # read public transport routes:
    for key in pt_combination:        
        req = [x for x in requests if x.ID == int(key)][0]
        bus_stop_from = pt_combination[key][0]
        bus_stop_to = pt_combination[key][1]
        pt_line = pt_combination[key][2]
        first_mile = travel_time[key][0]
        last_mile = travel_time[key][2]

        string_pos = 96
        
        # set DRT service for first and last mile
        if req.drt[0].leg == "first":
            try:
                first_area = req.drt[0].area
                second_area = req.drt[1].area
            except:
                first_area = req.drt[0].area
                second_area = None
        else:
            try:
                second_area = req.drt[0].area
                first_area = req.drt[1].area
            except:
                second_area = req.drt[0].area
                first_area = None
        
        for area in service_areas:
            if area.name == first_area:
                first_area_edges = area.edges
            if area.name == second_area:
                second_area_edges = area.edges                

        if first_mile <= options.maxwalk and last_mile <= options.maxwalk:
            print("Request %s could not be served. Connection with PT with a short walking time is available" % req.ID)
            req.rejected = True
            req.rejected_cause = "PT connection accessible by walking"

        elif first_mile > options.maxwalk and last_mile > options.maxwalk:
            if first_area == None or second_area == None:
                print("Request %s could not be served. Walking times from/to PT exceed the maximum and there is no DRT service for the first/last mile" % req.ID)
                req.rejected = True
                req.rejected_cause = "No DRT service available in first/last mile area"
                continue
            
            # Max walking time to/from PT is exceeded: DRT service for first and second leg trip
            # search all usable PT stops
            for stop1 in pt_stops:
                if not pt_line in stop1.line:
                    continue
                elif not stop1.edge in first_area_edges:
                    continue
                elif not req.orig_window[0] < (stop1.time - options.minwait):
                    continue
                
                pt_vehicle = stop1.vehicle
                
                for stop2 in pt_stops:
                    if stop1.stop == stop2.stop:
                        continue
                    if not pt_line in stop2.line:
                        continue
                    elif stop1.time > stop2.time:
                        continue 
                    elif not stop2.vehicle == pt_vehicle:
                        continue
                    elif not stop2.edge in second_area_edges:
                        continue
                    elif not (stop2.time + options.minwait) < req.dest_window[1]:
                        continue

                    # first mile trip
                    string_pos = string_pos+1
                    first_dest_window = [max(req.orig_window[0], (stop1.time - options.maxwait)) , (stop1.time - options.minwait)]
                    first_orig_window = [req.orig_window[0], min(req.orig_window[1], first_dest_window[1])]
                    req.trips.append(req_trip(req.ID, "%i%s" % (req.ID, chr(string_pos)), req.orig_edge, stop1.edge, req.orig_pos, 10, req.depart, first_orig_window, first_dest_window, first_area, pt_line, pt_vehicle, stop1.stop, stop2.stop, "first"))
                    if first_dest_window[0] > first_dest_window[1]:
                        print("Bug: Destination window of request ", req.ID, " for Trip ", req.ID, "%i%s" % (req.ID, chr(string_pos)), "not possible. Latest time minor than earlier")
                    # last mile trip
                    #string_pos = string_pos+1
                    last_orig_window = [(stop2.time + options.minwait) , min(stop2.time + options.maxwait, req.dest_window[1])]
                    req.trips.append(req_trip("%i%s" % (req.ID, chr(string_pos)), req.ID, stop2.edge, req.dest_edge, 10, req.dest_pos, req.depart, last_orig_window, req.dest_window, second_area, pt_line, pt_vehicle, stop1.stop, stop2.stop, "last"))
                    if last_orig_window[0] > last_orig_window[1]:
                        print("Bug: Origin window of request ", req.ID, " for Trip ", "%i%s" % (req.ID, chr(string_pos)), req.ID, "not possible. Latest time minor than earlier")
                    # set request value to 0.5 to secure that both first and last mile will served
                    req.value = 0.5
                                
        
        elif first_mile > options.maxwalk:
            if first_area == None:
                print("Request %s could not be served. First mile requiers DRT service and there is no service in the area"% req.ID)
                req.rejected = True
                req.rejected_cause = "No DRT service available in first mile area"
                continue
            # Max walking time to PT exceeded: DRT service from origin to PT line (first leg trip)
            stop2 = bus_stop_to
            possible = False
            # first check which pt vehicles can take the request to make at destination on time
            walk_time = travel_time["%s-%s" % (stop2, req.ID)]
            latest_PT_arrival = req.dest_window[1] - walk_time
            try:
                stop2_vehicles = [stop.vehicle for stop in pt_stops if stop.stop == stop2 and req.orig_window[0] < stop.time <= latest_PT_arrival]
            except:
                continue
            for stop2_vehicle in stop2_vehicles:
                stop2_times = [stop.time for stop in pt_stops if stop.stop == stop2 and req.orig_window[0] < stop.time <= latest_PT_arrival]
                for stop2_time in stop2_times:
                    # search all usable PT stops
                    for stop1 in pt_stops:
                        if stop1.stop == stop2:
                            continue
                        if not pt_line in stop1.line:
                            continue
                        elif stop1.time > stop2_time:
                            continue
                        elif not stop1.edge in first_area_edges:
                            continue
                        elif not stop1.vehicle in stop2_vehicle:
                            continue
                        elif not req.orig_window[0] < (stop1.time - options.minwait):
                            continue
                        #if pt_line in stop1.line and stop1.edge in first_area_edges and stop1.vehicle in stop2_vehicle and req.orig_window[0] < (stop1.time - options.minwait):
                        # if stop is used by the pt line and if the stop is in the service area of the DRT and if the vehicle is possible and if the pt line stops at the stop after the earliest request depart
                        possible = True
                        string_pos = string_pos+1
                        first_dest_window = [max(req.orig_window[0], (stop1.time - options.maxwait)) , (stop1.time - options.minwait)]
                        first_orig_window = [req.orig_window[0], min(req.orig_window[1], first_dest_window[1])]
                        req.trips.append(req_trip("%i%s" % (req.ID, chr(string_pos)), "%i%s" % (req.ID, chr(string_pos)), req.orig_edge, stop1.edge, req.orig_pos, 10, req.depart, first_orig_window, first_dest_window, first_area, pt_line, stop2_vehicle, stop1.stop, stop2, "first"))
                        if first_dest_window[0] > first_dest_window[1]:
                            print("Bug: Destination window of request ", req.ID, " for Trip ", req.ID, "%i%s" % (req.ID, chr(string_pos)), "not possible. Latest time minor than earlier")

            if possible == False:
                print("Request %s could not be served. No combination between PT and DRT for first mile possible"% req.ID)
                req.rejected = True                
                req.rejected_cause = "No DRT-PT combination for first mile"
                                    
        elif last_mile > options.maxwalk:
            if second_area == None:
                print("Request %s could not be served. Last mile requiers DRT service and there is no service in the area"% req.ID)
                req.rejected = True
                req.rejected_cause = "No DRT service available in last mile area"
                continue
            
            # Max walking time from PT exceeded: DRT service from PT line to destination (last leg trip)
            stop1 = bus_stop_from
            possible = False
            # first check which PT vehicles can take the request, considering walking time to stop:
            walk_time = travel_time["%s-%s" % (req.ID, stop1)]
            earliest_PT_arrival = req.orig_window[0] + walk_time
            try:
                stop1_vehicles = [stop.vehicle for stop in pt_stops if stop.stop == stop1 and (earliest_PT_arrival + options.minwait) < stop.time < req.dest_window[1]]
            except:
                continue
            for stop1_vehicle in stop1_vehicles:
                stop1_times = [stop.time for stop in pt_stops if stop.stop == stop1 and (earliest_PT_arrival + options.minwait) < stop.time < req.dest_window[1]]
                for stop1_time in stop1_times:
                    # search all usable PT stops
                    for stop2 in pt_stops: 
                        if stop1 == stop2.stop:
                            continue
                        if not pt_line in stop2.line:
                            continue
                        if stop1_time > stop2.time:
                            continue
                        elif not stop2.edge in second_area_edges:
                            continue
                        elif not stop2.vehicle in stop1_vehicle:
                            continue
                        elif not (stop2.time + options.minwait) < req.dest_window[1]:
                            continue
                        #if pt_line in stop2.line and stop2.edge in second_area_edges and stop2.vehicle in stop1_vehicle and (stop2.time + options.minwait) < req.dest_window[1]:
                        # if stop is used by the pt line and if the stop is in the service area of the DRT and if vehicle is possible and if the pt line stops at the stop before the latest request drop-off
                        depart = [stop.time for stop in pt_stops if stop.stop == stop1 and stop.vehicle == stop2.vehicle][0] - walk_time - options.minwait
                        possible = True
                        string_pos = string_pos+1
                        last_orig_window = [(stop2.time + options.minwait) , min(stop2.time + options.maxwait, req.dest_window[1])]
                        last_dest_window = [max(last_orig_window[0], req.dest_window[0]), req.dest_window[1]]
                        req.trips.append(req_trip("%i%s" % (req.ID, chr(string_pos)), "%i%s" % (req.ID, chr(string_pos)), stop2.edge, req.dest_edge, 10, req.dest_pos, depart, last_orig_window, last_dest_window, second_area, pt_line, stop1_vehicle, stop1, stop2.stop, "last"))
                        if last_orig_window[0] > last_orig_window[1]:
                            print("Bug: Origin window of request ", req.ID, " for Trip ", "%i%s" % (req.ID, chr(string_pos)), req.ID, "not possible. Latest time minor than earlier")
                    
            if possible == False:
                print("Request %s could not be served. No combination between PT and DRT for last mile possible"% req.ID)
                req.rejected = True
                req.rejected_cause = "No DRT-PT combination for last mile"

def pair_possible(req1, req2, first_stop, second_stop, travel_time):
    # check if combination is possible     
    if first_stop.endswith("y"):
        first_orig_window = [trip.orig_window for trip in req1.trips if str(trip.orig_ID) == first_stop[:-1]]
        first_orig_window = first_orig_window[0]
        if first_stop[-2] is str:
            first_dest_window = [trip.dest_window for trip in req1.trips if str(trip.dest_ID) == first_stop[:-2]]
        else:
            first_dest_window = req1.dest_window
    elif first_stop.endswith("z"):
        first_dest_window = [trip.dest_window for trip in req1.trips if str(trip.dest_ID) == first_stop[:-1]]
        first_dest_window = first_dest_window[0]
        if first_stop[-2] is str:
            first_orig_window = [trip.orig_window for trip in req1.trips if str(trip.dest_ID) == first_stop[:-2]]
        else:
            first_orig_window = req1.orig_window
    
    if second_stop.endswith("y"):
        second_orig_window = [trip.orig_window for trip in req2.trips if str(trip.orig_ID) == second_stop[:-1]]
        second_orig_window = second_orig_window[0]
        if second_stop[-2] is str:
            second_dest_window = [trip.dest_window for trip in req2.trips if str(trip.dest_ID) == second_stop[:-2]]
        else:
            second_dest_window = req2.dest_window
    elif second_stop.endswith("z"):
        second_dest_window = [trip.dest_window for trip in req2.trips if str(trip.dest_ID) == second_stop[:-1]]
        second_dest_window = second_dest_window[0]    
        if second_stop[-2] is str:
            second_orig_window = [trip.orig_window for trip in req2.trips if str(trip.dest_ID) == second_stop[:-2]]
        else:
            second_orig_window = req2.orig_window    
    
    possible_pair = False
    if first_stop.endswith("y") and second_stop.endswith("y"):
        if (first_orig_window[0] + travel_time) <= second_orig_window[1] and \
            (first_orig_window[0] + travel_time) <= first_dest_window[1]:
        # if picking up req 1 at earliest time, req 2 can be pick up at least at latest time
        # and if combining this request, the latest drop off time of first request is not exceed then pair possible
            possible_pair = True
    elif first_stop.endswith("y") and second_stop.endswith("z"):
        if (first_orig_window[0] + travel_time) <= second_dest_window[1] and \
            (first_orig_window[0] + travel_time) <= first_dest_window[1]:
        # if picking up req 1 at earliest time, req 2 can be drop off at least at latest time
        # and if combining this requests, the latest drop off time of first request is not exceed then pair possible
            possible_pair = True
    elif first_stop.endswith("z") and second_stop.endswith("y"):
        if (first_dest_window[0] + travel_time) < second_orig_window[1]:
        # if droping off req 1 at earliest time, req 2 can be pick up at least at latest time, then pair possible
            possible_pair = True
    elif first_stop.endswith("z") and second_stop.endswith("z"):
        if (first_dest_window[0] + travel_time) <= second_dest_window[1] and \
            second_orig_window[0] <= first_dest_window[1]:
        # if droping off req 1 at earliest time, req 2 can be drop off at least at latest time
        # and if req 2 can be picked up before lastest dropp off of req 1, then pair possible
            possible_pair = True
    
    return possible_pair

def get_routes(options, requests, vehicles, RV_dic, RD_dic):

    # creates input file for duarouter
    with open("%saux_dua_input.xml" % options.path, "w+") as dua_file:
        # VR pair: combination between vehicle and request pick-up
        # RD pair: request to vehicle depot
        # RR pair: combination between two different requests

        # TODO this can be improve
        dua_file.write("<routes>\n")
        counter = 0 # avoid repeted route IDs
        for index1, req_t1 in enumerate(requests):  # for each request
            if req_t1.rejected == True:
                continue
            for req1 in req_t1.trips:
                # write SR (direct route) pair
                dua_file.write("""\t<trip id="SR%s-%sy_%sz" depart="0.0" departPos="%0.1f" from="%s" to="%s" 
                                arrivalPos="%0.1f" type="%s"/>\n""" % (counter, req1.orig_ID, req1.dest_ID, req1.orig_pos, req1.orig_edge,
                                                                        req1.dest_edge, req1.dest_pos, vehicles[0].type))
                # write VR (vehicle and request pick-up) and RD (request to vehicle depot) pairs
                for vehicle in vehicles:
                    if vehicle.area == req1.service_area:
                        dua_file.write("""\t<trip id="VR%s-%s_%sy" depart="0.0" departPos="%0.1f" from="%s" to="%s" 
                                        arrivalPos="%0.1f" type="%s"/>\n""" % (counter, vehicle.ID, req1.orig_ID, vehicle.depot_pos,
                                                                                vehicle.depot, req1.orig_edge, req1.orig_pos, vehicle.type))
                        dua_file.write("""\t<trip id="RD%s-%sz_%s" depart="0.0" departPos="%0.1f" from="%s" to="%s" 
                                        arrivalPos="%0.1f" type="%s"/>\n""" % (counter, req1.dest_ID, vehicle.ID, req1.orig_pos,
                                                                                req1.orig_edge, vehicle.depot,
                                                                                vehicle.depot_pos, vehicle.type))
                        counter += 1 # avoid repeted route IDs

                # write RR pairs ("0p1d", pick-up 0 and then deliver 1)
                for req_t2 in requests[index1 + 1:]:
                    if req_t2.rejected == True:
                        continue
                    for req2 in req_t2.trips:
                        if req1.service_area == req2.service_area:
                        # combination only possible if requests are in the same area
                            if req1.orig_window[0] <= req2.orig_window[1] and req2.orig_window[0] <= req1.dest_window[1]:
                            # if earliest pick up of req 1 before latest pick up time of req 2 and
                            # if earliest pick up time of req 2 before latest drop off of req 1: combination 1p2p possible
                                dua_file.write("""\t<trip id="RR%s-%sy_%sy" depart="0.0" departPos="%0.1f" from="%s" to="%s" 
                                                arrivalPos="%0.1f" type="%s"/>\n""" % (counter, req1.orig_ID, req2.orig_ID, req1.orig_pos,
                                                                                        req1.orig_edge, req2.orig_edge, req2.orig_pos, vehicle.type))                        
                            if req1.orig_window[0] <= req2.dest_window[1] and req2.dest_window[0] <= req1.dest_window[1] and req2.orig_window[0] <= req1.orig_window[1]:
                            # if earliest pick up of req 1 before latest drop off of req 2 and
                            # if earliest drop off of req 2 before latest drop off of req 1 and
                            # if earliest pick up of req 2 before latest pick up of req 1: combination 1p2d possible
                                dua_file.write("""\t<trip id="RR%s-%sy_%sz" depart="0.0" departPos="%0.1f" from="%s" to="%s" 
                                                arrivalPos="%0.1f" type="%s"/>\n""" % (counter, req1.orig_ID, req2.dest_ID, req1.orig_pos,
                                                                                        req1.orig_edge, req2.dest_edge, req2.dest_pos, vehicle.type))
                            if req1.dest_window[0] <= req2.orig_window[1]:
                            # if earliest drop off of req 1 before latest pick up of req 2: combination 1d2p possible
                                dua_file.write("""\t<trip id="RR%s-%sz_%sy" depart="0.0" departPos="%0.1f" from="%s" to="%s" 
                                                arrivalPos="%0.1f" type="%s"/>\n""" % (counter, req1.dest_ID, req2.orig_ID, req1.dest_pos,
                                                                                        req1.dest_edge, req2.orig_edge, req2.orig_pos, vehicle.type))
                            if req1.dest_window[0] <= req2.dest_window[1] and req2.orig_window[0] <= req1.dest_window[1]:
                            # if earliest drop off of req 1 before latest drop off of req 2 and 
                            # if earliest pick up of req 2 before latest drop off of req 1: combination 1d2d possible
                                dua_file.write("""\t<trip id="RR%s-%sz_%sz" depart="0.0" departPos="%0.1f" from="%s" to="%s" 
                                                arrivalPos="%0.1f" type="%s"/>\n""" % (counter, req1.dest_ID, req2.dest_ID, req1.dest_pos,
                                                                                        req1.dest_edge, req2.dest_edge, req2.dest_pos, vehicle.type))                        
                            if req2.orig_window[0] <= req1.orig_window[1] and req1.orig_window[0] <= req2.dest_window[1]:
                            # if earliest pick up of req 2 before latest pick up time of req 1 and
                            # if earliest pick up of req 1 before latest drop off time of req 2: combination 2p1p possible
                                dua_file.write("""\t<trip id="RR%s-%sy_%sy" depart="0.0" departPos="%0.1f" from="%s" to="%s" 
                                            arrivalPos="%0.1f" type="%s"/>\n""" % (counter, req2.orig_ID, req1.orig_ID, req2.orig_pos,
                                                                                    req2.orig_edge, req1.orig_edge, req1.orig_pos, vehicle.type))
                            if req2.orig_window[0] <= req1.dest_window[1] and req1.dest_window[0] <= req2.dest_window[1] and req1.orig_window[0] <= req2.orig_window[1]:
                            # if earliest pickup of req 2 before latest drop off of req 1 and
                            # if earliest drop off of req 1 before latest drop off of req 2 and
                            # if earliest pickup of req 1 before latest pickup of req 2: combination 2p1d possible
                                dua_file.write("""\t<trip id="RR%s-%sy_%sz" depart="0.0" departPos="%0.1f" from="%s" to="%s" 
                                            arrivalPos="%0.1f" type="%s"/>\n""" % (counter, req2.orig_ID, req1.dest_ID, req2.orig_pos,
                                                                                    req2.orig_edge, req1.dest_edge, req1.dest_pos, vehicle.type))
                            if req2.dest_window[0] <= req1.orig_window[1]:
                            # if earliest drop off of req 2 before latest pick up of req 1: combination 2d1p possible
                                dua_file.write("""\t<trip id="RR%s-%sz_%sy" depart="0.0" departPos="%0.1f" from="%s" to="%s" 
                                            arrivalPos="%0.1f" type="%s"/>\n""" % (counter, req2.dest_ID, req1.orig_ID, req2.dest_pos,
                                                                                    req2.dest_edge, req1.orig_edge, req1.orig_pos, vehicle.type))
                            if req2.dest_window[0] <= req1.dest_window[1] and req1.orig_window[0] <= req2.dest_window[1]:
                            # if earliest drop off of req 2 before latest drop off of req 1 and
                            # if earliest pick up of req 1 before latest drop off of req 2: combination 2d1d possible
                                dua_file.write("""\t<trip id="RR%s-%sz_%sz" depart="0.0" departPos="%0.1f" from="%s" to="%s" 
                                            arrivalPos="%0.1f" type="%s"/>\n""" % (counter, req2.dest_ID, req1.dest_ID, req2.dest_pos,
                                                                                    req2.dest_edge, req1.dest_edge, req1.dest_pos, vehicle.type))
                    
                        counter = counter + 1 # avoid repeted route IDs

        dua_file.write("</routes>\n")
    
    # runs Duarouter
    subprocess.call("duarouter -n %s --additional-files %s --route-files %saux_dua_input.xml -o %saux_dua_output.xml --no-warnings true"
                    % (options.network, options.additional, options.path, options.path))

    removed_trips = []
    # parse SR solutions:
    for trip, route in sumolib.xml.parse_fast_nested("%saux_dua_output.alt.xml" % options.path, "vehicle", ("id"), "route", ("cost", "edges")):        
        if "SR" not in trip.id:
            continue
        
        route_id = trip.id.split("-")[1]
        travel_time = int(float(route.cost))
        route_edges = route.edges.split(" ")        
        req_id = route_id.split("_")[1]
        req_id= [s for s in req_id if s.isdigit()]
        req_id = int(''.join(req_id))
        req =  [req for req in requests if req.ID == req_id][0] # request element

        # consider time at stop
        if len(route_edges) > 1:
            travel_u_stop_time = travel_time + options.stop_length
        else:
            # if stop are the same, consider stop time only once
            travel_u_stop_time = travel_time

        # parse SR routes (direct)
        # check if PT combination founded are possible
        for trip in req.trips:
            if (trip.orig_window[0] + travel_u_stop_time) > trip.dest_window[1]:
                # trip not possible
                removed_trips.append(trip.orig_ID)
                req.trips.remove(trip)
        if not req.trips:                
            req.rejected = True
            req.rejected_cause = "No DRT-PT combination possible"
        else:
            RV_dic[route_id] = (travel_u_stop_time, -req.pax, -req.pax_wc)

    # parse rest solutions:
    for trip, route in sumolib.xml.parse_fast_nested("%saux_dua_output.alt.xml" % options.path, "vehicle", ("id"), "route", ("cost", "edges")):        
        if "SR" in trip.id:
            continue
        
        route_id = trip.id.split("-")[1]

        # check if request rejected in SR parsing
        if route_id.split("_")[0][:-1] in removed_trips or route_id.split("_")[1][:-1] in removed_trips:
            continue

        if route_id in RV_dic:
            # route already parsed
            continue        
        
        travel_time = int(float(route.cost))
        route_edges = route.edges.split(" ")

        if "RD" in trip.id:
        # parse RD routes (request to vehicle depot)
            # check if request rejected in SR parsing            
            if route_id.split("_")[0][:-1] in removed_trips:
                continue
            if route_id not in RD_dic:
                RD_dic[route_id] = travel_time
            continue
        
        req_id = route_id.split("_")[1]
        req_id= [s for s in req_id if s.isdigit()]
        req_id = int(''.join(req_id))
        req =  [req for req in requests if req.ID == req_id][0] # request element

        # consider time at stop
        if len(route_edges) > 1:
            travel_u_stop_time = travel_time + options.stop_length
        else:
            # if stop are the same, consider stop time only once
            travel_u_stop_time = travel_time
        
        if "VR" in trip.id:
        # parse VR routes (vehicle and request pick-up)
            vehicle_id = route_id.split("_")[0]
            vehicle = [vehicle for vehicle in vehicles if vehicle.ID == vehicle_id][0]

            if vehicle.start_time:
                #if start time given, check if arrives on time
                earliest_arrive = vehicle.start_time + travel_u_stop_time
                if earliest_arrive > req.orig_window[1]:
                    continue
            RV_dic[route_id] = (travel_u_stop_time, req.pax, req.pax_wc)

        elif "RR" in trip.id:
        # parse RR routes (request-request)        
            first_stop, second_stop = route_id.split("_")            
            
            if first_stop in removed_trips or second_stop in removed_trips:
                continue
            
            first_request = str(re.split('y|z', first_stop)[0])
            first_request= [s for s in first_request if s.isdigit()]
            first_request = int(''.join(first_request))
            req1 = [req for req in requests if req.ID == first_request][0]
            
            second_request = str(re.split('y|z', second_stop)[0])
            second_request= [s for s in second_request if s.isdigit()]
            second_request = int(''.join(second_request))
            req2 = [req for req in requests if req.ID == second_request][0]

            # check if pair is possible, if not continue                    
            if not pair_possible(req1, req2, first_stop, second_stop, travel_time):
                continue

            if route_id.endswith("z"):  # at delivery passenger get off the car (-)
                RV_dic[route_id] = (travel_u_stop_time, -req2.pax, -req2.pax_wc)
            else:  # at pick-up passenger get in the car (+)
                RV_dic[route_id] = (travel_u_stop_time, req2.pax, req2.pax_wc)
    
    return RV_dic, RD_dic


def RV_offline(options, requests, vehicles, service_areas, pt_stops):  # calculates all request-request and request-vehicle combinations
    # warning object
    warning_info = []
    
    # dict with possible route pairs and request to depot
    RV_dic = {}
    RD_dic = {}
    
    # find direct routes and set time windows
    get_direct_routes(options, requests)

    # set trips info for each request according to the DRT service type
    # define tuple for trip info and service area
    req_trip = namedtuple('req_trip', ['orig_ID', 'dest_ID', 'orig_edge', 'dest_edge', 'orig_pos', 'dest_pos', 'depart', 'orig_window', 'dest_window', 'service_area', 'pt_line','pt_vehicle', 'bus_stop_from', 'bus_stop_to', "leg"])
    drt_service = namedtuple('DRT', ['area', 'leg'])
            
    if options.pt_stops:
    # if DRT as feeder system for PT
        # create dict with stop ids and edges
        unique_stops = {stop.stop: stop.edge for stop in pt_stops}

        get_pt_routes(requests, options, unique_stops, service_areas, req_trip, drt_service, pt_stops)

    elif options.service_area:
    # if DRT as independent system but working with service areas        
        for req in requests:
            areas = [area.name for area in service_areas if (req.dest_edge in area.edges) and (req.orig_edge in area.edges)]
            string_pos = 96 # add letter to id
            if not areas:
                # request origin and destination are not in DRT service areas: trip not possible
                warning_info.append("Request %s (id %s) could not be served. Origin and destination outside DRT service areas" %(req.name, req.ID))
                print("Request %s could not be served. Origin and destination outside DRT service areas" % req.ID)
                req.rejected = True
                req.rejected_cause = "No DRT service available in trip area"
            for area in areas:
                string_pos += 1
                req.trips.append(req_trip("%i%s" % (req.ID, chr(string_pos)), "%i%s" % (req.ID, chr(string_pos)), req.orig_edge, req.dest_edge, req.orig_pos, req.dest_pos, req.depart, req.orig_window, req.dest_window, area, None, None, None, None, None))
                
    else:
    # if DRT as independent system working in all net
        for req in requests:
            req.trips = [req_trip(req.ID, req.ID, req.orig_edge, req.dest_edge, req.orig_pos, req.dest_pos, req.depart, req.orig_window, req.dest_window, None, None, None, None, None, None)]
   
    # serach possible routes for all pair combinations
    RV_dic, RD_dic = get_routes(options, requests, vehicles, RV_dic, RD_dic)

    # Return pairwise Graph with travel times and passenger
    return RV_dic, RD_dic
