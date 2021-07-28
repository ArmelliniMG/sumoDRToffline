# (C) 2021 Armellini
# This code is licensed under MIT license (see LICENSE for details)

# @file    rtvGraph.py
# @author  Maria Giuliana Armellini
# @date    2020-01-01

# Search all feasible trips for each vehicle

import re
import psutil
import os
import time
from collections import namedtuple
import random


def search_trips(RV_dic, trip, RTV_graph, trips_tree, vehicle_bin, requests,
                 RD_dic, tree_branch, RTV_graph_element, options, vehicle,
                 hailing, depth_complete):

    trip_id = trip.trip_id
    next_R = trip_id.split("_")[-1]

    # search next possible destination based on the RV graph starting with
    # "next_R" sort RV_dic by shortest time, which increase the probability of
    # better route
    for item in sorted(RV_dic.items(), key=lambda item: item[1]):
        request_bin = [0] * len(requests)
        pair = item[0]
        if pair.startswith(next_R):
            if hailing:
                if next_R.endswith("y"):
                    if pair.endswith("y"):
                        continue
                    if next_R[:-1] != pair.split("_")[1][:-1]:
                        continue
                    trip_id_new = '%s_%s' % (trip_id, pair.split("_")[1])
                    already_RTV = any(trip_id_new in i.trip_id
                                      for i in RTV_graph)
                    if already_RTV:
                        continue
                elif next_R.endswith("z") and pair.endswith("z"):
                    continue
            req_id = str(pair.split("_")[1])
            req_id = [s for s in req_id if s.isdigit()]
            req_id = int(''.join(req_id))

            request = [r for r in requests if r.ID == req_id][0]
            sub_stop = "_%s" % pair.split("_")[1][:-1]

            requests_trip_pick = []
            requests_trip_drop = []
            trip_pick = [s for s in trip_id.split('_')[1:] if s.endswith("y")]
            trip_drop = [s for s in trip_id.split('_')[1:] if s.endswith("z")]
            for stop_pick in trip_pick:
                req_trip = [s for s in stop_pick if s.isdigit()]
                requests_trip_pick.append(int(''.join(req_trip)))
            for stop_drop in trip_drop:
                req_trip = [s for s in stop_drop if s.isdigit()]
                requests_trip_drop.append(int(''.join(req_trip)))

            if pair.endswith("z") and req_id not in requests_trip_pick:
                # picked up before delivery
                continue
            elif pair.endswith("y") and req_id in requests_trip_pick:
                # request already picked up
                continue
            elif pair.endswith("z") and req_id in requests_trip_drop:
                # request already droped off
                continue
            elif request.value != 0.5 and pair.endswith("z") and sub_stop not in trip_id:  # noqa
                # request subtrip different
                continue
            else:
                # trip leg possible
                # time window for arrival at stop
                trip_time_win = [trip.trip_time_win[0] + RV_dic[pair][0],
                                 trip.trip_time_win[1] + RV_dic[pair][0]]

                if pair.endswith("y"):
                    req_time_win = [req_trip.orig_window for req_trip in request.trips  # noqa
                                    if str(req_trip.orig_ID) == re.split('_|y|z', pair)[2]][0]  # noqa
                    requests_trip_pick.append(request.ID)
                elif pair.endswith("z"):
                    req_time_win = [req_trip.dest_window for req_trip in request.trips  # noqa
                                    if str(req_trip.dest_ID) == re.split('_|y|z', pair)[2]][0]  # noqa
                    requests_trip_drop.append(request.ID)

                check_window = trip_time_win[0] <= req_time_win[1]
                trip_passenger = trip.trip_passenger + RV_dic[pair][1]
                trip_wcpassenger = trip.trip_wcpassenger + RV_dic[pair][2]
                check_waiting = req_time_win[0] - trip_time_win[1]
                check_empty = (trip.trip_passenger + trip.trip_wcpassenger) == 0  # noqa

                if not check_window or trip_passenger > vehicle.cap or \
                   trip_wcpassenger > vehicle.cap_wc:
                    # basic constrains not fulfilled
                    continue

                trip_id_new = '%s_%s' % (trip_id, pair.split("_")[1])
                if req_time_win[0] <= trip_time_win[1]:
                    # if request time window in arrival time window
                    if trip_time_win[0] < req_time_win[0]:
                        # if earlier arrival minor than earlier request
                        # pick-up/delivery
                        trip_time_win[0] = req_time_win[0]  # update arrival
                    if trip_time_win[1] > req_time_win[1]:
                        # if latest arrival major than latest request
                        # pick-up/delivery
                        trip_time_win[1] = req_time_win[1]  # update arrival
                    trip_possible = True

                elif (check_waiting <= options.veh_wait) or \
                     (check_waiting > options.veh_wait and check_empty):
                    # if pick-up/delivery before minimum arrival time, than
                    # vehicle must wait
                    trip_time_win[0] = req_time_win[0]  # update arrival
                    # update latest arrival window
                    if check_empty:
                        trip_time_win[1] = req_time_win[1]
                    else:
                        trip_time_win[1] = min((trip_time_win[0] + options.veh_wait),  # noqa
                                               req_time_win[1])
                    trip_possible = True
                else:
                    if check_waiting < options.veh_wait and check_empty == 0:
                        # DEBUG only
                        print("Another time case found in RTV search tree for trip %s" % trip_id_new)  # noqa
                    continue

                # Check if vehicle can arrived on time at depot
                if vehicle.end_time <= trip_time_win[0]:
                    continue

                # Check if trip is complete
                if set(requests_trip_pick) != set(requests_trip_drop):
                    trip_complete = False
                else:
                    trip_complete = True

                # calculates stop times and waiting time
                stops = trip_id_new.split("_")
                stops.reverse()
                time_cum = trip_time_win[0]
                waiting_time = 0
                for index, stop in enumerate(stops):
                    if 0 < index < len(stops)-1:
                        # stops[index] is the stop before
                        pair = "%s_%s" % (stop, stops[index-1])
                        time_cum += -RV_dic[pair][0]
                        req_id = [s for s in stop if s.isdigit()]
                        req_id = int(''.join(req_id))
                        request = [r for r in requests if r.ID == req_id][0]
                        if stop.endswith("z"):
                            req_time_win = [req_trip.dest_window for req_trip in request.trips  # noqa
                                            if str(req_trip.dest_ID) == stop[:-1]][0]  # noqa
                            if not req_time_win[0] <= time_cum <= req_time_win[1]:  # noqa
                                # if not request.dest_window[0]
                                # <= time_cum <= request.dest_window[1]:
                                # Time window not satisfied:
                                if time_cum < req_time_win[0] and options.verbose:  # noqa
                                    print("check apecial case for dest window",
                                          req_time_win, time_cum)
                                # bus has to wait at stop until it can go to
                                # the next passenger to minimize the waiting
                                # time, take latest dropoff time not earliest
                                waiting_time += time_cum - req_time_win[1]
                                time_cum = req_time_win[1]
                            # request.check_tt[1] = time_cum # not possible
                            # to check max travel time

                        elif stop.endswith("y"):
                            req_time_win = [req_trip.orig_window for req_trip in request.trips  # noqa
                                            if str(req_trip.orig_ID) == stop[:-1]][0]  # noqa
                            if not req_time_win[0] <= time_cum <= req_time_win[1]:  # noqa
                                # if not request.orig_window[0] <= time_cum
                                # <= request.orig_window[1]:
                                # Time window not satisfy:
                                if time_cum < req_time_win[0] and options.verbose:  # noqa
                                    print("check apecial case for orig window",
                                          req_time_win, time_cum)
                                # bus has to wait at stop until it can go to
                                # the next passenger to minimize the waiting
                                # time, take latest pickup time not earliest
                                waiting_time += time_cum - req_time_win[1]
                                time_cum = req_time_win[1]
                            # TODO max travel time imposible to check:
                            # see notes in older versions
                    elif index == len(stops)-1:
                        # stops[index] is the stop before
                        pair = "%s_%s" % (stop, stops[index-1])
                        time_cum += -RV_dic[pair][0]
                        departure_time = time_cum
                        if departure_time < vehicle.start_time:
                            # earlier departure for bus exceeded,
                            # trip not possible
                            trip_possible = False
                            break

                trip_duration = trip_time_win[0] - departure_time - waiting_time # noqa
                for index, request in enumerate(requests):
                    if request.ID in requests_trip_pick:
                        request_bin[int(index)] = request.value

                similar_trips = [key for key in trips_tree
                                 if all([trip_id_new.split("_")[0] == key.trip_id.split("_")[0] and # noqa
                                 set(trip_id_new.split("_")) == set(key.trip_id.split("_")) and # noqa
                                 trip_id_new.split("_")[-1] == key.trip_id.split("_")[-1]])] # noqa
                for key in similar_trips:
                    if trip_duration+waiting_time+options.keep_trips >= key.trip_duration and\
                       trip_time_win[0] >= key.trip_time_win[0]:
                        trip_possible = False
                        break
                    # elif trip_duration <= key.trip_duration and
                    # trip_time_win[0] >= key.trip_time_win[0]:
                        # keep both
                    elif trip_duration < key.trip_duration and trip_time_win[0] <= key.trip_time_win[0]: # noqa
                        trips_tree.remove(key)

                if trip_possible is True:
                    if trip_complete is True:
                        # if trip is completed (all requests in trip picked up
                        # and delivered)
                        depth_complete = True
                        for index, request in enumerate(requests):
                            if request.ID in requests_trip_pick:
                                if request.value == 0.5:
                                    check_leg = [a for a in stops
                                                 if str(request.ID) in a and len(a) > 2]  # noqa
                                    if len(check_leg) > 2:
                                        print('Error line 192')
                                    if 'y' in check_leg[0]:
                                        check_leg = [a.leg for a in request.trips  # noqa
                                                     if a.orig_ID == check_leg[0][:-1]]  # noqa
                                    else:
                                        check_leg = [a.leg for a in request.trips  # noqa
                                                     if a.dest_ID == check_leg[0][:-1]]  # noqa
                                    if len(set(check_leg)) > 1:
                                        print('Error line 195')
                                    if check_leg[0] == 'first':
                                        request_bin[int(index)] = request.value - 0.75  # noqa
                                    else:
                                        request_bin[int(index)] = request.value + 0.75  # noqa
                                else:
                                    request_bin[int(index)] = request.value

                        to_depot = "%s_%s" % (trip_id_new.split("_")[-1],
                                              trip_id_new.split("_")[0])

                        if (trip_time_win[0] + RD_dic[to_depot]) <= vehicle.end_time:  # noqa
                            # Check if similar trip already founded and keep
                            # the better one
                            add_RTV = True
                            for key in RTV_graph:
                                if vehicle_bin == key.vehicle_bin and \
                                   request_bin == key.request_bin and \
                                   trip_id_new.split("_")[-1] == key.trip_id.split("_")[-1]:  # noqa
                                    if trip_duration+RD_dic[to_depot] >= key.trip_duration+300 and \
                                       departure_time >= key.departure_time+300:  # noqa
                                        add_RTV = False
                                        break
                                    else:
                                        RTV_graph.remove(key)
                                        add_RTV = True
                                        break
                            if add_RTV is False:
                                # if vehicle arrives at depot before end of
                                # operation time
                                continue

                            RTV_graph.append(RTV_graph_element(trip_id_new,
                                             departure_time,
                                             trip_duration + RD_dic[to_depot],
                                             vehicle_bin, request_bin,
                                             waiting_time))
                            # RTV not includes waiting time -> only minimize
                            # the time that the vehicle is actualy riding
                            trips_tree.append(tree_branch(trip_id_new,
                                              trip_duration + waiting_time,
                                              trip_time_win, trip_passenger,
                                              trip_wcpassenger))
                            # trips tree includes waiting time BUT not time to
                            # depot
                            if trip_time_win[0] > trip_time_win[1]:
                                # debug only
                                print("Trip Window not possible 2")
                    else:
                        # keep searching tree
                        trips_tree.append(tree_branch(trip_id_new,
                                          trip_duration + waiting_time,
                                          trip_time_win, trip_passenger,
                                          trip_wcpassenger))
                        if trip_time_win[0] > trip_time_win[1]:
                            # debug only
                            print("Trip Window not possible 3")

                # if trip_complete == True:
                #     # if trip is completed (all requests in trip
                #     # picked-up and delivered)
                #     for index, request in enumerate(requests):
                #         if request.ID in requests_trip_pick:
                #             request_bin[int(index)] = request.value

                #     departure_time = max(0, trip_time_win[0] - trip_duration,
                #                          vehicle.start_time)

                #     to_depot = "%s_%s" % (trip_id_new.split("_")[-1],
                #                           trip_id_new.split("_")[0])
                #     trip_duration = trip_duration + RD_dic[to_depot]

                #     if (departure_time + trip_duration) <= vehicle.end_time:
                #         # subtract waiting time from trip duration for only
                #         # minimize the time that vehicle is actually riding
                #         RTV_graph.append(RTV_graph_element(trip_id_new,
                #                          departure_time,
                #                          trip_duration-waiting_time,
                #                          vehicle_bin, request_bin,
                #                          waiting_time))
                #         trips_tree.append(tree_branch(trip_id_new,
                #                           trip_duration - RD_dic[to_depot],
                #                           trip_time_win, trip_passenger,
                #                           trip_wcpassenger, waiting_time))
                #         if trip_time_win[0] > trip_time_win[1]:
                #             print("Trip Window not possible")
                # elif trip_possible == True:  # keep searching tree
                #     trips_tree.append(tree_branch(trip_id_new, trip_duration,
                #                       trip_time_win, trip_passenger,
                #                       trip_wcpassenger, waiting_time))
                #     if trip_time_win[0] > trip_time_win[1]:
                #         print("Trip Window not possible")
    return trips_tree, RTV_graph, depth_complete


def RTV(options, RV_dic, vehicles, requests, RD_dic, service_areas):
    # RTV_graph_all = [None] * len(service_areas)
    RTV_graph = []
    RTV_graph_element = namedtuple('RTV_graph_element', ['trip_id', 'departure_time',  # noqa
                                                         'trip_duration', 'vehicle_bin',  # noqa
                                                         'request_bin', 'waiting_time'])  # noqa

    process = psutil.Process(os.getpid())
    memory_problems = 0

    # Avoid search trips for the "same" vehicles if they share area and depot
    vehicles_unique = []
    for vehicle in vehicles:
        if vehicles_unique:
            veh_double = [vehicle_compare for vehicle_compare in vehicles_unique  # noqa
                          if vehicle.depot == vehicle_compare.depot
                          and vehicle.area == vehicle_compare.area
                          and vehicle.cap == vehicle_compare.cap
                          and vehicle.cap_wc == vehicle_compare.cap_wc
                          and vehicle.start_time == vehicle_compare.start_time
                          and vehicle.end_time == vehicle_compare.end_time]
            if not veh_double:
                vehicles_unique.append(vehicle)
        else:
            vehicles_unique.append(vehicle)

    # Search possible trips with each vehicle:
    for vehicle in vehicles_unique:
        # RTV_graph = []
        vehicle_bin = [0] * len(vehicles)
        index = vehicles.index(vehicle)
        vehicle_bin[index] = 1
        p = 0  # tree depth
        trips_tree = [[]]
        tree_branch = namedtuple('tree_branch', ['trip_id', 'trip_duration',
                                                 'trip_time_win',
                                                 'trip_passenger',
                                                 'trip_wcpassenger'])

        for pair in RV_dic.keys():
            vehicle_id = pair.split("_")[0]
            if vehicle_id == vehicle.ID:
                req_id = str(re.split('_|y', pair)[1])  # TODO Check
                req_id = [s for s in req_id if s.isdigit()]
                req_id = int(''.join(req_id))
                request = [req for req in requests if req.ID == req_id][0]

                trip_duration = RV_dic[pair][0]  # trip time from BA1 to 0p
                if vehicle.start_time:
                    # if vehicle have an earliest start time from depot
                    trip_earliest = vehicle.start_time + trip_duration
                    # trip_time_win = [max(trip_earliest,
                    #                      request.orig_window[0]),
                    #                  request.orig_window[1]]
                    # time window for arrival at stop
                else:
                    trip_earliest = trip_duration
                    # trip_time_win = [request.orig_window[0],
                    #                  request.orig_window[1]]
                    # # time window for arrival at stop
                req_time_win = [req_trip.orig_window for req_trip in request.trips  # noqa
                                if str(req_trip.orig_ID) == pair.split("_")[1][:-1]][0]  # noqa
                # time window for arrival at stop
                trip_time_win = [max(trip_earliest, req_time_win[0]),
                                 req_time_win[1]]
                if trip_time_win[0] > trip_time_win[1]:
                    # Trip not possible, vehicle arrives at stop to late
                    if not request.rejected_cause:
                        request.rejected_cause = "Trip outside DRT service times"  # noqa
                    continue
                trip_passenger = RV_dic[pair][1]  # Passengers
                trip_wcpassenger = RV_dic[pair][2]  # special needs Passengers
                waiting_time = 0

                if trip_passenger < (vehicle.cap + 1) and \
                   trip_wcpassenger < (vehicle.cap_wc + 1):
                    # check max capacity (times not needed)
                    trips_tree[p].append(tree_branch(pair, trip_duration,
                                                     trip_time_win,
                                                     trip_passenger,
                                                     trip_wcpassenger))

        while trips_tree[p]:
            start_time = time.perf_counter()
            trips_tree.append([])
            # sort list according to trip duration, so the searching tree is
            # done starting with the shortest trips
            # trips_tree[p].sort(key = lambda x: (x.trip_duration*10 +
            # x.trip_time_win[1] - x.trip_id.count("z")*10000))
            # # consider the trips with complete trips first
            max_time_win = 1/max([x.trip_time_win[1] for x in trips_tree[p]])
            max_duration = 1/max([x.trip_duration for x in trips_tree[p]])
            max_served = 1/max(max([x.trip_id.count("z") for x in trips_tree[p]]),1)  # noqa
            trips_tree[p].sort(key=lambda x: (x.trip_duration * max_duration +
                                              x.trip_time_win[1] * max_time_win * 1.3 -  # noqa
                                              x.trip_id.count("z") * max_served))  # noqa
            depth_complete = None
            for index, trip in enumerate(trips_tree[p]):
                trips_tree[p + 1], RTV_graph,
                depth_complete = search_trips(RV_dic, trip, RTV_graph,
                                              trips_tree[p+1], vehicle_bin,
                                              requests, RD_dic, tree_branch,
                                              RTV_graph_element, options,
                                              vehicle, False, depth_complete)
                if (process.memory_info().rss*0.000001) > options.max_memory:
                    memory_problems = 1
                    if options.verbose:
                        print("Solution could maybe not be the optimal. Memory capacity exceeded for vehicle ",  # noqa
                              vehicle.ID, " at depth:", p)
                        print(index, "Trips have been considered")
                    break
                elif (time.perf_counter() - start_time) > options.max_time:
                    memory_problems = 1
                    if options.verbose:
                        print("Solution could maybe not be the optimal. Searching time for vehicle",  # noqa
                              vehicle.ID, "exceeded at depth:", p)
                        print(index, "Trips have been considered")
                    break
            # if (p+1)%2 == 1 and not depth_complete:
            #    # no need to keep searching in depth
            #    print("For vehicle %s trips possibles until %s requests" %
            #          (vehicle.ID, int(p/2)))
            #    break
            if p != 0:
                del trips_tree[p][:]
            p = p + 1  # next tree depth

        if memory_problems == 1 and options.ride_hailing:
            p = 0
            # search hailing
            print("\nSearch for ride hailing trips\n")
            while trips_tree[p]:
                start_time = time.perf_counter()
                trips_tree.append([])
                random.shuffle(trips_tree[p])
                # trips_tree[p].sort(key = lambda x: (-x.trip_id.count("z")))
                depth_complete = None
                for index, trip in enumerate(trips_tree[p]):
                    trips_tree[p + 1], RTV_graph,
                    depth_complete = search_trips(RV_dic, trip, RTV_graph,
                                                  trips_tree[p+1], vehicle_bin,
                                                  requests, RD_dic,
                                                  tree_branch,
                                                  RTV_graph_element, options,
                                                  vehicle, True,
                                                  depth_complete)
                    if (process.memory_info().rss*0.000001) > options.max_memory:  # noqa
                        memory_problems = 1
                        if options.verbose:
                            print("Solution could maybe not be the optimal. Memory capacity exceeded for vehicle ",  # noqa
                                  vehicle.ID, " at depth:", p)
                            print(index, "Trips have been considered")
                        break
                    elif (time.perf_counter() - start_time) > (options.max_time * 2):  # noqa
                        memory_problems = 1
                        if options.verbose:
                            print("Solution could maybe not be the optimal. Searching time for vehicle",  # noqa
                                  vehicle.ID, "exceeded at depth:", p)
                            print(index, "Trips have been considered")
                        break
                # if (p+1)%2 == 1 and not depth_complete:
                #    # no need to keep searching in depth
                #    print("For vehicle %s trips possibles until %s requests"
                #          % (vehicle.ID, int(p/2)))
                #    break
                del trips_tree[p][:]
                p = p + 1  # next tree depth
        else:
            del trips_tree[p][:]
        del trips_tree

        for vehicle_same in vehicles:
            if vehicle.ID != vehicle_same.ID and \
               vehicle.depot == vehicle_same.depot and \
               vehicle.area == vehicle_same.area and \
               vehicle.cap == vehicle_same.cap and \
               vehicle.cap_wc == vehicle_same.cap_wc and \
               vehicle.start_time == vehicle_same.start_time and \
               vehicle.end_time == vehicle_same.end_time:

                vehicle_bin = [0] * len(vehicles)
                index = vehicles.index(vehicle_same)
                vehicle_bin[index] = 1
                done = False
                for element in RTV_graph:
                    if element.trip_id.split("_")[0] == vehicle.ID:
                        done = True
                        trip_id = element.trip_id.replace(vehicle.ID,
                                                          vehicle_same.ID)
                        RTV_graph.append(RTV_graph_element(trip_id,
                                         element.departure_time,
                                         element.trip_duration, vehicle_bin,
                                         element.request_bin,
                                         element.waiting_time))
                    else:
                        if done is True:
                            break

    if memory_problems == 1:
        print("Search limit reach, problem will be solved approximately.")

    return RTV_graph, memory_problems
