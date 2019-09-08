#pragma once
#include "entity\Vehicle.h"
#include "entity\Bin.h"
#include "entity\Station.h"
#include "bpp.h"
#include "custom_hash.h"
#include "util.h" //���ߺ���
#include "rapidjson\document.h"
#include "rapidjson\writer.h"
#include "rapidjson\stringbuffer.h"
#include <iostream>
#include <tuple>
#include <unordered_set>
#include <ctime>
#include <cmath>

using namespace rapidjson;
using namespace my_util;

vector<Vehicle> used_vehicles;
unordered_map<string, Bin> bins;
unordered_map<string, Vehicle> unused_vehicles;
unordered_map<string, Station> stations;
unordered_map<pair<string, string>, double, pair_hash> distance_matrix;
unordered_map<pair<string, string>, double, pair_hash> load_time_matrix;

const int print_freq = 300;  // Local Search������
size_t current_iter = 1;
const int TABU_LENGTH = 10;
double ETA = 0.02;   //�ͷ�ϵ�� 

string move4_sid;

char* best_known_sol;
double best_known_cost;
double current_neighbour_cost;
double current_neighbour_cost_cmp;
string current_neighbour_move[6];
unordered_set<tuple<string, string, string>, tuple_hash3> my_tabuset1;
unordered_set<tuple<string, string, string, string>, tuple_hash4> my_tabuset2;
unordered_set<tuple<string, string, string>, tuple_hash3> my_tabuset3;

unordered_map<pair<string, string>, double, pair_hash> tabuset_in;  //��ֹ����(STATION_ID, VEHICLE_ID)
unordered_map<pair<string, string>, double, pair_hash> tabuset_out; //��ֹ�Ƴ�

unordered_map<pair<string, string>, int, pair_hash> times_in;    //(STATION_ID����VEHICLE_ID)�Ĵ���
unordered_map<pair<string, string>, int, pair_hash> times_out;   //(STATION_ID�뿪VEHICLE_ID)�Ĵ���

namespace ts {
	//��ʼ��
	char* initialize() {
		char* init_sol_json = readFileIntoString("result\\init_sol_928.json");
		resolve_sol(init_sol_json);
		for (auto& v : used_vehicles) {
			for (auto& s : stations) {
				tabuset_in.insert({ make_pair(s.first, v.get_id()), 0 });
				tabuset_out.insert({ make_pair(s.first, v.get_id()), 0 });
				times_in.insert({ make_pair(s.first, v.get_id()), 0 });
				times_out.insert({ make_pair(s.first, v.get_id()), 0 });
			}
		}
		return init_sol_json;
	}

	template <typename T>
	void update_single_tabuset(Vehicle v1, Vehicle v2, T tabuset) {
		auto it = tabuset.begin();
		while (it != tabuset.end()) {
			if (get<0>(*it) == v1.get_id() || get<1>(*it) == v1.get_id() \
				|| get<0>(*it) == v2.get_id() || get<1>(*it) == v2.get_id()) {
				it = tabuset.erase(it);
			}
			else {
				it++;
			}
		}
	}

	template <typename T>
	void update_single_tabuset(Vehicle v1, T tabuset) {
		auto it = tabuset.begin();
		while (it != tabuset.end()) {
			if (get<0>(*it) == v1.get_id() || get<1>(*it) == v1.get_id()) {
				it = tabuset.erase(it);
			}
			else {
				it++;
			}
		}
	}

	void update_tabusets(Vehicle v1) {
		update_single_tabuset(v1, my_tabuset1);
		update_single_tabuset(v1, my_tabuset2);
		update_single_tabuset(v1, my_tabuset3);
	}

	void update_tabusets(Vehicle v1, Vehicle v2) {
		update_single_tabuset(v1, v2, my_tabuset1);
		update_single_tabuset(v1, v2, my_tabuset2);
		update_single_tabuset(v1, v2, my_tabuset3);
	}

	bool move1(Station& s,
		Vehicle& v1,
		Vehicle& v2) {

		if (my_tabuset1.find(make_tuple(v1.get_id(), v2.get_id(), s.get_id())) != my_tabuset1.end())
			return false;
		vector<string> items1;
		vector<string> items1_left;
		for (string item : v1.loaded_items) {
			if (s.get_id() == bins.at(item).get_station())
				items1.push_back(item);
			else
				items1_left.push_back(item);
		}
		vector<string> items(items1);
		items.insert(items.end(), v2.loaded_items.begin(), v2.loaded_items.end());

		double total_area = 0, total_weight = 0;
		for (string bid : items) {
			total_area += bins.at(bid).get_area();
			total_weight += bins.at(bid).get_weight();
		}

		if (total_area <= v2.get_area() && total_weight <= v2.get_weight()) {
			vector<Bin> Bin_items;
			for (string bid : items)
				Bin_items.push_back(bins.at(bid));
			BPPManager M(v2.get_width(), v2.get_length());
			M.add_bins(Bin_items);

			if (M.checkbpp_sort_multi()) {
				double cost = current_neighbour_cost;
				//�Ƴ�V1��ʡ�ĳɱ�
				vector<string> &route = v1.visit_order;
				int route_size = route.size();
				if (route_size == 1)
					cost -= v1.get_flagdown_fare();
				else if (distance(route.begin(), find(route.begin(), route.end(), s.get_id())) == 0)
					cost -= v1.get_distance_fare() * distance_matrix.at(make_pair(route[0], route[1]));
				else if (distance(route.begin(), find(route.begin(), route.end(), s.get_id())) == route_size - 1)
					cost -= v1.get_distance_fare() * distance_matrix.at(make_pair(route[route_size - 2], route[route_size - 1]));
				else {
					int s_idx = distance(route.begin(), find(route.begin(), route.end(), s.get_id()));
					cost -= v1.get_distance_fare() * (distance_matrix.at(make_pair(route[s_idx - 1], route[s_idx])) + \
						distance_matrix.at(make_pair(route[s_idx], route[s_idx + 1])) - \
						distance_matrix.at(make_pair(route[s_idx - 1], route[s_idx + 1])));
				}


				//����V2���ӵĳɱ�
				double pre_distance = route_distance(v2.visit_order);
				vector<string> record_order = v2.visit_order;
				double smallest_distance = compute_tsp(v2.visit_order, s.get_id());
				v2.visit_order = record_order;



				if (smallest_distance != -1) {
					cost += (smallest_distance - pre_distance) * v2.get_distance_fare();

					if (cost >= current_neighbour_cost) {
						cost += ETA * cost * (times_in.at(make_pair(s.get_id(), v2.get_id())) + times_out.at(make_pair(s.get_id(), v1.get_id()))) / 2 / log(current_iter);
					}
					if (cost < current_neighbour_cost_cmp) {

						if (tabuset_out.at(make_pair(s.get_id(), v1.get_id())) <= current_iter &&
							tabuset_in.at(make_pair(s.get_id(), v2.get_id())) <= current_iter) {
							current_neighbour_cost_cmp = cost;
							current_neighbour_move[0] = "1";
							current_neighbour_move[1] = v1.get_id();
							current_neighbour_move[2] = v2.get_id();
							current_neighbour_move[3] = s.get_id();

							//std::cout << "���ʣ�" << s.get_id() << v1.get_id() << endl;
						}


						//to do ����ѧϰ����Ҫ��֤װ��
					/*	current_neighbour_cost = cost;
						std::cout << "New Neighbour Best:" << cost << endl;
						v1.visit_order.erase(find(v1.visit_order.begin(), v1.visit_order.end(), s.get_id()));
						v1.loaded_items = items1_left;
						v1.set_loaded_area(v1.get_loaded_area() - total_area + v2.get_loaded_area());
						v1.set_loaded_weight(v1.get_loaded_weight() - total_weight + v2.get_loaded_weight());
						M.return_seq(v2.loaded_items);
						v2.set_loaded_area(total_area);
						v2.set_loaded_weight(total_weight);
						s.discard(v1.get_id());
						s.pass_vehicles.insert(v2.get_id());
						if (v1.visit_order.empty()) {
							unused_vehicles.insert({ v1.get_id(), v1 });
							used_vehicles.erase(find(used_vehicles.begin(), used_vehicles.end(), v1));
						}*/
						if (cost < best_known_cost) {
							if (tabuset_out.at(make_pair(s.get_id(), v1.get_id())) > current_iter ||
								tabuset_in.at(make_pair(s.get_id(), v2.get_id())) > current_iter) {
								current_neighbour_cost_cmp = cost;
								current_neighbour_move[0] = "1";
								current_neighbour_move[1] = v1.get_id();
								current_neighbour_move[2] = v2.get_id();
								current_neighbour_move[3] = s.get_id();

								//std::cout << "���ʣ�" << s.get_id() << v1.get_id() << endl;
							}
							//std::cout << "New Best Cost:" << cost << endl;
							//best_known_cost = cost;
							//best_known_sol = save_sol("result\\init_sol.json");
							//int num_bins = cal_num_bins();
							//std::cout << "��ǰ�������� " << num_bins << endl;
						}
						//std::cout << "Current # of bins: " << cal_num_bins() << endl;
						return true;
					}
					else {
						v2.visit_order = record_order;
						return false;
					}
				}
			}
		}
		my_tabuset1.insert(make_tuple(v1.get_id(), v2.get_id(), s.get_id()));
		return false;
	}

	bool move2(Station& s1, Station& s2, Vehicle& v1, Vehicle& v2)
	{
		if (my_tabuset2.find(make_tuple(v1.get_id(), v2.get_id(), s1.get_id(), s2.get_id())) != my_tabuset2.end())
			return false;
		vector<string> items1;  // v1��s1ȫ�ƶ���V2
		vector<string> minitems1;  // v1����װ��
		vector<string> maxitems2;  // v2����װ��
		vector<string> minitems2;  // v2����װ��
		for (string item : v1.loaded_items) {
			if (s1.get_id() == bins.at(item).get_station())
				items1.push_back(item);
			else
				minitems1.push_back(item);
		}
		vector<string> items2; //v2��s2�ƶ���V1�ı�ѡ
		for (string item : v2.loaded_items) {
			if (s2.get_id() == bins.at(item).get_station())
				items2.push_back(item);
			else
				minitems2.push_back(item);
		}

		sort(items2.begin(), items2.end(), comp_desca_id);
		minitems2.insert(minitems2.end(), items1.begin(), items1.end());
		maxitems2.insert(maxitems2.end(), items1.begin(), items1.end());
		maxitems2.insert(maxitems2.end(), v2.loaded_items.begin(), v2.loaded_items.end());
		//��֤v2��װ������Ҫװ��
		vector<Bin> Bin_minitems2;
		for (string bid : minitems2)
			Bin_minitems2.push_back(bins.at(bid));
		BPPManager M2(v2.get_width(), v2.get_length());
		M2.add_bins(Bin_minitems2);

		if (!M2.checkbpp_sort_multi())
		{
			my_tabuset2.insert(make_tuple(v1.get_id(), v2.get_id(), s1.get_id(), s2.get_id()));
			return false;
		}

		double total_area1 = 0, total_weight1 = 0, total_area2 = 0, total_weight2 = 0;
		for (string bid : minitems1) {
			total_area1 += bins.at(bid).get_area();
			total_weight1 += bins.at(bid).get_weight();
		}
		for (string bid : maxitems2) {
			total_area2 += bins.at(bid).get_area();
			total_weight2 += bins.at(bid).get_weight();
		}
		size_t n_remove = 1;
		while (n_remove <= items2.size())
		{
			double temp_weight = 0, temp_area = 0;
			for (size_t i = 0; i < n_remove; i++)
			{
				temp_area += bins.at(items2[i]).get_area();
				temp_weight += bins.at(items2[i]).get_weight();
			}

			if ((total_area1 + temp_area) < v1.get_area() && (total_weight1 + temp_weight) < v1.get_weight()
				&& (total_area2 - temp_area) < v2.get_area() && (total_weight2 - temp_weight) < v2.get_weight())
			{
				M2.clear_bin();
				vector<Bin> Bin_tempitems2;
				for (string bid : maxitems2)
				{
					if (find(items2.begin(), items2.begin() + n_remove, bid) == (items2.begin() + n_remove))
						Bin_tempitems2.push_back(bins.at(bid));

				}
				M2.add_bins(Bin_tempitems2);
				if (M2.checkbpp_sort_multi())
				{
					BPPManager M1(v1.get_width(), v1.get_length());
					vector<Bin> Bin_tempitems1;
					for (string bid : minitems1)
						Bin_tempitems1.push_back(bins.at(bid));
					for (size_t j = 0; j < n_remove; j++)
						Bin_tempitems1.push_back(bins.at(items2[j]));
					M1.add_bins(Bin_tempitems1);

					if (M1.checkbpp_sort_multi())
					{

						double cost = current_neighbour_cost;
						//v1 v2�ɱ��仯
						vector<string> temp_visit1 = v1.visit_order;
						vector<string> temp_visit2 = v2.visit_order;
						temp_visit1.erase(find(temp_visit1.begin(), temp_visit1.end(), s1.get_id()));
						double pre_distance1 = route_distance(v1.visit_order);
						double smallest_distance1 = compute_tsp(temp_visit1, s2.get_id());
						if (n_remove == items2.size())
							temp_visit2.erase(find(temp_visit2.begin(), temp_visit2.end(), s2.get_id()));

						if (find(temp_visit2.begin(), temp_visit2.end(), s1.get_id()) == temp_visit2.end())
							temp_visit2.push_back(s1.get_id());
						double pre_distance2 = route_distance(v2.visit_order);
						double smallest_distance2 = compute_tsp(temp_visit2);


						if (smallest_distance1 != -1 && smallest_distance2 != -1)
						{
							cost += (smallest_distance1 - pre_distance1) * v1.get_distance_fare();
							cost += (smallest_distance2 - pre_distance2) * v2.get_distance_fare();
							if (cost >= current_neighbour_cost) {
								cost += ETA * cost * (times_in.at(make_pair(s1.get_id(), v2.get_id())) +
									times_out.at(make_pair(s1.get_id(), v1.get_id())) +
									times_out.at(make_pair(s2.get_id(), v2.get_id())) +
									times_in.at(make_pair(s2.get_id(), v1.get_id()))) / 4 / log(current_iter);
							}
							if (cost < current_neighbour_cost_cmp)
							{

								if (tabuset_out.at(make_pair(s1.get_id(), v1.get_id())) <= current_iter &&
									tabuset_out.at(make_pair(s2.get_id(), v2.get_id())) <= current_iter &&
									tabuset_in.at(make_pair(s1.get_id(), v2.get_id())) <= current_iter &&
									tabuset_in.at(make_pair(s2.get_id(), v1.get_id())) <= current_iter) {
									current_neighbour_cost_cmp = cost;
									current_neighbour_move[0] = "2";
									current_neighbour_move[1] = v1.get_id();
									current_neighbour_move[2] = v2.get_id();
									current_neighbour_move[3] = s1.get_id();
									current_neighbour_move[4] = s2.get_id();
									current_neighbour_move[5] = to_string(n_remove);


									//std::cout << "���ʣ�" << s1.get_id() << v1.get_id() << endl;
								}
								/*	current_neighbour_cost = cost;
									std::cout << "New Neighbour Best:" << cost << endl;
									v1.visit_order = temp_visit1;
									M1.return_seq(v1.loaded_items);

									v1.set_loaded_area(total_area1 + temp_area);
									v1.set_loaded_weight(total_weight1 + temp_weight);
									v2.visit_order = temp_visit2;
									M2.return_seq(v2.loaded_items);
									v2.set_loaded_area(total_area2 - temp_area);
									v2.set_loaded_weight(total_weight2 - temp_weight);
									s1.discard(v1.get_id());
									if (n_remove == items2.size())
										s2.pass_vehicles.erase(v2.get_id());
									s2.pass_vehicles.insert(v1.get_id());*/
								if (cost < best_known_cost) {
									/*		std::cout << "New Best Cost:" << cost << endl;
											best_known_cost = cost;
											best_known_sol = save_sol("result\\init_sol.json");
											std::cout << "best��¼����������" << cal_num_bins() << endl;*/
									if (tabuset_out.at(make_pair(s1.get_id(), v1.get_id())) > current_iter ||
										tabuset_out.at(make_pair(s2.get_id(), v2.get_id())) > current_iter ||
										tabuset_in.at(make_pair(s1.get_id(), v2.get_id())) > current_iter ||
										tabuset_in.at(make_pair(s2.get_id(), v1.get_id())) > current_iter) {
										current_neighbour_cost_cmp = cost;
										current_neighbour_move[0] = "2";
										current_neighbour_move[1] = v1.get_id();
										current_neighbour_move[2] = v2.get_id();
										current_neighbour_move[3] = s1.get_id();
										current_neighbour_move[4] = s2.get_id();
										current_neighbour_move[5] = to_string(n_remove);
										//std::cout << "���ʣ�" << s1.get_id() << v1.get_id() << endl;
									}
								}
								return true;
							}
							else {
								if (n_remove == items2.size())
									return false;
							}
						}
						else
							break;
					}
				}
			}
			n_remove += 1;
		}
		my_tabuset2.insert(make_tuple(v1.get_id(), v2.get_id(), s1.get_id(), s2.get_id()));
		return false;
	}

	bool move3(Station& s,
		Vehicle& v1,
		Vehicle& v2) {

		if (my_tabuset3.find(make_tuple(v1.get_id(), v2.get_id(), s.get_id())) != my_tabuset3.end())
			return false;
		vector<string> items1;    //v1��վ��s������
		vector<string> items1_left;   //v1�в�����վ��s������
		for (string item : v1.loaded_items) {
			if (s.get_id() == bins.at(item).get_station())
				items1.push_back(item);
			else
				items1_left.push_back(item);
		}
		double s1_area = 0, s1_weight = 0;
		for (auto bid : items1) {
			s1_area += bins.at(bid).get_area();
			s1_weight += bins.at(bid).get_weight();
		}

		for (string sid2 : v2.visit_order) {
			if (sid2 == s.get_id())
				continue;
			if (stations.at(sid2).get_limit() < v1.get_length())
				continue;
			vector<string> items2;       //v2��վ��s2������
			vector<string> items2_left;  //v2�в�����վ��s2������
			for (string item : v2.loaded_items) {
				if (sid2 == bins.at(item).get_station())
					items2.push_back(item);
				else
					items2_left.push_back(item);
			}

			double s2_area = 0, s2_weight = 0;
			for (auto bid : items2) {
				s2_area += bins.at(bid).get_area();
				s2_weight += bins.at(bid).get_weight();
			}

			vector<string> total_items1(items1_left);
			total_items1.insert(total_items1.end(), items2.begin(), items2.end());


			double total_area1 = v1.get_loaded_area() - s1_area + s2_area;
			double total_weight1 = v1.get_loaded_weight() - s1_weight + s2_weight;
			if (total_area1 <= v1.get_area() && total_weight1 <= v1.get_weight()) {
				vector<Bin> Bin_total_items1;
				for (string bid : total_items1) {
					Bin_total_items1.push_back(bins.at(bid));
				}
				BPPManager M1(v1.get_width(), v1.get_length());
				M1.add_bins(Bin_total_items1);
				if (M1.checkbpp_sort_multi()) {
					vector<string> total_items2(items2_left);
					total_items2.insert(total_items2.end(), items1.begin(), items1.end());
					double total_area2 = v2.get_loaded_area() - s2_area + s1_area;
					double total_weight2 = v2.get_loaded_weight() - s2_weight + s1_weight;
					if (total_area2 <= v2.get_area() && total_weight2 <= v2.get_weight()) {
						vector<Bin> Bin_total_items2;
						for (string bid : total_items2) {
							Bin_total_items2.push_back(bins.at(bid));
						}
						BPPManager M2(v2.get_width(), v2.get_length());
						M2.add_bins(Bin_total_items2);
						if (M2.checkbpp_sort_multi()) {
							double cost = current_neighbour_cost;
							vector<string> curr_route1 = { sid2 };
							if (v1.visit_order.size() > 1) {
								double pre_distance1 = route_distance(v1.visit_order);
								curr_route1 = v1.visit_order;
								curr_route1.erase(find(curr_route1.begin(), curr_route1.end(), s.get_id()));
								curr_route1.push_back(sid2);
								double smallest_dist1 = compute_tsp(curr_route1);
								if (smallest_dist1 != -1)
									cost += (smallest_dist1 - pre_distance1) * v1.get_distance_fare();
								else
									continue;
							}
							double pre_distance2 = route_distance(v2.visit_order);
							vector<string> curr_route2 = v2.visit_order;
							curr_route2.erase(find(curr_route2.begin(), curr_route2.end(), sid2));
							curr_route2.push_back(s.get_id());
							double smallest_dist2 = compute_tsp(curr_route2);


							if (smallest_dist2 != -1) {
								cost += (smallest_dist2 - pre_distance2) * v2.get_distance_fare();
								if (cost >= current_neighbour_cost) {
									cost += ETA * cost * (times_in.at(make_pair(s.get_id(), v2.get_id())) +
										times_in.at(make_pair(sid2, v1.get_id())) +
										times_out.at(make_pair(s.get_id(), v1.get_id())) +
										times_out.at(make_pair(sid2, v2.get_id()))) / 4 / log(current_iter);
								}
								if (cost < current_neighbour_cost_cmp) {
									if (tabuset_out.at(make_pair(s.get_id(), v1.get_id())) <= current_iter &&
										tabuset_out.at(make_pair(sid2, v2.get_id())) <= current_iter &&
										tabuset_in.at(make_pair(s.get_id(), v2.get_id())) <= current_iter &&
										tabuset_in.at(make_pair(sid2, v1.get_id())) <= current_iter) {
										current_neighbour_cost_cmp = cost;
										current_neighbour_move[0] = "3";
										current_neighbour_move[1] = v1.get_id();
										current_neighbour_move[2] = v2.get_id();
										current_neighbour_move[3] = s.get_id();
										current_neighbour_move[4] = sid2;

										//std::cout << "���ʣ�" << s.get_id() << v1.get_id() << endl;
									}

									//current_neighbour_cost = cost;
									//std::cout << "New Neighbour Best:" << cost << endl;
									//v1.visit_order = curr_route1;
									//v2.visit_order = curr_route2;
									//M1.return_seq(v1.loaded_items);
									//M2.return_seq(v2.loaded_items);
									//v1.set_loaded_area(total_area1);
									//v2.set_loaded_area(total_area2);
									//v1.set_loaded_weight(total_weight1);
									//v2.set_loaded_weight(total_weight2);
									//s.pass_vehicles.insert(v2.get_id());
									//s.discard(v1.get_id());
									//Station &s2 = stations.at(sid2);
									//s2.pass_vehicles.insert(v1.get_id());
									//s2.discard(sid2);
									if (cost < best_known_cost) {
										/*		std::cout << "New Best Cost:" << cost << endl;
												best_known_cost = cost;
												best_known_sol = save_sol("result\\init_sol.json");*/
										if (tabuset_out.at(make_pair(s.get_id(), v1.get_id())) > current_iter ||
											tabuset_out.at(make_pair(sid2, v2.get_id())) > current_iter ||
											tabuset_in.at(make_pair(s.get_id(), v2.get_id())) > current_iter ||
											tabuset_in.at(make_pair(sid2, v1.get_id())) > current_iter) {
											current_neighbour_cost_cmp = cost;
											current_neighbour_move[0] = "3";
											current_neighbour_move[1] = v1.get_id();
											current_neighbour_move[2] = v2.get_id();
											current_neighbour_move[3] = s.get_id();
											current_neighbour_move[4] = sid2;
											//std::cout << "���ʣ�" << s.get_id() << v1.get_id() << endl;
										}
									}
									return true;
								}
								else {
									return false;
								}
							}
						}
					}
				}

			}

		}
		my_tabuset3.insert(make_tuple(v1.get_id(), v2.get_id(), s.get_id()));
		return false;
	}

	bool move4(Vehicle v1) {
		//��� tabuset
		string sid = v1.visit_order[0];


		double cost = current_neighbour_cost;
		vector<string> vids;
		for (auto& vid : stations.at(sid).pass_vehicles)
		{
			if (vid == v1.get_id())
				continue;
			double destv_wid,destv_len;
			vector<string> destv_loaded_items;
			for (auto vvv : used_vehicles)
			{
				if (vvv.get_id() == vid)
				{
					destv_wid = vvv.get_width();
					destv_len = vvv.get_length();
					destv_loaded_items = vvv.loaded_items;
					break;
				}
			}
			BPPManager M(destv_wid, destv_len);
			vector<Bin> Bin_total_items;
			for (string bid : destv_loaded_items) {
				Bin_total_items.push_back(bins.at(bid));
			}
			M.add_bins(Bin_total_items);
			for (size_t idx = 0; idx < v1.loaded_items.size(); idx++) {
				M.update_backup();
				M.add_bin(bins.at(v1.loaded_items[idx]));
				if (M.checkbpp_sort_multi()) {
					VectorRemoveAt(v1.loaded_items, idx);
					idx--;
				}
				else
					M.restore();
			}
			if (destv_loaded_items.size() != M.get_bins_size()) {
				vids.push_back(vid);
			}
		}
		for (auto v : used_vehicles) {
			if (stations.at(sid).pass_vehicles.find(v.get_id()) != stations.at(sid).pass_vehicles.end())
				continue;
			if (v.get_id() == v1.get_id())
				continue;
			if (v.get_length() > stations.at(sid).get_limit())
				continue;
			if (v.occupancy() > 0.88 && find(v.visit_order.begin(), v.visit_order.end(), sid) == v.visit_order.end())
				continue;
			else {
				BPPManager M(v.get_width(), v.get_length());
				vector<Bin> Bin_total_items;
				for (string bid : v.loaded_items) {
					Bin_total_items.push_back(bins.at(bid));
				}
				M.add_bins(Bin_total_items);

				double pre_dist = route_distance(v.visit_order);
				double smallest_dist = pre_dist;
				if (find(v.visit_order.begin(), v.visit_order.end(), sid) == v.visit_order.end()) {
					smallest_dist = compute_tsp(v.visit_order, sid);
					if (smallest_dist == -1)
						continue;
				}

				for (size_t idx = 0; idx < v1.loaded_items.size(); idx++) {
					M.update_backup();
					M.add_bin(bins.at(v1.loaded_items[idx]));
					if (M.checkbpp_sort_multi()) {
						VectorRemoveAt(v1.loaded_items, idx);
						idx--;
					}
					else
						M.restore();
				}

				/*for (size_t idx = v1.loaded_items.size() - 1; idx > -1; --idx) {
					M.update_backup();
					M.add_bin(bins.at(v1.loaded_items[idx]));
					cout << 2222222222 << endl;
					if (M.checkbpp_sort_multi())
						VectorRemoveAt(v1.loaded_items, idx);
					else
						M.restore();
				}*/

				if (v.loaded_items.size() != M.get_bins_size()) {
					//cout << 1111111111111111 << endl;

					cost += (smallest_dist - pre_dist) * v.get_distance_fare();
					vids.push_back(v.get_id());
				}
			}
		}

		
		if (v1.loaded_items.size() == 0)
			cost -= v1.get_flagdown_fare();
		double penalty = 0;
		if (cost >= current_neighbour_cost) {
			double total_times = times_out.at(make_pair(sid, v1.get_id()));
			for (string vid : vids) {
				total_times += times_in.at(make_pair(sid, vid));
			}
			if (total_times > 0)
			{
				cost += ETA * cost * total_times / vids.size() / log(current_iter);
				penalty = ETA * cost * total_times / vids.size() / log(current_iter);
			}
		}

		if (cost < current_neighbour_cost_cmp) {
			double realcost = cost - penalty;
			cout << realcost << endl;

			current_neighbour_cost_cmp = cost;
			current_neighbour_move[0] = "4";
			current_neighbour_move[1] = v1.get_id();
		}
	}

	void LS1() {
		int count = 0;
		for (auto& v1 : used_vehicles) {
			if (count % print_freq == 0)
				std::cout << "local search 1 on going: " << count << endl;
			count++;
			if (v1.get_id() == "V764")
				print_vector(v1.visit_order);
			for (string sid : v1.visit_order) {
				Station& s = stations.at(sid);
				for (Vehicle& v2 : used_vehicles) {
					if (v2.get_id() != v1.get_id() && v2.get_length() <= s.get_limit())
						move1(s, v1, v2);
				}
			}
		}
	}

	void LS2()
	{
		int count = 0;
		for (auto& v1 : used_vehicles)
		{
			if (count % print_freq == 0)
				std::cout << "local search 2 on going: " << count << endl;
			count++;
			for (auto &s1id : v1.visit_order)
			{
				Station& s1 = stations.at(s1id);
				for (auto& v2id : s1.pass_vehicles)
				{
					if (v2id == v1.get_id())
						continue;
					size_t index2 = 0;
					for (; index2 < used_vehicles.size(); index2++)
					{
						if (used_vehicles[index2].get_id() == v2id)
							break;
					}
					Vehicle& v2 = used_vehicles[index2];
					if ((v1.visit_order.size() == 1 && v2.visit_order.size() == 1))
						continue;
					for (auto &s2id : v2.visit_order)
					{
						if (s2id == s1.get_id())
							continue;
						Station& s2 = stations.at(s2id);
						if (v1.get_length() <= s2.get_limit())
							move2(s1, s2, v1, v2);
					}
				}
			}
		}
	}

	void LS3() {
		int count = 0;
		for (auto& v1 : used_vehicles) {
			if (count % print_freq == 0)
				std::cout << "local search 3 on going: " << count << endl;
			count++;
			for (auto &sid : v1.visit_order) {
				Station& s = stations.at(sid);
				for (auto& v2 : used_vehicles) {
					if (v2.get_id() != v1.get_id() && v2.get_length() <= s.get_limit() && v2.visit_order.size() > 1)
						move3(s, v1, v2);
				}
			}
		}
	}

	void LS4() {
		int count = 0;
		sort(used_vehicles.begin(), used_vehicles.end(), comp_veh_free_area);
		for (auto& v1 : used_vehicles) {
			if (count % print_freq == 0)
				std::cout << "local search 4 on going: " << count << endl;
			count++;
			if (v1.visit_order.size() > 1)
				continue;
			
			move4(v1);
		}
	}

	bool real_move1(Station& s,
		Vehicle& v1,
		Vehicle& v2) {/*
		std::cout << "move1 ǰv1��"; print_vector(v1.visit_order);
		std::cout << "move1 ǰv2��"; print_vector(v2.visit_order);
		std::cout << "���ƶ�վ��: " << s.get_id() << endl;*/
		vector<string> items1;
		vector<string> items1_left;
		for (string item : v1.loaded_items) {
			if (s.get_id() == bins.at(item).get_station())
				items1.push_back(item);
			else
				items1_left.push_back(item);
		}
		vector<string> items(items1);
		items.insert(items.end(), v2.loaded_items.begin(), v2.loaded_items.end());

		double total_area = 0, total_weight = 0;
		for (string bid : items) {
			total_area += bins.at(bid).get_area();
			total_weight += bins.at(bid).get_weight();
		}

		vector<Bin> Bin_items;
		for (string bid : items)
			Bin_items.push_back(bins.at(bid));
		BPPManager M(v2.get_width(), v2.get_length());
		M.add_bins(Bin_items);

		if (M.checkbpp_sort_multi()) {
			double cost = current_neighbour_cost;
			//�Ƴ�V1��ʡ�ĳɱ�
			vector<string> &route = v1.visit_order;
			int route_size = route.size();
			if (route_size == 1)
				cost -= v1.get_flagdown_fare();
			else if (distance(route.begin(), find(route.begin(), route.end(), s.get_id())) == 0)
				cost -= v1.get_distance_fare() * distance_matrix.at(make_pair(route[0], route[1]));
			else if (distance(route.begin(), find(route.begin(), route.end(), s.get_id())) == route_size - 1)
				cost -= v1.get_distance_fare() * distance_matrix.at(make_pair(route[route_size - 2], route[route_size - 1]));
			else {
				int s_idx = distance(route.begin(), find(route.begin(), route.end(), s.get_id()));
				cost -= v1.get_distance_fare() * (distance_matrix.at(make_pair(route[s_idx - 1], route[s_idx])) + \
					distance_matrix.at(make_pair(route[s_idx], route[s_idx + 1])) - \
					distance_matrix.at(make_pair(route[s_idx - 1], route[s_idx + 1])));
			}

			//����V2���ӵĳɱ�
			double pre_distance = route_distance(v2.visit_order);
			vector<string> record_order = v2.visit_order;
			double smallest_distance = compute_tsp(v2.visit_order, s.get_id());

			cost += (smallest_distance - pre_distance) * v2.get_distance_fare();
			//to do ����ѧϰ����Ҫ��֤װ��
			if (cost >= current_neighbour_cost)
				std::cout << "�ͷ��ɱ�: " << ETA * cost * (times_in.at(make_pair(s.get_id(), v2.get_id())) + times_out.at(make_pair(s.get_id(), v1.get_id()))) / 2 / log(current_iter) << endl;

			current_neighbour_cost_cmp = 9999999;
			current_neighbour_cost = cost;
			std::cout << "New Neighbour Best:" << cost << endl;
			v1.visit_order.erase(find(v1.visit_order.begin(), v1.visit_order.end(), s.get_id()));
			v1.loaded_items = items1_left;
			v1.set_loaded_area(v1.get_loaded_area() - total_area + v2.get_loaded_area());
			v1.set_loaded_weight(v1.get_loaded_weight() - total_weight + v2.get_loaded_weight());
			M.return_seq(v2.loaded_items);
			v2.set_loaded_area(total_area);
			v2.set_loaded_weight(total_weight);
			s.discard(v1.get_id());
			s.pass_vehicles.insert(v2.get_id());
			if (v1.visit_order.empty()) {
				unused_vehicles.insert({ v1.get_id(), v1 });
				used_vehicles.erase(find(used_vehicles.begin(), used_vehicles.end(), v1));
			}
			std::cout << "Move1 Real New Neighbour Best:" << cal_total_cost() << endl;

			if (cost < best_known_cost) {
				std::cout << "New Best Cost:" << cost << endl;
				best_known_cost = cost;
				best_known_sol = save_sol("result\\init_sol.json");
				int num_bins = cal_num_bins();
				std::cout << "��ǰ�������� " << num_bins << endl;

			}

			//std::cout << "move1 ��v1��"; print_vector(v1.visit_order);
			//std::cout << "move1 ��v2��"; print_vector(v2.visit_order);

			return true;
		}

		return false;
	}

	bool real_move2(Station& s1, Station& s2, Vehicle& v1, Vehicle& v2, int n_remove)
	{
		vector<string> items1;  // v1��s1ȫ�ƶ���V2
		vector<string> minitems1;  // v1����װ��
		vector<string> maxitems2;  // v2����װ��
		vector<string> minitems2;  // v2����װ��
		/*std::cout << "move2 ǰv1��" << v1.visit_order.size() << endl;
		std::cout << "move2 ǰv2��" << v2.visit_order.size() << endl;*/

		for (string item : v1.loaded_items) {
			if (s1.get_id() == bins.at(item).get_station())
				items1.push_back(item);
			else
				minitems1.push_back(item);
		}
		vector<string> items2; //v2��s2�ƶ���V1�ı�ѡ
		for (string item : v2.loaded_items) {
			if (s2.get_id() == bins.at(item).get_station())
				items2.push_back(item);
			else
				minitems2.push_back(item);
		}

		sort(items2.begin(), items2.end(), comp_desca_id);
		minitems2.insert(minitems2.end(), items1.begin(), items1.end());
		maxitems2.insert(maxitems2.end(), items1.begin(), items1.end());
		maxitems2.insert(maxitems2.end(), v2.loaded_items.begin(), v2.loaded_items.end());
		//��֤v2��װ������Ҫװ��
		vector<Bin> Bin_minitems2;
		for (string bid : minitems2)
			Bin_minitems2.push_back(bins.at(bid));
		BPPManager M2(v2.get_width(), v2.get_length());
		M2.add_bins(Bin_minitems2);

		double total_area1 = 0, total_weight1 = 0, total_area2 = 0, total_weight2 = 0;
		for (string bid : minitems1) {
			total_area1 += bins.at(bid).get_area();
			total_weight1 += bins.at(bid).get_weight();
		}
		for (string bid : maxitems2) {
			total_area2 += bins.at(bid).get_area();
			total_weight2 += bins.at(bid).get_weight();
		}

		double temp_weight = 0, temp_area = 0;
		for (size_t i = 0; i < n_remove; i++)
		{
			temp_area += bins.at(items2[i]).get_area();
			temp_weight += bins.at(items2[i]).get_weight();
		}


		M2.clear_bin();
		vector<Bin> Bin_tempitems2;
		for (string bid : maxitems2)
		{
			if (find(items2.begin(), items2.begin() + n_remove, bid) == (items2.begin() + n_remove))
				Bin_tempitems2.push_back(bins.at(bid));

		}
		M2.add_bins(Bin_tempitems2);
		if (M2.checkbpp_sort_multi())
		{
			BPPManager M1(v1.get_width(), v1.get_length());
			vector<Bin> Bin_tempitems1;
			for (string bid : minitems1)
				Bin_tempitems1.push_back(bins.at(bid));
			for (size_t j = 0; j < n_remove; j++)
				Bin_tempitems1.push_back(bins.at(items2[j]));
			M1.add_bins(Bin_tempitems1);

			if (M1.checkbpp_sort_multi())
			{

				double cost = current_neighbour_cost;
				//v1 v2�ɱ��仯
				vector<string> temp_visit1 = v1.visit_order;
				vector<string> temp_visit2 = v2.visit_order;
				temp_visit1.erase(find(temp_visit1.begin(), temp_visit1.end(), s1.get_id()));
				double pre_distance1 = route_distance(v1.visit_order);
				double smallest_distance1 = compute_tsp(temp_visit1, s2.get_id());
				if (n_remove == items2.size())
					temp_visit2.erase(find(temp_visit2.begin(), temp_visit2.end(), s2.get_id()));

				if (find(temp_visit2.begin(), temp_visit2.end(), s1.get_id()) == temp_visit2.end())
					temp_visit2.push_back(s1.get_id());
				double pre_distance2 = route_distance(v2.visit_order);
				double smallest_distance2 = compute_tsp(temp_visit2);


				cost += (smallest_distance1 - pre_distance1) * v1.get_distance_fare();
				cost += (smallest_distance2 - pre_distance2) * v2.get_distance_fare();

				if (cost >= current_neighbour_cost)
					std::cout << "�ͷ��ɱ�: " << ETA * cost * (times_in.at(make_pair(s1.get_id(), v2.get_id())) +
						times_out.at(make_pair(s1.get_id(), v1.get_id())) +
						times_out.at(make_pair(s2.get_id(), v2.get_id())) +
						times_in.at(make_pair(s2.get_id(), v1.get_id()))) / 4 / log(current_iter) << endl;

				current_neighbour_cost_cmp = 9999999;
				current_neighbour_cost = cost;

				std::cout << "New Neighbour Best:" << cost << endl;
				v1.visit_order = temp_visit1;
				M1.return_seq(v1.loaded_items);

				v1.set_loaded_area(total_area1 + temp_area);
				v1.set_loaded_weight(total_weight1 + temp_weight);
				v2.visit_order = temp_visit2;
				M2.return_seq(v2.loaded_items);
				v2.set_loaded_area(total_area2 - temp_area);
				v2.set_loaded_weight(total_weight2 - temp_weight);
				s1.discard(v1.get_id());
				if (n_remove == items2.size())
					s2.pass_vehicles.erase(v2.get_id());
				s2.pass_vehicles.insert(v1.get_id());
				std::cout << "Move2 Real New Neighbour Best:" << cal_total_cost() << endl;

				if (cost < best_known_cost) {
					std::cout << "New Best Cost:" << cost << endl;
					best_known_cost = cost;
					best_known_sol = save_sol("result\\init_sol.json");
					std::cout << "best��¼����������" << cal_num_bins() << endl;
				}
				/*		std::cout << "move2 ��v1��" << v1.visit_order.size() << endl;
						std::cout << "move2 ��v2��" << v2.visit_order.size() << endl;*/

				return true;
			}
		}
		return false;
	}

	bool real_move3(Station& s,
		Station& s2,
		Vehicle& v1,
		Vehicle& v2) {
		vector<string> items1;    //v1��վ��s������
		vector<string> items1_left;   //v1�в�����վ��s������
		for (string item : v1.loaded_items) {
			if (s.get_id() == bins.at(item).get_station())
				items1.push_back(item);
			else
				items1_left.push_back(item);
		}
		/*	std::cout << "move3 ǰv1��" << v1.visit_order.size() << endl;
			std::cout << "move3 ǰv2��" << v2.visit_order.size() << endl;*/

		double s1_area = 0, s1_weight = 0;
		for (auto bid : items1) {
			s1_area += bins.at(bid).get_area();
			s1_weight += bins.at(bid).get_weight();
		}

		vector<string> items2;       //v2��վ��s2������
		vector<string> items2_left;  //v2�в�����վ��s2������
		for (string item : v2.loaded_items) {
			if (s2.get_id() == bins.at(item).get_station())
				items2.push_back(item);
			else
				items2_left.push_back(item);
		}

		double s2_area = 0, s2_weight = 0;
		for (auto bid : items2) {
			s2_area += bins.at(bid).get_area();
			s2_weight += bins.at(bid).get_weight();
		}

		vector<string> total_items1(items1_left);
		total_items1.insert(total_items1.end(), items2.begin(), items2.end());

		double total_area1 = v1.get_loaded_area() - s1_area + s2_area;
		double total_weight1 = v1.get_loaded_weight() - s1_weight + s2_weight;
		vector<Bin> Bin_total_items1;
		for (string bid : total_items1) {
			Bin_total_items1.push_back(bins.at(bid));
		}
		BPPManager M1(v1.get_width(), v1.get_length());
		M1.add_bins(Bin_total_items1);
		if (M1.checkbpp_sort_multi()) {
			vector<string> total_items2(items2_left);
			total_items2.insert(total_items2.end(), items1.begin(), items1.end());
			double total_area2 = v2.get_loaded_area() - s2_area + s1_area;
			double total_weight2 = v2.get_loaded_weight() - s2_weight + s1_weight;
			vector<Bin> Bin_total_items2;
			for (string bid : total_items2) {
				Bin_total_items2.push_back(bins.at(bid));
			}
			BPPManager M2(v2.get_width(), v2.get_length());
			M2.add_bins(Bin_total_items2);
			if (M2.checkbpp_sort_multi()) {
				double cost = current_neighbour_cost;
				vector<string> curr_route1 = { s2.get_id() };
				if (v1.visit_order.size() > 1) {
					double pre_distance1 = route_distance(v1.visit_order);
					curr_route1 = v1.visit_order;
					curr_route1.erase(find(curr_route1.begin(), curr_route1.end(), s.get_id()));
					curr_route1.push_back(s2.get_id());
					double smallest_dist1 = compute_tsp(curr_route1);
					cost += (smallest_dist1 - pre_distance1) * v1.get_distance_fare();
				}
				double pre_distance2 = route_distance(v2.visit_order);
				vector<string> curr_route2 = v2.visit_order;
				curr_route2.erase(find(curr_route2.begin(), curr_route2.end(), s2.get_id()));
				curr_route2.push_back(s.get_id());
				double smallest_dist2 = compute_tsp(curr_route2);
				cost += (smallest_dist2 - pre_distance2) * v2.get_distance_fare();
				if (cost >= current_neighbour_cost)
					std::cout << "�ͷ��ɱ�: " << ETA * cost * (times_in.at(make_pair(s.get_id(), v2.get_id())) +
						times_in.at(make_pair(s2.get_id(), v1.get_id())) +
						times_out.at(make_pair(s.get_id(), v1.get_id())) +
						times_out.at(make_pair(s2.get_id(), v2.get_id()))) / 4 / log(current_iter) << endl;
				current_neighbour_cost_cmp = 9999999;
				current_neighbour_cost = cost;

				std::cout << "New Neighbour Best:" << cost << endl;
				v1.visit_order = curr_route1;
				v2.visit_order = curr_route2;
				M1.return_seq(v1.loaded_items);
				M2.return_seq(v2.loaded_items);
				v1.set_loaded_area(total_area1);
				v2.set_loaded_area(total_area2);
				v1.set_loaded_weight(total_weight1);
				v2.set_loaded_weight(total_weight2);
				s.pass_vehicles.insert(v2.get_id());
				s.discard(v1.get_id());
				s2.pass_vehicles.insert(v1.get_id());
				s2.discard(s2.get_id());
				std::cout << "Move3 Real New Neighbour Best:" << cal_total_cost() << endl;

				if (cost < best_known_cost) {
					std::cout << "New Best Cost:" << cost << endl;
					best_known_cost = cost;
					best_known_sol = save_sol("result\\init_sol.json");
					std::cout << "��ǰ�������� " << cal_num_bins() << endl;
				}
				/*		std::cout << "move3 ��v1��" << v1.visit_order.size() << endl;
						std::cout << "move3 ��v2��" << v2.visit_order.size() << endl;*/

				return true;
			}
		}
		return false;
	}

	bool real_move4(Vehicle& v1) {
		
		string sid = v1.visit_order[0];


		double cost = current_neighbour_cost;
		vector<string> vids;
		int pre_size = v1.loaded_items.size();

		for (auto& vid : stations.at(sid).pass_vehicles)
		{
			if (vid == v1.get_id())
				continue;
			double destv_wid, destv_len;
			vector<string> destv_loaded_items;
			for (auto vvv : used_vehicles)
			{
				if (vvv.get_id() == vid)
				{
					destv_wid = vvv.get_width();
					destv_len = vvv.get_length();
					destv_loaded_items = vvv.loaded_items;
					break;
				}
			}
			BPPManager M(destv_wid, destv_len);
			vector<Bin> Bin_total_items;
			for (string bid : destv_loaded_items) {
				Bin_total_items.push_back(bins.at(bid));
			}
			M.add_bins(Bin_total_items);
			for (size_t idx = 0; idx < v1.loaded_items.size(); idx++) {
				M.update_backup();
				M.add_bin(bins.at(v1.loaded_items[idx]));
				if (M.checkbpp_sort_multi()) {
					VectorRemoveAt(v1.loaded_items, idx);
					idx--;
				}
				else
					M.restore();
			}
			if (destv_loaded_items.size() != M.get_bins_size()) {
				vids.push_back(vid);
			}
		}

		for (auto& v : used_vehicles) {
			if (stations.at(sid).pass_vehicles.find(v.get_id()) != stations.at(sid).pass_vehicles.end())
				continue;
			if (v.get_id() == v1.get_id())
				continue;
			if (v.get_length() > stations.at(sid).get_limit())
				continue;
			if (v.occupancy() > 0.88 && find(v.visit_order.begin(), v.visit_order.end(), sid) == v.visit_order.end())
				continue;
			else {
				BPPManager M(v.get_width(), v.get_length());
				vector<Bin> Bin_total_items;
				for (string bid : v.loaded_items) {
					Bin_total_items.push_back(bins.at(bid));
				}
				M.add_bins(Bin_total_items);

				double pre_dist = route_distance(v.visit_order);
				double smallest_dist = pre_dist;
				vector<string> record_vo = v.visit_order;
				if (find(v.visit_order.begin(), v.visit_order.end(), sid) == v.visit_order.end()) {
					smallest_dist = compute_tsp(v.visit_order, sid);
					if (smallest_dist == -1)
						continue;
				}

				for (size_t idx = 0; idx < v1.loaded_items.size(); idx++) {
					M.update_backup();
					M.add_bin(bins.at(v1.loaded_items[idx]));
					if (M.checkbpp_sort_multi()) {
						v.set_loaded_area(v.get_loaded_area() + bins.at(v1.loaded_items.at(idx)).get_area());
						v.set_loaded_weight(v.get_loaded_weight() + bins.at(v1.loaded_items.at(idx)).get_weight());
						VectorRemoveAt(v1.loaded_items, idx);
						idx--;
					}
					else
						M.restore();
				}

				/*for (size_t idx = v1.loaded_items.size() - 1; idx > -1; --idx) {
					M.update_backup();
					M.add_bin(bins.at(v1.loaded_items.at(idx)));
					if (M.checkbpp_sort_multi()) {
						VectorRemoveAt(v1.loaded_items, idx);
						v.set_loaded_area(v.get_loaded_area() + bins.at(v1.loaded_items.at(idx)).get_area());
						v.set_loaded_weight(v.get_loaded_weight() + bins.at(v1.loaded_items.at(idx)).get_weight());
					}
					else
						M.restore();
				}*/

				if (v.loaded_items.size() != M.get_bins_size()) {
					M.return_seq(v.loaded_items);
					stations.at(sid).pass_vehicles.insert(v.get_id());
					cost += (smallest_dist - pre_dist) * v.get_distance_fare();
					vids.push_back(v.get_id());
					update_tabusets(v);
				}
				else {
					v.visit_order = record_vo;
				}
			}
		}
		cout << "shuru1 v1" << v1.get_id() << endl;
		if (v1.loaded_items.size() != pre_size)
			update_tabusets(v1);

		if (v1.loaded_items.size() == 0) {
			unused_vehicles.insert({ v1.get_id(), v1 });
			cost -= v1.get_flagdown_fare();
			cout << "real flag car��" << v1.get_id() << v1.get_flagdown_fare() << endl;

			stations.at(sid).discard(v1.get_id());
		}
		else {
			v1.set_loaded_area(0);
			v1.set_loaded_weight(0);
			for (string item : v1.loaded_items) {
				v1.set_loaded_area(v1.get_loaded_area() + bins.at(item).get_area());
				v1.set_loaded_weight(v1.get_loaded_weight() + bins.at(item).get_weight());
			}
		}

		if (cost >= current_neighbour_cost) {
			double total_times = times_out.at(make_pair(sid, v1.get_id()));
			for (string vid : vids) {
				total_times += times_in.at(make_pair(sid, vid));
			}
			std::cout << "�ͷ��ɱ���" << ETA * cost * total_times / vids.size() / log(current_iter) << endl;
		}

		current_neighbour_cost_cmp = 99999999;
		current_neighbour_cost = cost;
		std::cout << "New Neighbour Best:" << cost << endl;

		if (cost < best_known_cost) {
			std::cout << "New Best Cost:" << cost << endl;
			best_known_cost = cost;
			best_known_sol = save_sol("result\\init_sol.json");
			std::cout << "��ǰ�������� " << cal_num_bins() << endl;
		}

		tabuset_in.at(make_pair(sid, v1.get_id())) = current_iter + TABU_LENGTH;
		times_out.at(make_pair(sid, v1.get_id()))++;
		for (string vid : vids) {
			times_in.at(make_pair(sid, vid))++;
			tabuset_out.at(make_pair(sid, vid)) = current_iter + TABU_LENGTH;
		}

		if (v1.loaded_items.size() == 0) 
			used_vehicles.erase(find(used_vehicles.begin(), used_vehicles.end(), v1));
		std::cout << "Move4 Real New Neighbour Best:" << cal_total_cost() << endl;

	}



	bool real_move() {

		if (current_neighbour_move[0] == "1") {
			Station& s = stations.at(current_neighbour_move[3]);
			for (auto& v : used_vehicles) {
				if (v.get_id() == current_neighbour_move[1]) {
					Vehicle &v1 = v;
					for (auto& vv : used_vehicles) {
						if (vv.get_id() == current_neighbour_move[2]) {
							Vehicle &v2 = vv;
							std::cout << v1.get_id() << endl;
							print_vector(v.visit_order);
							update_tabusets(v1, v2);
							times_in.at(make_pair(s.get_id(), v2.get_id()))++;
							times_out.at(make_pair(s.get_id(), v1.get_id()))++;
							tabuset_in.at(make_pair(current_neighbour_move[3], current_neighbour_move[1])) = TABU_LENGTH + current_iter;
							tabuset_out.at(make_pair(current_neighbour_move[3], current_neighbour_move[2])) = TABU_LENGTH + current_iter;
							return real_move1(s, v1, v2);
						}
					}
				}
			}
		}
		else if (current_neighbour_move[0] == "2") {
			Station& s1 = stations.at(current_neighbour_move[3]);
			Station& s2 = stations.at(current_neighbour_move[4]);
			for (auto& v : used_vehicles) {
				if (v.get_id() == current_neighbour_move[1]) {
					Vehicle &v1 = v;
					for (auto& vv : used_vehicles) {
						if (vv.get_id() == current_neighbour_move[2]) {
							Vehicle &v2 = vv;
							update_tabusets(v1, v2);
							times_in.at(make_pair(s1.get_id(), v2.get_id()))++;
							times_out.at(make_pair(s1.get_id(), v1.get_id()))++;
							times_in.at(make_pair(s2.get_id(), v1.get_id()))++;
							times_out.at(make_pair(s2.get_id(), v2.get_id()))++;
							tabuset_in.at(make_pair(current_neighbour_move[3], current_neighbour_move[1])) = TABU_LENGTH + current_iter;
							tabuset_in.at(make_pair(current_neighbour_move[4], current_neighbour_move[2])) = TABU_LENGTH + current_iter;
							tabuset_out.at(make_pair(current_neighbour_move[3], current_neighbour_move[2])) = TABU_LENGTH + current_iter;
							tabuset_out.at(make_pair(current_neighbour_move[4], current_neighbour_move[1])) = TABU_LENGTH + current_iter;
							return real_move2(s1, s2, v1, v2, stoi(current_neighbour_move[5]));
						}
					}
				}
			}
		}

		else if (current_neighbour_move[0] == "3") {
			Station& s1 = stations.at(current_neighbour_move[3]);
			Station& s2 = stations.at(current_neighbour_move[4]);
			for (auto& v : used_vehicles) {
				if (v.get_id() == current_neighbour_move[1]) {
					Vehicle &v1 = v;
					for (auto& vv : used_vehicles) {
						if (vv.get_id() == current_neighbour_move[2]) {
							Vehicle &v2 = vv;
							update_tabusets(v1, v2);
							times_in.at(make_pair(s1.get_id(), v2.get_id()))++;
							times_out.at(make_pair(s1.get_id(), v1.get_id()))++;
							times_in.at(make_pair(s2.get_id(), v1.get_id()))++;
							times_out.at(make_pair(s2.get_id(), v2.get_id()))++;
							tabuset_in.at(make_pair(current_neighbour_move[3], current_neighbour_move[1])) = TABU_LENGTH + current_iter;
							tabuset_in.at(make_pair(current_neighbour_move[4], current_neighbour_move[2])) = TABU_LENGTH + current_iter;
							tabuset_out.at(make_pair(current_neighbour_move[3], current_neighbour_move[2])) = TABU_LENGTH + current_iter;
							tabuset_out.at(make_pair(current_neighbour_move[4], current_neighbour_move[1])) = TABU_LENGTH + current_iter;
							return real_move3(s1, s2, v1, v2);
						}
					}
				}
			}
		}
		else {
			for (auto& v : used_vehicles) {
				if (v.get_id() == current_neighbour_move[1]) {
					return real_move4(v);
				}
			}
		}
	}
}
