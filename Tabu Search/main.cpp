#include "TS.h"
#include "bpp.h"
#include "entity\Vehicle.h"
#include "entity\Bin.h"
#include "entity\Station.h"
#include "util.h"
#include "rapidjson\document.h"
#include "rapidjson\writer.h"
#include "rapidjson\stringbuffer.h"
#include <iostream>
#include <map>
#include <cmath>

using namespace rapidjson;
using namespace ts;
using namespace my_util;

int main() {

	// ��������
	bins = get_bins_data();
	stations = get_stations_data();
	get_distance_matrix();
	get_load_time_matrix();


	//VNS��ʼ
	best_known_sol = ts::initialize();
	best_known_cost = cal_total_cost();
	current_neighbour_cost = best_known_cost;
	current_neighbour_cost_cmp = 9999999;

	cout << "��ʼ�ɱ���" << best_known_cost << endl;
	cout << "��ʼ��������" << cal_num_bins() << endl;


	for (current_iter; current_iter < 100000000; ++current_iter) {
		cout << "\n��ǰ������" << current_iter << "��:" << endl;
		cout << "��ǰ���ųɱ���" << best_known_cost << endl;
		LS1();
		LS3();
		LS2();
		if (current_neighbour_cost <= current_neighbour_cost_cmp)
			LS4();
		real_move();
	}

	getchar();
	return 0;
}