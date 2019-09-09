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

	// 导入数据
	bins = get_bins_data();
	stations = get_stations_data();
	get_distance_matrix();
	get_load_time_matrix();


	//TS开始
	best_known_sol = ts::initialize();
	best_known_cost = cal_total_cost();
	current_neighbour_cost = best_known_cost;
	current_neighbour_cost_cmp = 9999999;

	cout << "初始成本：" << best_known_cost << endl;
	std::cout << "Current # of bins: " << cal_num_bins() << endl;

	for (current_iter; current_iter < 100000000; ++current_iter) {
		cout << "\n当前迭代第" << current_iter << "次:" << endl;
		cout << "当前最优成本：" << best_known_cost << endl;
		cout << "当前用车：" << used_vehicles.size() << endl;
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