#pragma once
#include "TS.h"
#include <functional>
#include <unordered_map>

extern vector<Vehicle> used_vehicles;
extern unordered_map<string, Vehicle> unused_vehicles;
extern unordered_map<string, Bin> bins;
extern unordered_map<string, Station> stations;




bool comp_desca(const Bin &a, const Bin &b)
{
	return a.get_area() > b.get_area();
}

bool comp_asca(const Bin &a, const Bin &b)
{
	return a.get_area() < b.get_area();
}

bool comp_descls(const Bin &a, const Bin &b)
{
	double longera = a.get_width() > a.get_length() ? a.get_width() : a.get_length();
	double longerb = b.get_width() > b.get_length() ? b.get_width() : b.get_length();
	return longera > longerb;
}

bool comp_ascls(const Bin &a, const Bin &b)
{
	double longera = a.get_width() > a.get_length() ? a.get_width() : a.get_length();
	double longerb = b.get_width() > b.get_length() ? b.get_width() : b.get_length();
	return longera < longerb;
}

bool comp_descss(const Bin &a, const Bin &b)
{
	double shortera = a.get_width() < a.get_length() ? a.get_width() : a.get_length();
	double shorterb = b.get_width() < b.get_length() ? b.get_width() : b.get_length();
	return shortera > shorterb;
}

bool comp_ascss(const Bin &a, const Bin &b)
{
	double shortera = a.get_width() < a.get_length() ? a.get_width() : a.get_length();
	double shorterb = b.get_width() < b.get_length() ? b.get_width() : b.get_length();
	return shortera < shorterb;
}

bool comp_descperim(const Bin &a, const Bin &b)
{
	double perima = a.get_width() + a.get_length();
	double perimb = b.get_width() + b.get_length();
	return perima > perimb;
}

bool comp_ascperim(const Bin &a, const Bin &b)
{
	double perima = a.get_width() + a.get_length();
	double perimb = b.get_width() + b.get_length();
	return perima < perimb;
}


bool comp_descdiff(const Bin &a, const Bin &b)
{
	double diffa = abs(a.get_width() - a.get_length());
	double diffb = abs(b.get_width() - b.get_length());
	return diffa > diffb;
}

bool comp_ascdiff(const Bin &a, const Bin &b)
{
	double diffa = abs(a.get_width() - a.get_length());
	double diffb = abs(b.get_width() - b.get_length());
	return diffa < diffb;
}

bool comp_descratio(const Bin &a, const Bin &b)
{
	double ratioa = a.get_width() / a.get_length();
	double ratiob = b.get_width() / b.get_length();
	return ratioa > ratiob;
}

bool comp_ascratio(const Bin &a, const Bin &b)
{
	double ratioa = a.get_width() / a.get_length();
	double ratiob = b.get_width() / b.get_length();
	return ratioa < ratiob;
}

//按空闲面积降序
bool cmp_rest_area(const Vehicle& a, const Vehicle& b) {
	double rest_a = a.get_area() - a.get_loaded_area();
	double rest_b = b.get_area() - b.get_loaded_area();
	return rest_a > rest_b;
}

//按已占用面积升序
bool cmp_by_area(const Vehicle& a, const Vehicle& b) {
	return a.get_loaded_area() < b.get_loaded_area();
}
//按id比箱子
bool comp_desca_id(const string &aid, const string &bid)
{
	return bins.at(aid).get_area() > bins.at(bid).get_area();
}

bool comp_asca_id(const string &aid, const string &bid)
{
	return bins.at(aid).get_area() < bins.at(bid).get_area();
}


bool comp_descls_id(const string &aid, const string &bid)
{
	double longera = bins.at(aid).get_width() > bins.at(aid).get_length() ? bins.at(aid).get_width() : bins.at(aid).get_length();
	double longerb = bins.at(bid).get_width() > bins.at(bid).get_length() ? bins.at(bid).get_width() : bins.at(bid).get_length();
	return longera > longerb;
}

bool comp_ascls_id(const string &aid, const string &bid)
{
	double longera = bins.at(aid).get_width() > bins.at(aid).get_length() ? bins.at(aid).get_width() : bins.at(aid).get_length();
	double longerb = bins.at(bid).get_width() > bins.at(bid).get_length() ? bins.at(bid).get_width() : bins.at(bid).get_length();
	return longera < longerb;
}


bool comp_descss_id(const string &aid, const string &bid)
{
	double shortera = bins.at(aid).get_width() < bins.at(aid).get_length() ? bins.at(aid).get_width() : bins.at(aid).get_length();
	double shorterb = bins.at(bid).get_width() < bins.at(bid).get_length() ? bins.at(bid).get_width() : bins.at(bid).get_length();
	return shortera > shorterb;
}

bool comp_ascss_id(const string &aid, const string &bid)
{
	double shortera = bins.at(aid).get_width() < bins.at(aid).get_length() ? bins.at(aid).get_width() : bins.at(aid).get_length();
	double shorterb = bins.at(bid).get_width() < bins.at(bid).get_length() ? bins.at(bid).get_width() : bins.at(bid).get_length();
	return shortera < shorterb;
}

bool comp_descperim_id(const string &aid, const string &bid)
{
	double perima = bins.at(aid).get_width() + bins.at(aid).get_length();
	double perimb = bins.at(bid).get_width() + bins.at(bid).get_length();
	return perima > perimb;
}

bool comp_ascperim_id(const string &aid, const string &bid)
{
	double perima = bins.at(aid).get_width() + bins.at(aid).get_length();
	double perimb = bins.at(bid).get_width() + bins.at(bid).get_length();
	return perima < perimb;
}

bool comp_descdiff_id(const string &aid, const string &bid)
{
	double diffa = abs(bins.at(aid).get_width() - bins.at(aid).get_length());
	double diffb = abs(bins.at(bid).get_width() - bins.at(bid).get_length());
	return diffa > diffb;
}

bool comp_ascdiff_id(const string &aid, const string &bid)
{
	double diffa = abs(bins.at(aid).get_width() - bins.at(aid).get_length());
	double diffb = abs(bins.at(bid).get_width() - bins.at(bid).get_length());
	return diffa < diffb;
}



bool comp_descratio_id(const string &aid, const string &bid)
{
	double ratioa = bins.at(aid).get_width() / bins.at(aid).get_length();
	double ratiob = bins.at(bid).get_width() / bins.at(bid).get_length();
	return ratioa > ratiob;
}

bool comp_ascratio_id(const string &aid, const string &bid)
{
	double ratioa = bins.at(aid).get_width() / bins.at(aid).get_length();
	double ratiob = bins.at(bid).get_width() / bins.at(bid).get_length();
	return ratioa < ratiob;
}




typedef bool(*fcomp)(const Bin &a, const Bin &b);
fcomp fun_comp[12] = { comp_desca, comp_descdiff, comp_descls, comp_descperim, comp_descratio, comp_descss,
comp_asca, comp_ascdiff, comp_ascls, comp_ascperim, comp_ascratio, comp_ascss };
typedef bool(*fcomp1)(const string &aid, const string &bid);
fcomp1 fun_comp_id[12] = { comp_desca_id, comp_descdiff_id, comp_descls_id, comp_descperim_id, comp_descratio_id, comp_descss_id,
comp_asca_id, comp_ascdiff_id, comp_ascls_id, comp_ascperim_id, comp_ascratio_id, comp_ascss_id };


struct pair_hash {
	template <class T1, class T2>
	std::size_t operator () (const std::pair<T1, T2> &p) const {
		auto h1 = std::hash<T1>{}(get<0>(p));
		auto h2 = std::hash<T2>{}(get<1>(p));

		return h1 ^ h2;
	}
};

struct tuple_hash3 {
	template <class T1, class T2, class T3>
	std::size_t operator () (const std::tuple<T1, T2, T3> &p) const {
		auto h1 = std::hash<T1>{}(get<0>(p));
		auto h2 = std::hash<T2>{}(get<1>(p));
		auto h3 = std::hash<T3>{}(get<2>(p));

		return h1 ^ h2 ^ h3;
	}
};

struct tuple_hash4 {
	template <class T1, class T2, class T3, class T4>
	std::size_t operator () (const std::tuple<T1, T2, T3, T4> &p) const {
		auto h1 = std::hash<T1>{}(get<0>(p));
		auto h2 = std::hash<T2>{}(get<1>(p));
		auto h3 = std::hash<T3>{}(get<2>(p));
		auto h4 = std::hash<T4>{}(get<3>(p));

		return h1 ^ h2 ^ h3 ^ h4;
	}
};
