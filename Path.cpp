#include "Path.h"
//#include "hwcout.h"

//std::ostream& operator<<(std::ostream& os, Point& const p) {
//	os << "(x : " << p.x << ", y : " << p.y << ")";
//	return os;
//}
//HWCout& operator<<(HWCout& os, Point& const p) {
//	os << "(x : " << p.x << ", y : " << p.y << ")";
//	return os;
//}
//
//std::ostream& operator<<(std::ostream& os, Path& p) {
//	int len = p.size();
//	if (len == 0) {
//		os << " this path is empty " << endl;
//		return os;
//	}
//
//	os << 0 << ":" << p[0];
//
//	for (int i = 1; i < len; ++i) {
//		os << ", ";
//		os << i << ":" << p[i];
//	}
//	return os;
//}
//HWCout& operator<<(HWCout & os, Path& p) {
//	int len = p.size();
//	if (len == 0) {
//		os << " this path is empty " << hwendl;
//		return os;
//	}
//
//	os << 0 << ":" << p[0];
//
//	for (int i = 1; i < len; ++i) {
//		os << ", ";
//		os << i << ":" << p[i];
//	}
//	return os;
//}