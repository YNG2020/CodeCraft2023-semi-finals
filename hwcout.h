//#pragma once
//#include <iostream>
//#include <fstream>
////#include "Path.h"
////#ifndef _WIN32
////	#define HW_ONLINE 1
////#endif // WIN32
//
//
//#ifndef DEBUG
//	#define HW_ONLINE 1
//#endif // WIN32
//
//
//using namespace std;
//class HWCout {
//public:
//	std::ofstream of;
//	streambuf* coutBuf = nullptr; 
//	bool isFirst = true;
//
//	HWCout():of(stderr){
//	}
//	void setCoutBuf() {
//		#ifndef HW_ONLINE
//			coutBuf = cout.rdbuf();
//		#endif
//	}
//	void operator<<(HWCout& i) {
//		#ifndef HW_ONLINE
//			this->isFirst = true;
//			cout << endl;
//			cout.rdbuf(coutBuf);
//		#endif
//		
//		return;
//	}
//	/*template<typename T>
//	HWCout& operator<<(const T t){
//		#ifndef HW_ONLINE
//			if (this->isFirst) {
//				this->isFirst = false;
//				cout.rdbuf(of.rdbuf());
//			}
//			cout << t;
//		#endif
//		return *this;
//	}*/
//	template<typename T>
//	HWCout& operator<<(T t){
//		#ifndef HW_ONLINE
//			if (this->isFirst) {
//				this->isFirst = false;
//				cout.rdbuf(of.rdbuf());
//			}
//			cout << t;
//		#endif
//		return *this;
//	}
//};
//
//extern HWCout hwcout;
//extern HWCout hwendl;