#include "h_globalContext.h"
#include "hwRobot_2.h"
#include "workbench.h"
#include "basePathPlanning.h"

h_globalContext H_GlobalContext;

h_globalContext::h_globalContext() {
	botArr = (hwRobot_2*) malloc(sizeof(hwRobot_2) * 4);
	wbArr = (hwWorkbench*) malloc(sizeof(hwWorkbench) * 50);
	jamBufferSize = 6;
}

const double calDist(const double x1, const double y1, const double x2, const double y2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

const double calDistSq(const double x1, const double y1, const double x2, const double y2) {
	return (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
}

void getLatticeIndex(double x, double y, int& idx, int& idy) {
	idx = round(2 * x - 0.5);
	idy = round(2 * y - 0.5);
}