#include "controlConstraints.h"

float ConstraintList::Constraint::range() const {
	return this->max - this->min;
}

int ConstraintList::size() const {
	return _cv.size();
}


void ConstraintList::add(float min, float max) {
	Constraint c;
	c.min = min;
	c.max = max;
	_cv.push_back(c);
}


bool ConstraintList::getMin(int index, float * min) const {
	if (index < 0 || index >= (int)_cv.size()) {
		*min = 0;
		return false;
	}

	*min = _cv[index].min;
	return true;
}


bool ConstraintList::getMax(int index, float * max) const {
	if (index < 0 || index >= (int)_cv.size()) {
		*max = 0;
		return false;
	}

	*max = _cv[index].max;
	return true;
}


bool ConstraintList::getRange(int index, float * range) const {
	if (index < 0 || index >= (int)_cv.size())
		return false;

	*range = _cv[index].range();
	return true;
}
