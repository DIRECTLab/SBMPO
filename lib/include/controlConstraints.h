#pragma once

#include <initializer_list>

#include "definitions.h"

/// @brief define and retrieve the max and min value corresponding to a
/// particular control.
class ConstraintList {
public:

	/// @brief The max and min value corresponding to a control
	struct Constraint {
		float min;
		float max;

		/// @brief The range between max and min
		///
		/// Useful when iterating through a `ConstraintList`.
		/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
		/// for (auto c : /* ConstraintList */ cl) {
		///     std::cout << c.range() << std::endl;
		/// }
		/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		float range() const;
	};

	/// @brief determine the size of private control list aka number of controls
	int size() const;

	/// @brief add the control with min and max to the constraint list
	///
	/// \param min - Min value.
	/// \param max - Max value.
	void add(float min, float max);

	/// @brief access the min value of a control stored at index
	///
	/// Gets the min constraint value for joint with the requested index.
	///
	/// \param index - Index of the joint whose min is to be retrieved.
	/// \param min - Pointer to a float where the min constraint value will be
	/// stored.
	/// \return Returns false if the requested index is out of bounds; true,
	/// otherwise.
	///
	/// \sa getMax(), getRange()
	bool getMin(int index, float *min) const;

	/// @brief access the max value of a control stored at index
	///
	/// Gets the max constraint value for joint with the requested index.
	///
	/// \param index - Index of the joint whose max is to be retrieved.
	/// \param max - Pointer to a float where the max constraint value will be
	/// stored.
	/// \return Returns false if the requested index is out of bounds; true,
	/// otherwise.
	///
	/// \sa getMin(), getRange()
	bool getMax(int index, float *max) const;

	/// @brief access the range of a control stored at index
	///
	/// Gets the range covered by the constraint values for joint with the
	/// requested index.
	///
	/// \param index - Index of the joint whose range is to be retrieved.
	/// \param range - Pointer to a float where the range constraint value will
	/// be stored.
	/// \return Returns false if the requested index is out of bounds; true,
	/// otherwise.
	/// \sa getMin(), getMax()
	bool getRange(int index, float *range) const;

	/// @name Iterator Support
	///
	/// For loop with iterator semantics
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
	/// for (auto c = cl.begin(); c != cl.end(); c++) {
	///     // we now have access to each constraint as c
	///     std::cout << c.min << " " << c.max << " " << c.range() << std::endl;
	/// }
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	/// or with a range-based for loop
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
	/// for (auto c : cl) {
	///     // we now have access to each constraint as c
	///     std::cout << c.min << " " << c.max << " " << c.range() << std::endl;
	/// }
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	/// @{

	typedef vector<Constraint>::iterator iterator;
	typedef vector<Constraint>::const_iterator const_iterator;
	iterator begin() { return _cv.begin(); }
	const_iterator begin() const { return _cv.begin(); }
	iterator end() { return _cv.end(); }
	const_iterator end() const { return _cv.end(); }
	Constraint operator [](int i) const { return _cv[i]; }

	/// @}

private:
	vector<Constraint> _cv;
};
