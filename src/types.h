#ifndef TYPES_H
#define TYPES_H

template <typename VAL_T, typename TS_T=unsigned long>
class Reading
{
public:
	VAL_T value;
	TS_T timestamp;
	Reading() : value(0), timestamp(0) {}
	Reading(VAL_T v, TS_T ts) : value(v), timestamp(ts) { }
};

template <typename T>
class Differential
{
public:
	T position;
	T speed;
	T acceleration;
	Differential(T pos, T spd, T acc) : position(pos), speed(spd), acceleration(acc) { }
	Differential(T val) : Differential(val, val, val) { }
};

#endif
