#ifndef SOFTFILTERS_H
#define SOFTFILTERS_H

#include "framework.h"
#include "types.h"
#include "OneEuro.h"

/**
 * A differential filter calculates the speed and acceleration from its raw
 * scalar input.
 *
 * Time chart of a data structure to support second and third order
 * (speed & acceleration):
 *
 * ```text
 *
 *       +-- previous
 *       | +-- before
 *       | | +-- (current)
 *       | | | +-- after
 *       | | | | +-- next
 *       | | | | |
 * pos   * | * | *
 *        \|/|\|/      spd = d_pos / d_t
 * spd     *-+-*       interpolate two speeds to get the current speed
 *          \|/        acc = d_spd / d_t
 * acc       *
 *
 * ```
 *
 */
template <typename VAL_T, typename TS_T=unsigned long, typename INTERNAL_T=double>
class DifferentialFilter : public BaseFilter<Reading<VAL_T, TS_T>, Reading<Differential<VAL_T>, TS_T> >
{

public:
	typedef Reading<VAL_T, TS_T> IN_T;
	typedef Reading<Differential<VAL_T>, TS_T> OUT_T;
	DifferentialFilter() : seen_first(false), seen_second(false) { }

protected:

/**
 * @def ITN(x)
 * Cast x to the internal processing data type.
 */
#define ITN(x) ((INTERNAL_T)(x))
/**
 * @def INTERPOLATE(p0, v0, p1, p2, v2)
 * Interpolates the value of p1
 * given positions and values of two other points p0 and p2.
 *
 * @note
 * The parameters are evaluated more than once.
 */
#define INTERPOLATE(p0, v0, p1, p2, v2) (ITN(v2)*(ITN(p1)-ITN(p0))+ITN(v0)*(ITN(p2)-ITN(p1)))/(ITN(p2)-ITN(p0))
	virtual bool update(void const * const input) override
	{
		in_ptr = (IN_T const * const) input;
		if (!seen_first) {
			// On the first observation, cache the value and timestamp
			// in the internal storage for the output value.
			this->out_val.value.position = in_ptr->value;
			this->out_val.timestamp = in_ptr->timestamp;
			seen_first = true;
			return false;
		}
		else if (!seen_second) {
			// Ignore the observation if the same timestamp,
			// to avoid divide-by-zero.
			if (this->out_val.timestamp != in_ptr->timestamp) {
				// On the second observation, calculate the speed.
				next_pos = in_ptr->value;
				next_ts = in_ptr->timestamp;
				aft_ts = ITN(next_ts - this->out_val.timestamp);
				aft_spd = ITN(next_pos - this->out_val.value.position) / aft_ts;
				aft_ts /= ITN(2);
				aft_ts += ITN(this->out_val.timestamp);
				seen_second = true;
			}
			return false;
		}
		else {
			// Ignore the observation if the same timestamp,
			// to avoid divide-by-zero.
			if (next_ts == in_ptr->timestamp) {
				return false;
			}
			// update internal data
			bef_spd = aft_spd;
			bef_ts = aft_ts;
			this->out_val.value.position = next_pos;
			this->out_val.timestamp = next_ts;
			next_pos = in_ptr->value;
			next_ts = in_ptr->timestamp;
			// calculate the new speed
			aft_ts = ITN(next_ts - this->out_val.timestamp);
			aft_spd = ITN(next_pos - this->out_val.value.position) / aft_ts;
			aft_ts /= ITN(2);
			aft_ts += ITN(this->out_val.timestamp);
			// interpolate the speed
			this->out_val.value.speed = INTERPOLATE(bef_ts, bef_spd, this->out_val.timestamp, aft_ts, aft_spd);
			// calculate the acceleration
			this->out_val.value.acceleration = (aft_spd - bef_spd) / (aft_ts - bef_ts);
			return true;
		}
	}
#undef INTERPOLATE
#undef ITN

private:

	INTERNAL_T bef_spd, bef_ts;
	INTERNAL_T aft_spd, aft_ts;
	VAL_T next_pos;
	TS_T next_ts;
	bool seen_first;
	bool seen_second;
	IN_T const * in_ptr;
};

template <typename VAL_T, typename TS_T=unsigned long, TS_T (*time_fn)()=micros>
class TimestampFilter : public BaseFilter<VAL_T, Reading<VAL_T, TS_T> >
{
public:
	virtual bool update(void const * const input) override
	{
		this->out_val.value = *((VAL_T const * const) input);
		this->out_val.timestamp = time_fn();
		return true;
	}
};

template <typename IN_T, typename OUT_T>
class CachedFilter : public BaseFilter<IN_T, OUT_T>
{
public:
	CachedFilter(size_t cap) : capacity(cap), size(0), pos(0)
	{
		this->buffer = new IN_T[this->capacity];
	}
	~CachedFilter() { delete[] this->buffer; }
protected:
	virtual bool update(void const * const input) override
	{
		if (this->size < this->capacity) {
			++size;
			this->buffer[pos++] = *(IN_T const * const) input;
			pos %= capacity;
			return refresh((IN_T const * const) input, NULL, this->out_val);
		}
		else {
			cached_val = this->buffer[pos];
			this->buffer[pos++] = *(IN_T const * const) input;
			pos %= capacity;
			return refresh((IN_T const * const) input, &cached_val, this->out_val);
		}
	}
	virtual bool refresh(IN_T const * const new_val, IN_T const * const old_val, OUT_T &output) = 0;
	size_t capacity;
	size_t size;
	int pos;
	IN_T *buffer;
private:
	IN_T cached_val;
};

/**
 * INTERNAL_T is the type used for internal processing, which defaults to using
 * `double' to prevent any loss of precision between `float' input and `double'
 * output.  Any input if first casted to the internal type for processing,
 * whose result is cast to the output type.
 */
template <typename IN_T, typename OUT_T, typename INTERNAL_T=double>
class MovingAverageFilter : public CachedFilter<IN_T, OUT_T>
{
public:
	MovingAverageFilter(size_t w_sz) : CachedFilter<IN_T, OUT_T>(w_sz), sum(0) {  }
protected:
	INTERNAL_T sum;
	virtual bool refresh(IN_T const * const new_val, IN_T const * const old_val, OUT_T &output) override
	{
		// Udpate the sum.
		sum += (INTERNAL_T) *new_val - (old_val == NULL ? 0 : (INTERNAL_T) *old_val);
		output = sum / (INTERNAL_T) CachedFilter<IN_T, OUT_T>::size;
		return true;
	}
};

template <typename IN_T, typename OUT_T, typename INTERNAL_T=double>
class MovingVarianceFilter : public MovingAverageFilter<IN_T, OUT_T, INTERNAL_T>
{
public:
	MovingVarianceFilter(size_t w_sz) : MovingAverageFilter<IN_T, OUT_T, INTERNAL_T>(w_sz), squared_sum(0) { }
private:
	INTERNAL_T new_val_2;
	INTERNAL_T old_val_2;
protected:
	INTERNAL_T squared_sum;
	virtual bool refresh(IN_T const * const new_val, IN_T const * const old_val, OUT_T &output) override
	{
		// now `output' holds the mean value
		MovingAverageFilter<IN_T, OUT_T>::refresh(new_val, old_val, output);
		new_val_2 = *new_val;
		new_val_2 *= new_val_2;
		old_val_2 = old_val == NULL ? 0 : *old_val;
		old_val_2 *= old_val_2;
		squared_sum += new_val_2 - old_val_2;
		output = squared_sum / (INTERNAL_T) CachedFilter<IN_T, OUT_T>::size - output * output;
		return true;
	}
};

template <typename IN_T, typename OUT_T, typename INTERNAL_T=double>
class WeightedUpdateFilter : public BaseFilter<IN_T, OUT_T>
{
public:
	WeightedUpdateFilter(double w) : sensitivity(w), innertia(1 - w) { }
	virtual bool update(void const * const input) override
	{
		this->out_val = innertia * this->out_val + sensitivity * (INTERNAL_T) *(IN_T const * const)input;
		return true;
	}
private:
	INTERNAL_T sensitivity;  // sensitivity of the new observation, as a factor between 0 and 1
	INTERNAL_T innertia;  // 1 - sensitivity
};

/**
 * The 1-euro filter is based on the paper of the same name by Gery Casiez
 */
template <typename VAL_T, typename TS_T>
class OneEuroFilter : public BaseFilter<Reading<VAL_T, TS_T>, Reading<VAL_T, TS_T>>
{
public:
	OneEuroFilter(double _freq, VAL_T _mincutoff, VAL_T _beta, VAL_T _dcutoff)
		: filter(one_euro_filter<VAL_T, TS_T>(_freq, _mincutoff, _beta, _dcutoff))
	{}
	VAL_T mincutoff()
	{
		return filter.mincutoff;
	}
	VAL_T mincutoff(VAL_T v)
	{
		filter.mincutoff = v;
		return v;
	}
	VAL_T beta()
	{
		return filter.beta;
	}
	VAL_T beta(VAL_T v)
	{
		filter.beta = v;
		return v;
	}
	VAL_T dcutoff()
	{
		return filter.dcutoff;
	}
	VAL_T dcutoff(VAL_T v)
	{
		filter.dcutoff = v;
		return v;
	}
protected:
	virtual bool update(void const * const input) override
	{
		this->out_val.value = filter(
				((Reading<VAL_T, TS_T> const * const)input)->value,
				((Reading<VAL_T, TS_T> const * const)input)->timestamp);
		this->out_val.timestamp = ((Reading<VAL_T, TS_T> const * const)input)->timestamp;
		return true;
	}
private:
	one_euro_filter<VAL_T, TS_T> filter;
};

#endif
