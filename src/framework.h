/**
 * @file framework.h
 * %Filter framework.
 *
 * @author Haimo Zhang <zh.hammer.dev@gmail.com>
 */
#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include "LinkedList.h"
#include "Tree.h"

/**
 * %Filter interface without type checking at compile time.
 * @note It is the client code's responsibility to ensure type compatibility
 * between filters.
 */
class Filter
{
	friend class FilterChain;
	friend class FilterTree;
public:
	/**
	 * Push a new data through the filter.
	 *
	 * @note
	 * A filter is not required to always output a data in response
	 * to a new input data.
	 * For example, a delay filter might wait for several input data
	 * before outputing.
	 * This behavior is supported through the boolean return value.
	 *
	 * @param[in] input
	 * A read-only pointer to the input data.
	 *
	 * @param[out] output
	 * A pointer to the memory (managed by the client code)
	 * where the output data is to be written.
	 *
	 * @returns
	 * True if there is output data; false otherwise.
	 * @note
	 * The output memory is not guaranteed to remain the same even if
	 * the return value is false.
	 */
	bool push(void const * const input, void * const output)
	{
		if (input != NULL && update(input)) {
			copy_to_client(output);
			return true;
		}
		return false;
	}
protected:
	/**
	 * Access the read-only internal output memory.
	 *
	 * @returns
	 * A read-only pointer to the memory where the output value is stored internally
	 * by the filter.
	 */
	virtual void const * const get_output_val_ptr() = 0;
	/**
	 * Internally update the filter output based on the given input.
	 * This method behaves similarly to the public push method,
	 * but without copying the output to the client memory.
	 * This method is for internal workings of the filter framework.
	 */
	virtual bool update(void const * const input) = 0;
	/**
	 * Copy the output to client memory.
	 */
	virtual void copy_to_client(void * const output) = 0;
};

/**
 * The typed filter base class.
 *
 * @tparam IN_T type of input data
 * @tparam OUT_T type of output data
 *
 * @see Filter
 */
template <typename IN_T, typename OUT_T>
class BaseFilter : public Filter
{
public:
	/**
	 * Push a new data through the filter.
	 *
	 * @param[in] input A read-only reference to the input data.
	 * @param[out] output The reference to the output data to be written to.
	 *
	 * @returns True if there is output data; false otherwise.
	 * @note The output variable is not guaranteed to remain the same even if
	 * the return value is false.
	 */
	bool push(IN_T const &input, OUT_T &output)
	{
		return Filter::push(&input, &output);
	}
protected:
	virtual void const * const get_output_val_ptr() final { return &out_val; }
	virtual void copy_to_client(void * const output) final
	{
		if (output != NULL) {
			*(OUT_T * const) output = out_val;
		}
	}
	/**
	 * Internally managed storage of the output value.
	 */
	OUT_T out_val;
};

/**
 * A chain of filters.
 */
class FilterChain : public LinkedList<Filter *>, public Filter
{
protected:
	virtual void const * const get_output_val_ptr() final
	{
		return (*last())->get_output_val_ptr();
	}
	virtual bool update(void const * const input) final
	{
		for (it = begin(); it != last(); ++it) {
			if (!(*it)->update(it != begin() ? (*prev)->get_output_val_ptr() : input)) {
				return false;
			}
			prev = it;
		}
		return (*it)->update((*prev)->get_output_val_ptr());
	}
	virtual void copy_to_client(void * const output) final
	{
		if (output != NULL && !isEmpty()) {
			(*last())->copy_to_client(output);
		}
	}
private:
	LinkedList<Filter *>::iterator it;
	LinkedList<Filter *>::iterator prev;
};

/**
 * A tree of interconnected filters.
 * While a filter has only one input, it can output to multiple other filters.
 * Therefore, we can construct a tree of filters to simplify the interaction
 * with complex filter graph in the client code.
 *
 * @todo Implement the filter tree when this use case is needed.
 */
class FilterTree : public Tree<Filter *>
{
};

#endif
