/**
 * @file LinkedList.h
 * The linked list class to support the tree class.
 * @note
 * This implementation is not meant to be directly used by the client code.
 *
 * @see Tree.h
 *
 * @author Haimo Zhang <zh.hammer.dev@gmail.com>
 */
#ifndef LINKEDLIST_H
#define LINKEDLIST_H

template <typename VAL_T>
class LinkedListNode
{
public:
	VAL_T value;
	LinkedListNode<VAL_T> *next;
	LinkedListNode(VAL_T const &v): value(v), next(NULL) { }
};

template <typename VAL_T>
class NodeIterator
{
public:
	VAL_T operator*() { return ptr->value; }
	void operator++() { ptr = ptr->next; }
	bool operator!=(NodeIterator<VAL_T> const &it) { return ptr != it.ptr; }
	NodeIterator(): ptr(NULL) { }
	NodeIterator(LinkedListNode<VAL_T> *n): ptr(n) { }
private:
	LinkedListNode<VAL_T> *ptr;
};

template <typename VAL_T>
class LinkedList
{
public:
	LinkedList(): head(NULL), tail(NULL), last_ptr(NULL) { }
	~LinkedList()
	{
		LinkedListNode<VAL_T> *to_del;
		while (head) {
			to_del = head;
			head = head->next;
			delete to_del;
		}
	}
	void append(VAL_T const &v)
	{
		LinkedListNode<VAL_T> *new_node = new LinkedListNode<VAL_T>(v);
		if (head) {
			*tail = new_node;
		}
		else {
			head = new_node;
		}
		tail = &new_node->next;
		last_ptr = new_node;
	}
	bool isEmpty() { return head == NULL; }
	typedef NodeIterator<VAL_T> iterator;
	NodeIterator<VAL_T> begin() { return NodeIterator<VAL_T>(head); }
	NodeIterator<VAL_T> end() { return NodeIterator<VAL_T>(); }
	NodeIterator<VAL_T> last() { return NodeIterator<VAL_T>(last_ptr); }
private:
	LinkedListNode<VAL_T> *head;
	LinkedListNode<VAL_T> **tail;
	LinkedListNode<VAL_T> *last_ptr;
};

#endif
