using System;
using System.Collections.Generic;

public class PriorityQueue<TElement, TPriority> where TPriority : IComparable<TPriority>
{
    private readonly List<(TElement element, TPriority priority, int insertionIndex)> _heap = new();
    private int _insertionCounter = 0;

    public int Count => _heap.Count;
    public bool IsEmpty => _heap.Count == 0;

    public void Enqueue(TElement element, TPriority priority)
    {
        _heap.Add((element, priority, _insertionCounter++));
        HeapifyUp(_heap.Count - 1);
    }

    public TElement Dequeue()
    {
        if (_heap.Count == 0)
            throw new InvalidOperationException("PriorityQueue is empty.");

        TElement result = _heap[0].element;
        _heap[0] = _heap[^1];
        _heap.RemoveAt(_heap.Count - 1);
        if (_heap.Count > 0)
            HeapifyDown(0);

        return result;
    }

    public (TElement element, TPriority priority) DequeueWithPriority()
    {
        if (_heap.Count == 0)
            throw new InvalidOperationException("PriorityQueue is empty.");

        var result = (_heap[0].element, _heap[0].priority);

        _heap[0] = _heap[^1];
        _heap.RemoveAt(_heap.Count - 1);
        if (_heap.Count > 0)
            HeapifyDown(0);

        return result;
    }


    public TElement Peek()
    {
        if (_heap.Count == 0)
            throw new InvalidOperationException("PriorityQueue is empty.");

        return _heap[0].element;
    }

    private void HeapifyUp(int index)
    {
        while (index > 0)
        {
            int parent = (index - 1) / 2;
            if (Compare(index, parent) >= 0)
                break;
            Swap(index, parent);
            index = parent;
        }
    }

    private void HeapifyDown(int index)
    {
        int last = _heap.Count - 1;
        while (true)
        {
            int left = 2 * index + 1;
            int right = 2 * index + 2;
            int smallest = index;

            if (left <= last && Compare(left, smallest) < 0)
                smallest = left;
            if (right <= last && Compare(right, smallest) < 0)
                smallest = right;

            if (smallest == index)
                break;

            Swap(index, smallest);
            index = smallest;
        }
    }

    private int Compare(int i, int j)
    {
        int result = _heap[i].priority.CompareTo(_heap[j].priority);
        if (result == 0)
            result = _heap[i].insertionIndex.CompareTo(_heap[j].insertionIndex); // Tie-breaker
        return result;
    }

    private void Swap(int i, int j)
    {
        (_heap[i], _heap[j]) = (_heap[j], _heap[i]);
    }
}