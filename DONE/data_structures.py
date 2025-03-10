class Heap:
    def __init__(self):
        self.heap = []

    def push(self, item):
        self.heap.append(item)
        self.heapify_up(len(self.heap) - 1)

    def pop(self):
        self.heap[0], self.heap[-1] = self.heap[-1], self.heap[0]
        item = self.heap.pop()
        self.heapify_down(0)
        return item

    def heapify_up(self, i):
        while i > 0:
            parent = (i - 1) // 2
            if self.heap[parent].cost > self.heap[i].cost:
                self.heap[parent], self.heap[i] = self.heap[i], self.heap[parent]
                i = parent
            else:
                break

    def heapify_down(self, i):
        while 2 * i + 1 < len(self.heap):
            left = 2 * i + 1
            right = 2 * i + 2
            min_child = left
            if right < len(self.heap) and self.heap[right].cost < self.heap[left].cost:
                min_child = right
            if self.heap[i].cost > self.heap[min_child].cost:
                self.heap[i], self.heap[min_child] = self.heap[min_child], self.heap[i]
                i = min_child
            else:
                break

    def peek(self):
        return self.heap[0]

    def isEmpty(self):
        return len(self.heap) == 0

    def size(self):
        return len(self.heap)

    def empty(self):
        return len(self.heap) == 0

class Stack:
    def __init__(self):
        self.stack = []

    def push(self, item):
        self.stack.append(item)

    def pop(self):
        if not self.isEmpty():
            return self.stack.pop()
        else:
            raise IndexError("pop from empty stack")

    def peek(self):
        if not self.isEmpty():
            return self.stack[-1]
        else:
            raise IndexError("peek from empty stack")

    def isEmpty(self):
        return len(self.stack) == 0

    def size(self):
        return len(self.stack)

class Queue:
    def __init__(self):
        self.queue = []

    def enqueue(self, item):
        self.queue.append(item)

    def dequeue(self):
        return self.queue.pop(0)

    def peek(self):
        return self.queue[0]

    def isEmpty(self):
        return len(self.queue) == 0

    def size(self):
        return len(self.queue)

    def empty(self):
        return len(self.queue) == 0