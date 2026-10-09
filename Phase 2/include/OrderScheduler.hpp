#ifndef ORDER_SCHEDULER_HPP
#define ORDER_SCHEDULER_HPP

#include <queue>
#include <vector> 

struct DeliveryOrder {
    int order_id;
    int target_node_id;
    double payload_kg;
    double deadline_sec;

    bool operator>(const DeliveryOrder& o) const {
        return deadline_sec > o.deadline_sec;
    }
};

class OrderScheduler {
private:
    std::priority_queue<DeliveryOrder, std::vector<DeliveryOrder>, std::greater<DeliveryOrder>> min_heap;

public:
    void enqueueOrder(const DeliveryOrder& order) {
        min_heap.push(order);
    }

    bool hasPendingOrders() const {
        return !min_heap.empty();
    }

    DeliveryOrder popNextOrder() {
        DeliveryOrder top = min_heap.top();
        min_heap.pop();
        return top;
    }

    size_t pendingCount() const {
        return min_heap.size();
    }
};

#endif
