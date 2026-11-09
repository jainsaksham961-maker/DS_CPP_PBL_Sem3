#include "BenchmarkScheduler.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>

BenchmarkScheduler::BenchmarkScheduler() {}

BenchmarkMetrics BenchmarkScheduler::runFIFOBenchmark(std::vector<DeliveryOrder> orders, 
                                                      const AirwayGraph& graph, 
                                                      const RoutingEngine& router, 
                                                      const WindField& wind) {
    BenchmarkMetrics m;
    m.algorithm_name = "FIFO (First-In-First-Out)";
    
    std::sort(orders.begin(), orders.end(), [](const DeliveryOrder& a, const DeliveryOrder& b) {
        return a.release_time_sec < b.release_time_sec;
    });

    double current_time = 0.0;
    double total_turnaround = 0.0;

    for (auto& o : orders) {
        current_time = std::max(current_time, o.release_time_sec);
        PathResult p = router.computeAStarRoute(graph, o.source_node, o.destination_node, 18.0, o.payload_kg, wind);
        double duration = p.success ? p.total_time : 300.0;
        current_time += duration;

        double completion_time = current_time;
        total_turnaround += (completion_time - o.release_time_sec);
        if (completion_time > o.sla_deadline_sec) {
            m.sla_violations++;
        }
    }

    m.total_makespan_sec = current_time;
    m.avg_turnaround_time_sec = orders.empty() ? 0.0 : total_turnaround / orders.size();
    return m;
}

BenchmarkMetrics BenchmarkScheduler::runEDFBenchmark(std::vector<DeliveryOrder> orders, 
                                                     const AirwayGraph& graph, 
                                                     const RoutingEngine& router, 
                                                     const WindField& wind) {
    BenchmarkMetrics m;
    m.algorithm_name = "EDF (Earliest Deadline First Min-Heap)";

    auto comp = [](const DeliveryOrder& a, const DeliveryOrder& b) {
        return a.sla_deadline_sec > b.sla_deadline_sec;
    };
    std::priority_queue<DeliveryOrder, std::vector<DeliveryOrder>, decltype(comp)> edf_queue(comp);

    for (const auto& o : orders) edf_queue.push(o);

    double current_time = 0.0;
    double total_turnaround = 0.0;
    size_t count = orders.size();

    while (!edf_queue.empty()) {
        DeliveryOrder o = edf_queue.top();
        edf_queue.pop();

        current_time = std::max(current_time, o.release_time_sec);
        PathResult p = router.computeAStarRoute(graph, o.source_node, o.destination_node, 18.0, o.payload_kg, wind);
        double duration = p.success ? p.total_time : 300.0;
        current_time += duration;

        double completion_time = current_time;
        total_turnaround += (completion_time - o.release_time_sec);
        if (completion_time > o.sla_deadline_sec) {
            m.sla_violations++;
        }
    }

    m.total_makespan_sec = current_time;
    m.avg_turnaround_time_sec = count == 0 ? 0.0 : total_turnaround / count;
    return m;
}

void BenchmarkScheduler::printComparison(const BenchmarkMetrics& fifo, const BenchmarkMetrics& edf) const {
    std::cout << "\n----------------- MAKESPAN & SLA BENCHMARK REPORT (SLIDE 14) -----------------\n";
    std::cout << std::left << std::setw(32) << "Evaluation Metric"
              << std::setw(22) << "FIFO Baseline"
              << std::setw(22) << "EDF Min-Heap" << "\n";
    std::cout << "------------------------------------------------------------------------------\n";
    
    std::cout << std::left << std::setw(32) << "Total Makespan (s)"
              << std::fixed << std::setprecision(1)
              << std::setw(22) << fifo.total_makespan_sec
              << std::setw(22) << edf.total_makespan_sec << "\n";

    std::cout << std::left << std::setw(32) << "Avg Turnaround Time (s)"
              << std::setw(22) << fifo.avg_turnaround_time_sec
              << std::setw(22) << edf.avg_turnaround_time_sec << "\n";

    std::cout << std::left << std::setw(32) << "SLA Violations (Missed)"
              << std::setw(22) << fifo.sla_violations
              << std::setw(22) << edf.sla_violations << "\n";

    double improvement = fifo.avg_turnaround_time_sec > 0.0 
        ? ((fifo.avg_turnaround_time_sec - edf.avg_turnaround_time_sec) / fifo.avg_turnaround_time_sec) * 100.0 
        : 0.0;

    std::cout << "------------------------------------------------------------------------------\n";
    std::cout << "\033[1;32m[RESULT] EDF Min-Heap reduced SLA breaches by " 
              << (fifo.sla_violations - edf.sla_violations) 
              << " orders with a " << std::setprecision(1) << improvement 
              << "% turnaround improvement!\033[0m\n";
    std::cout << "------------------------------------------------------------------------------\n\n";
}
