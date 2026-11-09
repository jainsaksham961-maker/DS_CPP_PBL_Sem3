#pragma once

#include "Types.hpp"
#include "AirwayGraph.hpp"
#include "RoutingEngine.hpp"
#include <vector>
#include <queue>

class BenchmarkScheduler {
public:
    BenchmarkScheduler();

    BenchmarkMetrics runFIFOBenchmark(std::vector<DeliveryOrder> orders, 
                                     const AirwayGraph& graph, 
                                     const RoutingEngine& router, 
                                     const WindField& wind);

    BenchmarkMetrics runEDFBenchmark(std::vector<DeliveryOrder> orders, 
                                    const AirwayGraph& graph, 
                                    const RoutingEngine& router, 
                                    const WindField& wind);

    void printComparison(const BenchmarkMetrics& fifo, const BenchmarkMetrics& edf) const;
};
