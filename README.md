Autonomous Drone Fleet Routing Management System

Team ID: DSCPP-III-2026-T154
Course: Data Structure and Programming in C++
Department: Computer Science & Engineering
Academic Session: 2026–27
Mentor: Dr. Amit Kumar

Team Members

Saksham Jain – 2510380116
Jiya – 2510370376
Tamanna Bhinder – 2510380219

---

1. Project Overview

The Autonomous Drone Fleet Routing Management System will be a C++17-based software system designed to manage multiple autonomous drones for delivery operations.

The system will receive delivery requests and intelligently assign them to suitable drones while considering delivery deadlines, battery capacity, payload weight, route distance, and wind conditions.

The system will combine Data Structures, Algorithms, Graph Theory, and Object-Oriented Programming in C++ to provide efficient fleet scheduling and route optimization.

---

2. Problem Statement

The project will address the problem of efficiently managing a fleet of autonomous drones for last-mile delivery.

For every delivery request, the system will need to determine:

- Which drone should handle the order?
- Which order should be handled first?
- Which route should the drone follow?
- Does the drone have enough battery?
- Can the drone carry the required payload?
- Can the delivery be completed before the deadline?
- How will wind conditions affect the route and energy consumption?

The system will therefore provide deadline-aware, battery-aware, payload-aware and wind-aware fleet routing.

---

3. Main Objectives

The project will aim to:

1. Automatically prioritize delivery orders.
2. Assign suitable drones to delivery requests.
3. Represent the drone airspace using a 3D weighted graph.
4. Find optimized routes using Dijkstra's and A* algorithms.
5. Consider wind conditions while calculating route costs.
6. Monitor the battery and payload status of drones.
7. Check whether a mission is feasible before assigning it.
8. Manage multiple drones simultaneously.
9. Generate optimized waypoints for drone missions.
10. Compare fleet performance using makespan and energy consumption.

---

4. How the System Will Work

The complete system will follow this workflow:

Customer Orders → Order Prioritization → Drone Selection → Route Generation → Energy Calculation → Feasibility Check → Mission Assignment → Waypoint Output

Step 1: Order Ingestion

The system will accept delivery requests containing:

- Order ID
- Customer location
- Delivery deadline
- Payload weight
- Pickup/depot location

Step 2: Order Scheduling

All delivery requests will be inserted into a Min-Heap Priority Queue.

Orders will be prioritized using:

- Earliest delivery deadline
- Urgency
- Other scheduling criteria

The highest-priority feasible order will be selected for processing.

Step 3: Drone Selection

The system will examine the available drones and their current states.

It will consider:

- Current location
- Battery level
- Payload capacity
- Current mission status
- Ability to complete the requested delivery

Step 4: Route Generation

The airspace will be represented as a 3D directed weighted graph.

Graph nodes will represent possible locations/waypoints, while edges will represent possible flight paths.

The system will use Dijkstra's algorithm and A* to calculate suitable routes.

Step 5: Wind-Aware Route Calculation

The cost of travelling between two points will be dynamically calculated based on factors such as:

- Distance
- Drone movement
- Payload
- Wind velocity
- Energy requirement

The system will use the relationship:

Vg = Va + Vw

where ground velocity depends on the drone's air velocity and wind velocity.

Step 6: Mission Feasibility

Before assigning a route, the system will verify:

Battery feasibility + Payload feasibility + Time feasibility + Route feasibility

If the selected drone cannot complete the mission, another suitable drone will be considered.

Step 7: Mission Output

For every accepted mission, the system will generate:

- Assigned drone
- Delivery order
- Optimized route
- Waypoints
- Estimated energy requirement
- Battery feasibility
- Expected delivery information
- Mission status

---

5. Data Structures

Min-Heap

The Min-Heap will be used to prioritize delivery orders according to their deadlines and urgency.

Purpose: Efficient order scheduling.

Graph — Adjacency List

A directed weighted adjacency-list graph will represent the 3D airway network.

Purpose: Store possible flight paths and their associated costs.

Hash Table — unordered_map

An "unordered_map" will maintain the current state of every drone.

Purpose: Fast drone telemetry lookup with average O(1) access.

---

6. Algorithms

Earliest-Deadline-First (EDF)

EDF scheduling will prioritize orders with earlier deadlines.

Dijkstra's Algorithm

Dijkstra's algorithm will calculate minimum-cost routes through the weighted airway graph.

A* Algorithm

A* will provide heuristic-based pathfinding between the source and destination.

Dynamic Edge Weight Calculation

The weight of graph edges will be dynamically calculated according to factors such as distance, wind and energy requirements.

---

7. Drone Fleet Management

The system will maintain a fleet registry containing information about each drone.

A drone record may contain:

- Drone ID
- Current coordinates
- Battery State of Charge
- Maximum payload
- Current payload
- Mission status
- Availability

The Fleet Manager will continuously use this information when selecting drones for new missions.

---

8. Battery and Energy Management

The system will calculate whether a drone has sufficient energy to complete its assigned route.

Energy calculations will consider:

- Flight distance
- Payload weight
- Wind conditions
- Current battery level
- Estimated route energy requirement

If the estimated energy requirement exceeds the available battery capacity, the mission will be marked infeasible or another drone will be selected.

---

9. Payload Management

Every delivery order will contain a payload weight.

Before assignment, the system will compare:

Order Payload ≤ Drone Maximum Payload

If this condition is not satisfied, that drone will not be assigned to the order.

---

10. Deadline Management

The system will attempt to ensure that deliveries are completed within their specified time windows.

The scheduling system will prioritize urgent orders and the routing system will determine whether the selected drone can reach the destination within the required time.

---

11. System Architecture

The project will consist of the following major modules:

Order Ingestion Layer

Handles customer requests, coordinates, deadlines and payload information.

Scheduling & Dispatcher Engine

Uses the Min-Heap Priority Queue for deadline/urgency-based scheduling.

Algorithmic Routing Core

Contains the 3D graph, Dijkstra, A*, and dynamic edge-weight calculations.

Fleet Manager & Telemetry Hub

Maintains drone states using a Hash Table and performs battery/payload checks.

Mission Execution & Output

Generates waypoints and provides mission feasibility and performance information.

---

12. C++ Object-Oriented Design

The system will be developed using C++17 and Object-Oriented Programming.

The software can be organized into modules/classes such as:

- "Drone"
- "Order"
- "FleetManager"
- "Scheduler"
- "Graph"
- "RoutingEngine"
- "TelemetryManager"
- "Mission"

The modular design will allow individual components to communicate while keeping their responsibilities separate.

---

13. Input

The system will accept information such as:

- Number of drones
- Drone IDs
- Drone locations
- Battery levels
- Payload capacities
- Customer locations
- Delivery deadlines
- Order payload weights
- Airway network
- Wind conditions

---

14. Output

The system will provide:

- Order priority
- Assigned drone
- Selected route
- Route waypoints
- Estimated energy consumption
- Battery feasibility
- Payload feasibility
- Delivery feasibility
- Mission status
- Fleet makespan
- Energy-performance information

---

15. Example Scenario

Suppose there are 3 drones and 5 delivery orders.

Each order has a different:

- Destination
- Deadline
- Payload

The system will first place all orders into the priority queue.

It will select the most urgent order and examine the available drones.

If Drone 1 does not have enough battery, the system will check Drone 2 and Drone 3.

After selecting a feasible drone, the system will generate possible routes through the 3D airway graph.

Dijkstra or A* will then determine a suitable route while considering the route cost.

Finally, the system will verify battery, payload and deadline constraints before generating the mission.

---

16. Real-Life Applications

The proposed system can be used for:

- E-commerce delivery
- Medical and healthcare deliveries
- Emergency supply delivery
- Disaster relief
- Remote-area logistics
- Time-critical package transportation

The project proposal specifically identifies logistics/e-commerce operators, emergency/healthcare teams, fleet dispatchers, drone operators and customers as potential stakeholders.

---

17. Expected Benefits

The system will aim to provide:

- Faster delivery planning
- Better utilization of the drone fleet
- Deadline-aware scheduling
- Energy-efficient route selection
- Battery-aware mission planning
- Payload-aware drone assignment
- Dynamic wind-aware routing
- Efficient fleet-state management
- Reduced overall delivery makespan

---

18. Technology Stack

Programming Language: C++17
Data Structures: Min-Heap, Graph, Hash Table
Algorithms: EDF, Dijkstra, A*
STL: "priority_queue", "vector", "unordered_map"
Compiler: GCC / Clang
Platform: Linux
Build System: CMake

The project is specified around C++17, STL, GCC/Clang and CMake.

---

19. Future Development

The complete project will be developed in stages:

Phase-II

- Implement the core data structures.
- Develop the scheduling engine.
- Implement the graph and routing algorithms.
- Develop drone fleet management.
- Implement energy and wind calculations.
- Integrate the individual modules.

Phase-III

- Integrate the complete system.
- Test different fleet and delivery scenarios.
- Benchmark routing performance.
- Compare makespan and energy consumption.
- Generate final mission demonstrations.

---

20. Final Expected Outcome

The final project will provide a C++17 autonomous drone fleet routing and management system capable of:

Prioritizing orders → Selecting suitable drones → Finding optimized 3D routes → Considering wind and energy → Checking battery/payload/deadline constraints → Generating feasible delivery missions.

The intended final deliverables include the DSA modules, energy/wind computation routines, dynamic order assignment, 3D trajectory calculation and benchmarking datasets.

---

Conclusion

The Autonomous Drone Fleet Routing Management System will demonstrate the practical application of Data Structures, Algorithms and C++ OOP to autonomous logistics.

By combining Min-Heap scheduling, Hash Tables, Graphs, Dijkstra, A, dynamic edge weights, battery management and payload constraints*, the system will provide an integrated approach to managing multiple autonomous delivery drones efficiently.
