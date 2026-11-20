# Montreal City Scenario Source

The network, polygon layer, and source traffic routes in `city/montreal` were adapted from the user-provided local checkout `E:\sumo-traffic-simulation-main` (SUMO Traffic Simulation). Its license is Apache License 2.0.

SAMTRUCK additions are `city_traffic.rou.xml`, `montreal.sumocfg`, `vehicle_types.rou.xml`, and `samtruck/montreal_demo.py`. The original route file contained stale edge IDs for an earlier map revision, so the platoon route is computed from valid edges in the copied current network at runtime.
