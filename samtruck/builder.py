from __future__ import annotations

import subprocess
import sys
from pathlib import Path
import xml.etree.ElementTree as ET


from .config import SCENARIO_DIR, SUMO_HOME


def build_highway_city_network() -> Path:
    """
    Generates a custom SUMO network combining:
    1. A high-speed multi-lane highway section with incline/decline grades.
    2. A multi-lane interchange ramp.
    3. An urban city grid with 12 distinct traffic-light-controlled junctions (J1..J12).
    """
    SCENARIO_DIR.mkdir(parents=True, exist_ok=True)
    
    nodes_file = SCENARIO_DIR / "city_platoon.nod.xml"
    edges_file = SCENARIO_DIR / "city_platoon.edg.xml"
    conns_file = SCENARIO_DIR / "city_platoon.con.xml"
    net_file = SCENARIO_DIR / "city_platoon.net.xml"
    rou_file = SCENARIO_DIR / "city_platoon.rou.xml"
    poly_file = SCENARIO_DIR / "city_platoon.poly.xml"
    cfg_file = SCENARIO_DIR / "city_platoon.sumocfg"

    # --- 1. NODES DEFINITION ---
    # Highway Nodes (H0 to H4) & City Nodes (J1 to J12)
    nodes_content = """<?xml version="1.0" encoding="UTF-8"?>
<nodes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/nodes_file.xsd">
    <!-- Highway Nodes -->
    <node id="H0" x="0.0" y="500.0" z="0.0" type="priority"/>
    <node id="H1" x="500.0" y="500.0" z="15.0" type="priority"/>  <!-- Uphill grade: +3% -->
    <node id="H2" x="1000.0" y="500.0" z="35.0" type="priority"/> <!-- Uphill peak -->
    <node id="H3" x="1500.0" y="500.0" z="10.0" type="priority"/> <!-- Downhill slope -->
    <node id="H4" x="2000.0" y="500.0" z="0.0" type="priority"/>  <!-- Highway End / Offramp junction -->

    <!-- City Grid Nodes (3x4 Junction Grid: J1 to J12) -->
    <!-- Row 1 (y=400) -->
    <node id="J1" x="2000.0" y="400.0" z="0.0" type="traffic_light"/>
    <node id="J2" x="2300.0" y="400.0" z="0.0" type="traffic_light"/>
    <node id="J3" x="2600.0" y="400.0" z="0.0" type="traffic_light"/>
    <node id="J4" x="2900.0" y="400.0" z="0.0" type="traffic_light"/>

    <!-- Row 2 (y=200) -->
    <node id="J5" x="2000.0" y="200.0" z="0.0" type="traffic_light"/>
    <node id="J6" x="2300.0" y="200.0" z="0.0" type="traffic_light"/>
    <node id="J7" x="2600.0" y="200.0" z="0.0" type="traffic_light"/>
    <node id="J8" x="2900.0" y="200.0" z="0.0" type="traffic_light"/>

    <!-- Row 3 (y=0) -->
    <node id="J9"  x="2000.0" y="0.0" z="0.0" type="traffic_light"/>
    <node id="J10" x="2300.0" y="0.0" z="0.0" type="traffic_light"/>
    <node id="J11" x="2600.0" y="0.0" z="0.0" type="traffic_light"/>
    <node id="J12" x="2900.0" y="0.0" z="0.0" type="traffic_light"/>

    <!-- City Outlet Node -->
    <node id="C_OUT" x="3300.0" y="0.0" z="0.0" type="priority"/>
</nodes>
"""
    nodes_file.write_text(nodes_content, encoding="utf-8")

    # --- 2. EDGES DEFINITION ---
    edges_content = """<?xml version="1.0" encoding="UTF-8"?>
<edges xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/edges_file.xsd">
    <!-- Highway Edges (3 Lanes, 100 km/h = 27.78 m/s) -->
    <edge id="H_0_1" from="H0" to="H1" numLanes="3" speed="27.78"/>
    <edge id="H_1_2" from="H1" to="H2" numLanes="3" speed="27.78"/>
    <edge id="H_2_3" from="H2" to="H3" numLanes="3" speed="27.78"/>
    <edge id="H_3_4" from="H3" to="H4" numLanes="3" speed="27.78"/>

    <!-- Interchange Off-ramp from Highway H4 to City Entrance J1 -->
    <edge id="RAMP_H4_J1" from="H4" to="J1" numLanes="2" speed="20.0"/>

    <!-- City Horizontal Arterials (2 Lanes, 50 km/h = 13.89 m/s) -->
    <edge id="J1_J2" from="J1" to="J2" numLanes="2" speed="13.89"/>
    <edge id="J2_J3" from="J2" to="J3" numLanes="2" speed="13.89"/>
    <edge id="J3_J4" from="J3" to="J4" numLanes="2" speed="13.89"/>

    <edge id="J5_J6" from="J5" to="J6" numLanes="2" speed="13.89"/>
    <edge id="J6_J7" from="J6" to="J7" numLanes="2" speed="13.89"/>
    <edge id="J7_J8" from="J7" to="J8" numLanes="2" speed="13.89"/>

    <edge id="J9_J10" from="J9" to="J10" numLanes="2" speed="13.89"/>
    <edge id="J10_J11" from="J10" to="J11" numLanes="2" speed="13.89"/>
    <edge id="J11_J12" from="J11" to="J12" numLanes="2" speed="13.89"/>

    <!-- City Vertical Avenues (2 Lanes) -->
    <edge id="J1_J5" from="J1" to="J5" numLanes="2" speed="13.89"/>
    <edge id="J5_J9" from="J5" to="J9" numLanes="2" speed="13.89"/>

    <edge id="J2_J6" from="J2" to="J6" numLanes="2" speed="13.89"/>
    <edge id="J6_J10" from="J6" to="J10" numLanes="2" speed="13.89"/>

    <edge id="J3_J7" from="J3" to="J7" numLanes="2" speed="13.89"/>
    <edge id="J7_J11" from="J7" to="J11" numLanes="2" speed="13.89"/>

    <edge id="J4_J8" from="J4" to="J8" numLanes="2" speed="13.89"/>
    <edge id="J8_J12" from="J8" to="J12" numLanes="2" speed="13.89"/>

    <!-- City Outlet -->
    <edge id="J12_COUT" from="J12" to="C_OUT" numLanes="2" speed="13.89"/>
</edges>
"""
    edges_file.write_text(edges_content, encoding="utf-8")

    # --- 3. NETCONVERT INVOCATION ---
    netconvert_bin = SUMO_HOME / "bin" / "netconvert.exe"
    cmd = [
        str(netconvert_bin),
        "--node-files", str(nodes_file),
        "--edge-files", str(edges_file),
        "--output-file", str(net_file),
        "--sidewalks.guess", "true",
        "--crossings.guess", "true",
    ]
    subprocess.run(cmd, check=True, capture_output=True, text=True)

    # --- 4. ROUTE FILE DEFINITION WITH HEAVY TRUCKS ---
    rou_content = """<?xml version="1.0" encoding="UTF-8"?>
<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">
    <!-- Heavy Truck Vehicle Type (40-ton articulated HGV) -->
    <vType id="HGV" accel="1.2" decel="4.5" emergencyDecel="7.0" length="16.5" minGap="3.0" maxSpeed="30.0"
           guiShape="truck/semitrailer" color="0,102,204" vClass="truck"/>
    
    <!-- Secondary City Traffic Vehicle Type -->
    <vType id="car" accel="2.6" decel="4.5" length="4.5" minGap="2.0" maxSpeed="20.0"
           guiShape="passenger" color="200,200,200" vClass="passenger"/>

    <!-- Main Highway-to-City Platoon Route -->
    <route id="platoon_route" edges="H_0_1 H_1_2 H_2_3 H_3_4 RAMP_H4_J1 J1_J5 J5_J6 J6_J7 J7_J11 J11_J12 J12_COUT"/>

    <!-- Cross City Traffic Routes for realism -->
    <route id="city_cross_1" edges="J9_J10 J10_J11 J11_J12 J12_COUT"/>
    <route id="city_cross_2" edges="J2_J6 J6_J10"/>

    <!-- Platoon Vehicles (Leader + 2 Followers) -->
    <vehicle id="truck_0" type="HGV" route="platoon_route" depart="0.0" departPos="120.0" departSpeed="18.0"/>
    <vehicle id="truck_1" type="HGV" route="platoon_route" depart="0.0" departPos="80.0"  departSpeed="18.0"/>
    <vehicle id="truck_2" type="HGV" route="platoon_route" depart="0.0" departPos="40.0"  departSpeed="18.0"/>

    <!-- Background Ambient Traffic -->
    <flow id="bg_car_1" type="car" route="city_cross_1" begin="5.0" end="80.0" number="10"/>
    <flow id="bg_car_2" type="car" route="city_cross_2" begin="10.0" end="80.0" number="8"/>
</routes>
"""
    rou_file.write_text(rou_content, encoding="utf-8")

    # --- 5. CITY BUILDINGS & POLYGONS ---
    poly_content = """<?xml version="1.0" encoding="UTF-8"?>
<additional xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/additional_file.xsd">
    <!-- City Buildings & Parks along the grid -->
    <poly id="bldg_1" type="building" color="180,180,200" fill="true" shape="2050,380 2250,380 2250,220 2050,220"/>
    <poly id="bldg_2" type="building" color="160,160,180" fill="true" shape="2350,380 2550,380 2550,220 2350,220"/>
    <poly id="park_1" type="park" color="100,200,100" fill="true" shape="2650,380 2850,380 2850,220 2650,220"/>
    <poly id="bldg_3" type="building" color="170,170,190" fill="true" shape="2050,180 2250,180 2250,20 2050,20"/>
    <poly id="bldg_4" type="building" color="190,190,210" fill="true" shape="2350,180 2550,180 2550,20 2350,20"/>
</additional>
"""
    poly_file.write_text(poly_content, encoding="utf-8")

    # --- 6. SUMO CONFIG FILE ---
    cfg_content = """<?xml version="1.0" encoding="UTF-8"?>
<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">
    <input>
        <net-file value="city_platoon.net.xml"/>
        <route-files value="city_platoon.rou.xml"/>
        <additional-files value="city_platoon.poly.xml"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="120"/>
        <step-length value="0.1"/>
    </time>
    <processing>
        <collision.action value="warn"/>
        <time-to-teleport value="-1"/>
    </processing>
    <report>
        <no-step-log value="true"/>
        <duration-log.disable value="true"/>
    </report>
</configuration>
"""
    cfg_file.write_text(cfg_content, encoding="utf-8")

    print(f"Custom Highway + 12-Junction City Network built successfully at {net_file}")
    return net_file


if __name__ == "__main__":
    build_highway_city_network()
