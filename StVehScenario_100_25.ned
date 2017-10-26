package com.liborio.sbrc2018;

import org.car2x.veins.nodes.RSU;
import org.car2x.veins.nodes.Scenario;

network StVehScenario_100_25 extends Scenario
{
	submodules:
		rsu[25] : RSU {
			@display("p=150,140;b=10,10,oval");
		}
}
