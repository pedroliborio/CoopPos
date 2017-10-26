package com.liborio.sbrc2018;

import org.car2x.veins.nodes.RSU;
import org.car2x.veins.nodes.Scenario;

network StVehScenario_100_50 extends Scenario
{
	submodules:
		rsu[50] : RSU {
			@display("p=150,140;b=10,10,oval");
		}
}
