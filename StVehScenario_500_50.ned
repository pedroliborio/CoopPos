package com.liborio.sbrc2018;

import org.car2x.veins.nodes.RSU;
import org.car2x.veins.nodes.Scenario;

network StVehScenario_500_50 extends Scenario
{
	submodules:
		rsu[250] : RSU {
			@display("p=150,140;b=10,10,oval");
		}
}
